"""Experimental bounded XL330 human-guided alignment safety core.

Ported from the project-local GELLO-ALIGN-A5 implementation. Legacy one-joint
probes and autonomous homing/force-ramp entry points are intentionally omitted.
Never used by the default read-only collector. See SAFETY.md and docs/assisted.md.
"""
from __future__ import annotations
import fcntl
import os
import select
import termios
import time
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, Iterable, Mapping
import numpy as np
from .calibration import GelloCalibration
from .dynamixel import COUNTS_PER_REVOLUTION, PROTOCOL2_HEADER, STATUS, protocol2_crc


PING = 0x01


READ = 0x02


WRITE = 0x03


SYNC_READ = 0x82


SYNC_WRITE = 0x83


BROADCAST_ID = 0xFE


CURRENT_BASED_POSITION_MODE = 5


ACTUATION_PROTOCOL_VERSION = "GELLO-ACT-P0-v3"


HOMING_PROTOCOL_VERSION = "GELLO-HOME-P1"


ALIGNMENT_ASSIST_PROTOCOL_VERSION = "GELLO-ALIGN-A4"


XL330_GOAL_POSITION_MIN = -1_048_575


XL330_GOAL_POSITION_MAX = 1_048_575


XL330_VELOCITY_UNIT_RAD_S = 0.229 * 2.0 * np.pi / 60.0


class GelloActuationError(RuntimeError):
    """Base class for fail-closed GELLO actuation errors."""


class GelloActuationSafetyError(GelloActuationError):
    """Raised when a configured hardware safety gate is violated."""


class GelloActuationShutdownUnverified(GelloActuationError):
    """Raised when software cannot prove that every motor torque is disabled."""


class GelloTrackingError(GelloActuationSafetyError):
    """Existing tracking cap; A6 may release early while the operator backdrives."""


class HomingState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    READ_ONLY_PREFLIGHT = "READ_ONLY_PREFLIGHT"
    SAFE_OFF_VERIFIED = "SAFE_OFF_VERIFIED"
    CONFIGURE_ZERO_GAIN = "CONFIGURE_ZERO_GAIN"
    ZERO_GOAL_LATCHED = "ZERO_GOAL_LATCHED"
    TORQUE_ON_ZERO_GAIN = "TORQUE_ON_ZERO_GAIN"
    POST_ENABLE_RELATCHED = "POST_ENABLE_RELATCHED"
    APPROACHING = "APPROACHING"
    HOLDING_AT_TARGET = "HOLDING_AT_TARGET"
    SOFTENING = "SOFTENING"
    RELEASED_FOR_TELEOP = "RELEASED_FOR_TELEOP"
    TORQUE_OFF_VERIFIED = "TORQUE_OFF_VERIFIED"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"
    FAULT_UNVERIFIED = "FAULT_UNVERIFIED"


@dataclass(frozen=True)
class ControlItem:
    name: str
    address: int
    length: int
    signed: bool = False


READ_ITEMS: dict[str, ControlItem] = {
    item.name: item
    for item in (
        ControlItem("model_number", 0, 2),
        ControlItem("firmware_version", 6, 1),
        ControlItem("drive_mode", 10, 1),
        ControlItem("operating_mode", 11, 1),
        ControlItem("current_limit_ma", 38, 2),
        ControlItem("startup_configuration", 60, 1),
        ControlItem("shutdown_mask", 63, 1),
        ControlItem("torque_enable", 64, 1),
        ControlItem("hardware_error", 70, 1),
        ControlItem("position_d_gain", 80, 2),
        ControlItem("position_i_gain", 82, 2),
        ControlItem("position_p_gain", 84, 2),
        ControlItem("bus_watchdog", 98, 1, signed=True),
        ControlItem("goal_current_ma", 102, 2, signed=True),
        ControlItem("goal_position_counts", 116, 4, signed=True),
        ControlItem("present_current_ma", 126, 2, signed=True),
        ControlItem("present_velocity_raw", 128, 4, signed=True),
        ControlItem("present_position_counts", 132, 4, signed=True),
        ControlItem("input_voltage_raw", 144, 2),
        ControlItem("temperature_c", 146, 1),
    )
}


ACTIVE_MONITOR_BLOCK = ControlItem("active_monitor_block", 64, 83)


ACTIVE_MONITOR_ITEM_NAMES = (
    "torque_enable",
    "hardware_error",
    "position_d_gain",
    "position_i_gain",
    "position_p_gain",
    "bus_watchdog",
    "goal_current_ma",
    "goal_position_counts",
    "present_current_ma",
    "present_velocity_raw",
    "present_position_counts",
    "input_voltage_raw",
    "temperature_c",
)


WRITE_ITEMS: dict[str, ControlItem] = {
    name: READ_ITEMS[name]
    for name in (
        "operating_mode",
        "torque_enable",
        "position_d_gain",
        "position_i_gain",
        "position_p_gain",
        "bus_watchdog",
        "goal_current_ma",
        "goal_position_counts",
    )
}


def _stuff_protocol2(data: bytes) -> bytes:
    """Apply Protocol 2.0 byte stuffing to instruction/status payload bytes."""

    stuffed = bytearray()
    for value in data:
        stuffed.append(value)
        if len(stuffed) >= 3 and stuffed[-3:] == b"\xff\xff\xfd":
            stuffed.append(0xFD)
    return bytes(stuffed)


def _unstuff_protocol2(data: bytes) -> bytes:
    unstuffed = bytearray()
    index = 0
    while index < len(data):
        if data[index : index + 4] == b"\xff\xff\xfd\xfd":
            unstuffed.extend(b"\xff\xff\xfd")
            index += 4
        else:
            unstuffed.append(data[index])
            index += 1
    return bytes(unstuffed)


def actuation_protocol2_packet(
    motor_id: int, instruction: int, parameters: bytes = b""
) -> bytes:
    """Build one Protocol 2.0 packet using only the P0 register allowlists."""

    if not (0 <= int(motor_id) <= 252 or int(motor_id) == BROADCAST_ID):
        raise ValueError("Dynamixel motor ID must be in [0, 252] or broadcast 254")
    if int(instruction) not in (PING, READ, WRITE, SYNC_READ, SYNC_WRITE):
        raise ValueError("GELLO actuation packet instruction is not allowlisted")
    if int(instruction) in (SYNC_READ, SYNC_WRITE) and int(motor_id) != BROADCAST_ID:
        raise ValueError("Dynamixel SYNC instructions require the broadcast ID")
    if int(instruction) in (PING, READ, WRITE) and int(motor_id) == BROADCAST_ID:
        raise ValueError("broadcast PING/READ/WRITE is forbidden by GELLO-ACT-P0")
    parameters = bytes(parameters)
    if int(instruction) == PING and parameters:
        raise ValueError("Dynamixel PING does not accept parameters")
    if int(instruction) in (READ, WRITE):
        if len(parameters) < 2:
            raise ValueError("typed Dynamixel packet is missing its control-item address")
        address = int.from_bytes(parameters[:2], "little")
        if int(instruction) == READ:
            if len(parameters) != 4:
                raise ValueError("typed Dynamixel packet control item is not allowlisted")
            length = int.from_bytes(parameters[2:4], "little")
            read_specs = tuple(READ_ITEMS.values()) + (ACTIVE_MONITOR_BLOCK,)
            matches = [
                item
                for item in read_specs
                if item.address == address and item.length == length
            ]
        else:
            matches = [
                item
                for item in WRITE_ITEMS.values()
                if item.address == address and len(parameters) == 2 + item.length
            ]
        if len(matches) != 1:
            raise ValueError("typed Dynamixel packet control item is not allowlisted")
    if int(instruction) == SYNC_READ:
        if len(parameters) < 5:
            raise ValueError("Dynamixel SYNC_READ requires an item and motor IDs")
        address = int.from_bytes(parameters[:2], "little")
        length = int.from_bytes(parameters[2:4], "little")
        matches = [
            item
            for item in tuple(READ_ITEMS.values()) + (ACTIVE_MONITOR_BLOCK,)
            if item.address == address and item.length == length
        ]
        ids = tuple(int(value) for value in parameters[4:])
        if len(matches) != 1 or len(set(ids)) != len(ids) or any(
            not 0 <= value <= 252 for value in ids
        ):
            raise ValueError("Dynamixel SYNC_READ item or motor IDs are not allowlisted")
    if int(instruction) == SYNC_WRITE:
        item = WRITE_ITEMS["goal_position_counts"]
        row_length = 1 + item.length
        if (
            len(parameters) < 4 + row_length
            or int.from_bytes(parameters[:2], "little") != item.address
            or int.from_bytes(parameters[2:4], "little") != item.length
            or (len(parameters) - 4) % row_length != 0
        ):
            raise ValueError("GELLO-ACT-P0 SYNC_WRITE permits only Goal Position rows")
        ids = tuple(
            int(parameters[index])
            for index in range(4, len(parameters), row_length)
        )
        if (
            len(ids) != 1
            or len(set(ids)) != len(ids)
            or any(not 0 <= value <= 252 for value in ids)
        ):
            raise ValueError(
                "GELLO-ACT-P0 SYNC_WRITE requires exactly one selected motor"
            )
    stuffed_payload = _stuff_protocol2(bytes((int(instruction),)) + bytes(parameters))
    body = bytes((int(motor_id),)) + (len(stuffed_payload) + 2).to_bytes(2, "little")
    packet = PROTOCOL2_HEADER + body + stuffed_payload
    return packet + protocol2_crc(packet).to_bytes(2, "little")


def _integer_bytes(value: int, item: ControlItem) -> bytes:
    bits = 8 * int(item.length)
    if item.signed:
        minimum, maximum = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    else:
        minimum, maximum = 0, (1 << bits) - 1
    value = int(value)
    if value < minimum or value > maximum:
        raise ValueError(
            f"{item.name} value {value} is outside [{minimum}, {maximum}]"
        )
    return value.to_bytes(item.length, "little", signed=item.signed)


def _decode_active_monitor_block(payload: bytes) -> dict[str, int]:
    if len(payload) != ACTIVE_MONITOR_BLOCK.length:
        raise IOError(
            "Dynamixel active-monitor block has an unexpected response length"
        )
    decoded: dict[str, int] = {}
    for name in ACTIVE_MONITOR_ITEM_NAMES:
        item = READ_ITEMS[name]
        offset = int(item.address - ACTIVE_MONITOR_BLOCK.address)
        if offset < 0 or offset + item.length > len(payload):
            raise AssertionError(f"active-monitor item {name} is outside the block")
        decoded[name] = int.from_bytes(
            payload[offset : offset + item.length],
            "little",
            signed=item.signed,
        )
    return decoded


class SafeXL330Transport:
    """Exclusive serial transport with typed, register-allowlisted XL330 writes.

    This class intentionally exposes no public arbitrary-address write method.
    Every mutation is routed through a semantic method below.
    """

    _baud_constants = {57600: termios.B57600, 115200: termios.B115200}

    def __init__(
        self,
        port: str,
        *,
        baudrate: int = 57600,
        timeout_seconds: float = 0.12,
    ):
        if int(baudrate) not in self._baud_constants:
            raise ValueError(f"unsupported GELLO actuation baud rate: {baudrate}")
        self.requested_port = str(port)
        self.resolved_port = str(Path(port).resolve(strict=True))
        self.timeout_seconds = float(timeout_seconds)
        self.fd = os.open(self.resolved_port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.command_log: list[dict] = []
        self.motor_register_writes = 0
        self.write_packets = 0
        self.instruction_counts = {
            "PING": 0,
            "READ": 0,
            "SYNC_READ": 0,
            "WRITE": 0,
            "SYNC_WRITE": 0,
        }
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            attributes = termios.tcgetattr(self.fd)
            attributes[0] = 0
            attributes[1] = 0
            attributes[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attributes[3] = 0
            attributes[4] = self._baud_constants[int(baudrate)]
            attributes[5] = self._baud_constants[int(baudrate)]
            attributes[6][termios.VMIN] = 0
            attributes[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attributes)
            os.set_blocking(self.fd, False)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except Exception:
            os.close(self.fd)
            self.fd = -1
            raise

    def close(self) -> None:
        if self.fd >= 0:
            try:
                fcntl.flock(self.fd, fcntl.LOCK_UN)
            finally:
                os.close(self.fd)
                self.fd = -1

    def __enter__(self) -> "SafeXL330Transport":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def _receive_status_packets(
        self,
        expected_ids: Iterable[int],
        *,
        timeout_seconds: float | None = None,
    ) -> dict[int, bytes]:
        expected = tuple(int(value) for value in expected_ids)
        if not expected or len(set(expected)) != len(expected):
            raise ValueError("expected Dynamixel response IDs must be unique and nonempty")
        receive_timeout = (
            float(self.timeout_seconds)
            if timeout_seconds is None
            else float(timeout_seconds)
        )
        if not np.isfinite(receive_timeout) or receive_timeout <= 0.0:
            raise ValueError("Dynamixel receive timeout must be positive and finite")
        remaining = set(expected)
        responses: dict[int, bytes] = {}
        deadline = time.monotonic() + receive_timeout
        buffer = bytearray()
        while remaining and time.monotonic() < deadline:
            readable, _, _ = select.select(
                [self.fd], [], [], max(0.0, deadline - time.monotonic())
            )
            if not readable:
                break
            chunk = os.read(self.fd, 512)
            if chunk:
                buffer.extend(chunk)
            while True:
                header_index = buffer.find(PROTOCOL2_HEADER)
                if header_index < 0:
                    if len(buffer) > 3:
                        del buffer[:-3]
                    break
                if header_index:
                    del buffer[:header_index]
                if len(buffer) < 7:
                    break
                packet_length = int.from_bytes(buffer[5:7], "little")
                total_length = 7 + packet_length
                if len(buffer) < total_length:
                    break
                packet = bytes(buffer[:total_length])
                del buffer[:total_length]
                if protocol2_crc(packet[:-2]) != int.from_bytes(packet[-2:], "little"):
                    raise IOError("Dynamixel status packet CRC mismatch")
                motor_id = int(packet[4])
                if motor_id not in remaining:
                    raise IOError(
                        f"unexpected or duplicate Dynamixel status ID {motor_id}; "
                        f"waiting for {sorted(remaining)}"
                    )
                payload = _unstuff_protocol2(packet[7:-2])
                if len(payload) < 2 or payload[0] != STATUS:
                    raise IOError("unexpected Dynamixel status instruction")
                error = int(payload[1])
                if error:
                    raise IOError(
                        f"Dynamixel ID {motor_id} returned status error 0x{error:02x}"
                    )
                responses[motor_id] = bytes(payload[2:])
                remaining.remove(motor_id)
        if remaining:
            raise TimeoutError(
                f"Dynamixel IDs {sorted(remaining)} did not answer within "
                f"{receive_timeout}s"
            )
        return responses

    def _receive_packet(self, expected_id: int) -> tuple[int, bytes]:
        response = self._receive_status_packets((int(expected_id),))[int(expected_id)]
        return 0, response

    def _unicast(
        self, motor_id: int, instruction: int, parameters: bytes = b""
    ) -> tuple[int, bytes]:
        termios.tcflush(self.fd, termios.TCIFLUSH)
        packet = actuation_protocol2_packet(motor_id, instruction, parameters)
        instruction_name = {PING: "PING", READ: "READ", WRITE: "WRITE"}.get(
            int(instruction)
        )
        if instruction_name is None:
            raise ValueError("unicast instruction counter received an unsupported opcode")
        self.instruction_counts[instruction_name] += 1
        written = os.write(self.fd, packet)
        if written != len(packet):
            raise IOError("short Dynamixel serial write")
        termios.tcdrain(self.fd)
        error, response = self._receive_packet(int(motor_id))
        if error:
            raise IOError(
                f"Dynamixel ID {motor_id} returned status error 0x{error:02x}"
            )
        return error, response

    def ping(self, motor_id: int) -> dict:
        _, parameters = self._unicast(int(motor_id), PING)
        if len(parameters) != 3:
            raise IOError(f"malformed Dynamixel PING response from ID {motor_id}")
        return {
            "id": int(motor_id),
            "model_number": int.from_bytes(parameters[:2], "little"),
            "firmware_version": int(parameters[2]),
        }

    def read_control_item(self, motor_id: int, name: str) -> int:
        if name not in READ_ITEMS:
            raise ValueError(f"Dynamixel read item is not allowlisted: {name}")
        item = READ_ITEMS[name]
        parameters = item.address.to_bytes(2, "little") + item.length.to_bytes(
            2, "little"
        )
        _, response = self._unicast(int(motor_id), READ, parameters)
        if len(response) != item.length:
            raise IOError(
                f"Dynamixel ID {motor_id} returned {len(response)} bytes for {name}, "
                f"expected {item.length}"
            )
        return int.from_bytes(response, "little", signed=item.signed)

    def read_active_snapshot(self, motor_id: int) -> dict[str, int]:
        item = ACTIVE_MONITOR_BLOCK
        parameters = item.address.to_bytes(2, "little") + item.length.to_bytes(
            2, "little"
        )
        _, response = self._unicast(int(motor_id), READ, parameters)
        return _decode_active_monitor_block(response)

    def sync_read_active_snapshots(
        self, motor_ids: Iterable[int]
    ) -> dict[int, dict[str, int]]:
        """Read the complete active-monitor RAM block for several motors once.

        Seven sequential 83-byte READ transactions measured about 256 ms on the
        physical GELLO and made the nominal 5 Hz A2 monitor fail every cycle.
        One SYNC_READ preserves the exact register evidence while removing six
        request/response round trips.  The separate all-ID torque SYNC_READ in
        the controller is retained so the gripper and every dormant ID remain
        fail-closed.
        """

        ids = tuple(int(value) for value in motor_ids)
        if not ids or len(set(ids)) != len(ids) or any(
            value == BROADCAST_ID or not 0 <= value <= 252 for value in ids
        ):
            raise ValueError("Dynamixel active snapshot IDs must be unique and valid")
        item = ACTIVE_MONITOR_BLOCK
        parameters = (
            item.address.to_bytes(2, "little")
            + item.length.to_bytes(2, "little")
            + bytes(ids)
        )
        termios.tcflush(self.fd, termios.TCIFLUSH)
        packet = actuation_protocol2_packet(BROADCAST_ID, SYNC_READ, parameters)
        self.instruction_counts["SYNC_READ"] += 1
        written = os.write(self.fd, packet)
        if written != len(packet):
            raise IOError("short Dynamixel active-monitor SYNC_READ")
        termios.tcdrain(self.fd)
        # Seven 83-byte status packets require about 114 ms of on-wire receive
        # time at 57,600 baud. The normal 120 ms unicast timeout therefore
        # leaves no practical USB/adapter margin. A4 permits 180 ms only for
        # this aggregate response; the controller's independent 200 ms cycle
        # deadline and 400 ms Bus Watchdog remain unchanged and fail closed.
        responses = self._receive_status_packets(
            ids, timeout_seconds=max(float(self.timeout_seconds), 0.18)
        )
        return {
            motor_id: _decode_active_monitor_block(responses[motor_id])
            for motor_id in ids
        }

    def sync_read_control_item(
        self, name: str, motor_ids: Iterable[int]
    ) -> dict[int, int]:
        if name not in READ_ITEMS:
            raise ValueError(f"Dynamixel sync-read item is not allowlisted: {name}")
        ids = tuple(int(value) for value in motor_ids)
        if not ids or len(set(ids)) != len(ids) or any(
            value == BROADCAST_ID or not 0 <= value <= 252 for value in ids
        ):
            raise ValueError("Dynamixel sync-read motor IDs must be unique and valid")
        item = READ_ITEMS[name]
        parameters = (
            item.address.to_bytes(2, "little")
            + item.length.to_bytes(2, "little")
            + bytes(ids)
        )
        termios.tcflush(self.fd, termios.TCIFLUSH)
        packet = actuation_protocol2_packet(BROADCAST_ID, SYNC_READ, parameters)
        self.instruction_counts["SYNC_READ"] += 1
        written = os.write(self.fd, packet)
        if written != len(packet):
            raise IOError("short Dynamixel SYNC_READ")
        termios.tcdrain(self.fd)
        responses = self._receive_status_packets(ids)
        result: dict[int, int] = {}
        for motor_id in ids:
            response = responses[motor_id]
            if len(response) != item.length:
                raise IOError(
                    f"Dynamixel ID {motor_id} returned {len(response)} bytes for "
                    f"SYNC_READ {name}, expected {item.length}"
                )
            result[motor_id] = int.from_bytes(
                response, "little", signed=item.signed
            )
        return result

    def _write_control_item(self, motor_id: int, name: str, value: int) -> None:
        if name not in WRITE_ITEMS:
            raise ValueError(f"Dynamixel write item is not allowlisted: {name}")
        item = WRITE_ITEMS[name]
        parameters = item.address.to_bytes(2, "little") + _integer_bytes(value, item)
        entry = {
            "instruction": "WRITE",
            "motor_ids": [int(motor_id)],
            "item": name,
            "address": int(item.address),
            "value": int(value),
            "acknowledged": False,
        }
        # Count and persist the attempt before transmission.  A motor may have
        # applied a packet even when its status response is lost, so recording
        # only acknowledged writes would make failure artifacts dangerously
        # optimistic.
        self.motor_register_writes += 1
        self.write_packets += 1
        self.command_log.append(entry)
        self._unicast(int(motor_id), WRITE, parameters)
        entry["acknowledged"] = True

    def set_operating_mode_current_based_position(self, motor_id: int) -> None:
        self._write_control_item(motor_id, "operating_mode", CURRENT_BASED_POSITION_MODE)

    def set_torque_enabled(self, motor_id: int, enabled: bool) -> None:
        self._write_control_item(motor_id, "torque_enable", int(bool(enabled)))

    def set_position_gains(self, motor_id: int, *, p: int, i: int, d: int) -> None:
        # P is written last so a zero-P motor does not produce spring force while
        # the remaining coefficients are being configured.
        self._write_control_item(motor_id, "position_d_gain", int(d))
        self._write_control_item(motor_id, "position_i_gain", int(i))
        self._write_control_item(motor_id, "position_p_gain", int(p))

    def set_bus_watchdog_ticks(self, motor_id: int, ticks: int) -> None:
        self._write_control_item(motor_id, "bus_watchdog", int(ticks))

    def set_goal_current_ma(self, motor_id: int, current_ma: int) -> None:
        self._write_control_item(motor_id, "goal_current_ma", int(current_ma))

    def set_goal_position_counts(self, motor_id: int, counts: int) -> None:
        counts = int(counts)
        if counts < XL330_GOAL_POSITION_MIN or counts > XL330_GOAL_POSITION_MAX:
            raise ValueError("XL330 current-based Goal Position is outside mode-5 range")
        self._write_control_item(motor_id, "goal_position_counts", counts)

    def set_goal_positions_counts(self, goals: Mapping[int, int]) -> None:
        """SYNC_WRITE only Goal Position values; broadcast torque-enable is forbidden."""

        if len(goals) != 1:
            raise ValueError(
                "GELLO-ACT-P0 Goal Position SYNC_WRITE requires exactly one motor"
            )
        if len(set(int(key) for key in goals)) != len(goals):
            raise ValueError("Goal Position SYNC_WRITE motor IDs must be unique")
        item = WRITE_ITEMS["goal_position_counts"]
        parameters = item.address.to_bytes(2, "little") + item.length.to_bytes(
            2, "little"
        )
        normalized: dict[int, int] = {}
        for motor_id, value in goals.items():
            motor_id = int(motor_id)
            value = int(value)
            if motor_id == BROADCAST_ID or not 0 <= motor_id <= 252:
                raise ValueError("invalid Goal Position motor ID")
            if value < XL330_GOAL_POSITION_MIN or value > XL330_GOAL_POSITION_MAX:
                raise ValueError("XL330 Goal Position is outside mode-5 range")
            normalized[motor_id] = value
            parameters += bytes((motor_id,)) + _integer_bytes(value, item)
        packet = actuation_protocol2_packet(BROADCAST_ID, SYNC_WRITE, parameters)
        entry = {
            "instruction": "SYNC_WRITE",
            "motor_ids": sorted(normalized),
            "item": "goal_position_counts",
            "address": int(item.address),
            "values": {str(key): value for key, value in normalized.items()},
            "transmission_complete": False,
            "status_return_expected": False,
        }
        self.instruction_counts["SYNC_WRITE"] += 1
        self.motor_register_writes += len(normalized)
        self.write_packets += 1
        self.command_log.append(entry)
        written = os.write(self.fd, packet)
        if written != len(packet):
            raise IOError("short Dynamixel Goal Position SYNC_WRITE")
        termios.tcdrain(self.fd)
        entry["transmission_complete"] = True


@dataclass(frozen=True)
class GelloAlignmentPlan:
    target_joint_positions: tuple[float, ...]
    reference_present_position_counts: tuple[int, ...]
    one_turn_goal_counts: tuple[int, ...]
    continuous_goal_counts: tuple[int, ...]
    delta_counts: tuple[int, ...]
    roundtrip_joint_positions: tuple[float, ...]
    roundtrip_errors_rad: tuple[float, ...]
    guarded_joint_limits_ok: tuple[bool, ...]
    p0_selected_motor_ids: tuple[int, ...]
    p0_execution_eligible: bool
    p0_block_reasons: tuple[str, ...]

    def to_dict(self) -> dict:
        return asdict(self)


def plan_follower_joint_alignment(
    calibration: GelloCalibration,
    follower_joint_positions: Iterable[float],
    reference_present_position_counts: Iterable[int],
    *,
    selected_motor_ids: Iterable[int] = (),
    limits: _LifecycleLimits | None = None,
) -> GelloAlignmentPlan:
    """Invert GELLO calibration and select the nearest continuous encoder branch.

    The returned seven-joint plan is always useful as a dry-run diagnostic.  Its
    ``p0_execution_eligible`` flag is much narrower: exactly one selected arm
    motor, no gripper, and no more than the registered eight-pulse transition.
    """

    target = np.asarray(tuple(follower_joint_positions), dtype=np.float64)
    reference = np.asarray(tuple(reference_present_position_counts), dtype=np.int64)
    selected = tuple(int(value) for value in selected_motor_ids)
    if target.shape != (7,) or reference.shape != (7,):
        raise ValueError("GELLO alignment planning requires seven targets and references")
    if not np.isfinite(target).all():
        raise ValueError("GELLO alignment target joints must be finite")
    if len(set(selected)) != len(selected):
        raise ValueError("GELLO alignment selected motor IDs must be unique")
    arm_ids = tuple(int(value) for value in calibration.arm_ids)
    if any(motor_id not in arm_ids for motor_id in selected):
        raise ValueError("GELLO alignment may select arm motor IDs only")
    joint_limits = np.asarray(calibration.joint_position_limits, dtype=np.float64)
    margin = 0.0 if limits is None else float(getattr(limits, "joint_limit_margin_rad", 0.0))
    guarded = np.logical_and(
        target >= joint_limits[:, 0] + margin,
        target <= joint_limits[:, 1] - margin,
    )
    offsets = np.asarray(calibration.assembly_offsets_radians, dtype=np.float64)
    signs = np.asarray(calibration.joint_signs, dtype=np.int64)
    raw_angles = offsets + signs.astype(np.float64) * target
    one_turn = (
        np.rint(
            np.mod(raw_angles, 2.0 * np.pi)
            * COUNTS_PER_REVOLUTION
            / (2.0 * np.pi)
        ).astype(np.int64)
        % COUNTS_PER_REVOLUTION
    )
    branches = np.rint((reference - one_turn) / COUNTS_PER_REVOLUTION).astype(np.int64)
    continuous = one_turn + branches * COUNTS_PER_REVOLUTION
    if np.any(continuous < XL330_GOAL_POSITION_MIN) or np.any(
        continuous > XL330_GOAL_POSITION_MAX
    ):
        raise ValueError("nearest GELLO alignment branch exceeds XL330 mode-5 range")
    roundtrip = calibration.mapped_joint_positions(continuous % COUNTS_PER_REVOLUTION)
    roundtrip_error = roundtrip - target
    if float(np.abs(roundtrip_error).max()) > np.pi / COUNTS_PER_REVOLUTION + 1e-12:
        raise ValueError("GELLO inverse alignment does not round-trip within one encoder half-step")
    delta = continuous - reference
    block_reasons: list[str] = []
    if not bool(np.all(guarded)):
        block_reasons.append("target_outside_guarded_panda_limits")
    if limits is None:
        block_reasons.append("p0_limits_not_supplied")
    elif not hasattr(limits, "maximum_enabled_arm_joints"):
        # _LifecycleLimits (GELLO-HOME-P1) intentionally has no one-joint P0
        # envelope; the seven-joint diagnostic only needs
        # guarded_joint_limits_ok and continuous_goal_counts, so the
        # P0-specific block reasons below do not apply to a non-P0 limits
        # object and are intentionally left empty here.
        pass
    else:
        if len(selected) != int(limits.maximum_enabled_arm_joints):
            block_reasons.append("p0_requires_exactly_one_selected_arm_motor")
        selected_indices = [arm_ids.index(motor_id) for motor_id in selected]
        if selected_indices and any(
            abs(int(delta[index])) > int(limits.maximum_total_transition_counts)
            for index in selected_indices
        ):
            block_reasons.append("selected_transition_exceeds_p0_total_pulse_cap")
        if any(
            int(delta[index]) != 0
            for index in range(7)
            if index not in selected_indices
        ):
            block_reasons.append("unselected_joint_transition_is_nonzero")
    return GelloAlignmentPlan(
        target_joint_positions=tuple(float(value) for value in target),
        reference_present_position_counts=tuple(int(value) for value in reference),
        one_turn_goal_counts=tuple(int(value) for value in one_turn),
        continuous_goal_counts=tuple(int(value) for value in continuous),
        delta_counts=tuple(int(value) for value in delta),
        roundtrip_joint_positions=tuple(float(value) for value in roundtrip),
        roundtrip_errors_rad=tuple(float(value) for value in roundtrip_error),
        guarded_joint_limits_ok=tuple(bool(value) for value in guarded),
        p0_selected_motor_ids=selected,
        p0_execution_eligible=not block_reasons,
        p0_block_reasons=tuple(block_reasons),
    )


def _json_telemetry(snapshot: Mapping[int, Mapping[str, object]]) -> dict:
    return {
        str(motor_id): {
            key: (value.item() if isinstance(value, np.generic) else value)
            for key, value in values.items()
        }
        for motor_id, values in snapshot.items()
    }


@dataclass(frozen=True)
class _LifecycleLimits:
    """Bounded multi-joint homing envelope for GELLO-HOME-P1.

    Distinct from :class:`GelloActuationLimits`: this drives up to seven arm
    joints at once, never the gripper, at a low, previously-registered
    per-joint current and a conservative step-per-tick pace well inside the
    validated P0 velocity cap.
    """

    control_hz: float
    bus_watchdog_ticks: int
    approach_goal_current_ma: int
    hold_goal_current_ma: int
    maximum_goal_current_ma: int
    approach_position_p_gain: int
    approach_position_i_gain: int
    approach_position_d_gain: int
    maximum_position_p_gain: int
    maximum_position_d_gain: int
    approach_position_p_gain_stages: tuple[int, ...]
    gain_ramp_tracking_error_counts: int
    gain_ramp_maximum_velocity_rad_s: float
    maximum_tracking_error_counts: int
    gain_ramp_dwell_cycles: int
    maximum_gain_raises_per_cycle: int
    maximum_stall_cycles_at_max_gain: int
    maximum_step_counts_per_tick: int
    settle_tolerance_counts: int
    maximum_settle_seconds: float
    soften_step_ma: int
    soften_step_interval_seconds: float
    maximum_latch_motion_counts: int
    minimum_input_voltage_v: float
    maximum_input_voltage_v: float
    maximum_temperature_c: float
    maximum_temperature_rise_c: float
    maximum_present_current_ma: int
    maximum_velocity_rad_s: float
    joint_limit_margin_rad: float

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> "_LifecycleLimits":
        values = {field: payload[field] for field in cls.__dataclass_fields__}
        values["approach_position_p_gain_stages"] = tuple(
            int(value) for value in values["approach_position_p_gain_stages"]
        )
        limits = cls(**values)
        limits.validate()
        return limits

    def validate(self) -> None:
        if not np.isfinite(self.control_hz) or self.control_hz <= 0:
            raise ValueError("GELLO-HOME-P1 control_hz must be positive")
        if not 1 <= self.bus_watchdog_ticks <= 127:
            raise ValueError("GELLO Bus Watchdog ticks must be in [1, 127]")
        if not (
            0 < self.approach_goal_current_ma
            <= self.hold_goal_current_ma
            <= self.maximum_goal_current_ma
            <= 600
        ):
            raise ValueError(
                "GELLO-HOME-P1 requires 0 < approach <= hold <= max <= 600 mA "
                "(600 mA matches the official GELLO current-based-position example; "
                "the actual approach/hold values used should still be raised gradually "
                "from the previously-validated lower envelope, not jumped to this ceiling)"
            )
        if self.approach_position_i_gain != 0:
            raise ValueError("GELLO-HOME-P1 requires position I gain 0")
        stages = tuple(int(value) for value in self.approach_position_p_gain_stages)
        if (
            not stages
            or stages[0] != self.approach_position_p_gain
            or tuple(sorted(set(stages))) != stages
            or stages[-1] != self.maximum_position_p_gain
            or not 0 < self.approach_position_p_gain <= self.maximum_position_p_gain <= 750
        ):
            raise ValueError(
                "GELLO-HOME-P1 staged P gains must increase from the approach "
                "gain to the registered Step H cap of 750"
            )
        if not 0 <= self.approach_position_d_gain <= self.maximum_position_d_gain <= 250:
            raise ValueError("GELLO-HOME-P1 D gain exceeds the registered cap")
        if not (
            self.maximum_step_counts_per_tick
            <= self.gain_ramp_tracking_error_counts
            and self.gain_ramp_tracking_error_counts
            + self.maximum_step_counts_per_tick
            <= self.maximum_tracking_error_counts
            <= 64
        ):
            raise ValueError(
                "GELLO-HOME-P1 gain-ramp/tracking thresholds must be ordered and <=64 counts"
            )
        if not 1 <= self.gain_ramp_dwell_cycles <= 10:
            raise ValueError("GELLO-HOME-P1 gain-ramp dwell must be 1-10 monitor cycles")
        if not (
            0.0
            <= float(self.gain_ramp_maximum_velocity_rad_s)
            < float(self.maximum_velocity_rad_s)
        ):
            raise ValueError(
                "GELLO-HOME-P1 gain-ramp velocity must be nonnegative and below the active velocity fault"
            )
        if self.maximum_gain_raises_per_cycle != 1:
            raise ValueError(
                "GELLO-HOME-P1 permits exactly one staged P-gain raise per monitor cycle"
            )
        if not 1 <= self.maximum_stall_cycles_at_max_gain <= 50:
            raise ValueError("GELLO-HOME-P1 maximum-gain stall dwell must be 1-50 cycles")
        if not 1 <= self.maximum_step_counts_per_tick <= 12:
            raise ValueError(
                "GELLO-HOME-P1 step-per-tick must stay well inside the validated velocity cap"
            )
        if not 0 <= self.settle_tolerance_counts <= 8:
            raise ValueError("GELLO-HOME-P1 settle tolerance must be small")
        if not 0.0 < self.maximum_settle_seconds <= 60.0:
            raise ValueError("GELLO-HOME-P1 settle timeout must be positive and bounded")
        if not 1 <= self.soften_step_ma <= 25:
            raise ValueError("GELLO-HOME-P1 soften step must be a small current decrement")
        if not 0.0 < self.soften_step_interval_seconds <= 1.0:
            raise ValueError("GELLO-HOME-P1 soften interval must be positive and bounded")
        if not 0 <= self.maximum_latch_motion_counts <= 2:
            raise ValueError("GELLO-HOME-P1 latch tolerance differs from preregistration")
        if not 3.7 <= self.minimum_input_voltage_v < self.maximum_input_voltage_v <= 6.0:
            raise ValueError("GELLO-HOME-P1 voltage limits must lie inside XL330 range")
        if not 0.0 < self.maximum_temperature_c <= 45.0 or not (
            0.0 < self.maximum_temperature_rise_c <= 10.0
        ):
            raise ValueError("GELLO-HOME-P1 temperature caps exceed the registered probe")
        if not 0 < self.maximum_present_current_ma <= 650:
            raise ValueError("GELLO-HOME-P1 present-current cap cannot exceed 650 mA")
        if not 0.0 < self.maximum_velocity_rad_s <= 0.20:
            raise ValueError("GELLO-HOME-P1 velocity cap cannot exceed the validated P0 value")
        if not 0.0 <= self.joint_limit_margin_rad <= 0.1:
            raise ValueError("GELLO-HOME-P1 joint-limit margin is invalid")


class _AlignmentLifecycle:
    refresh_watchdogs_during_arming = False
    """Fail-closed multi-joint (never gripper) homing-to-state controller.

    Separate from :class:`GelloAssistController`.  Drives up to seven GELLO
    arm joints at once toward a registered LIBERO state's target follower joint
    positions at a slow, bounded pace; holds torque at the target; and can
    soften-then-release so passive passive LIBERO teleoperation continues from
    that exact pose.  The gripper motor is never touched.
    """

    safety_protocol_label = "GELLO-HOME-P1"

    def __init__(
        self,
        *,
        bus,
        calibration: GelloCalibration,
        expected_model_numbers: Mapping[int, int],
        limits: _LifecycleLimits,
        clock: Callable[[], float] | None = None,
        sleep: Callable[[float], None] | None = None,
    ):
        self.bus = bus
        self.calibration = calibration
        self.expected_model_numbers = {
            int(key): int(value) for key, value in expected_model_numbers.items()
        }
        self.motor_ids = tuple(calibration.arm_ids) + (int(calibration.gripper_id),)
        if set(self.expected_model_numbers) != set(self.motor_ids):
            raise ValueError("GELLO expected model table must cover exactly the eight motors")
        self.limits = limits
        self._clock = time.monotonic if clock is None else clock
        self._sleep = time.sleep if sleep is None else sleep
        self.started_at = float(self._clock())
        self.torque_on_started_at: float | None = None
        self.state = HomingState.DISCONNECTED
        self.selected_motor_ids: tuple[int, ...] = tuple(calibration.arm_ids)
        self.baseline_temperature_c: dict[int, float] = {}
        self.baseline_voltage_v: dict[int, float] = {}
        self.initial_latch_positions: dict[int, int] = {}
        self.target_goal_counts: dict[int, int] = {}
        self.current_goal_ma: int = 0
        self.active_position_p_gains: dict[int, int] = {}
        self.check_read_only_velocity = True
        self.latest_active_snapshot: dict[int, dict[str, int | float]] = {}
        self.events: list[dict] = []
        self.telemetry: list[dict] = []
        self.monitor_cycle_seconds: list[float] = []
        self.monitor_deadline_miss_count = 0
        self.read_only_preflight_passed = False
        self.cleanup_attempted = False
        self.torque_off_verified = False
        self.physical_power_cut_required = False
        self.failure: str | None = None
        self._event("connected")

    def _event(self, event_type: str, **payload) -> dict:
        event = {
            "event_type": str(event_type),
            "state": self.state.value,
            "monotonic_elapsed_seconds": max(
                0.0, float(self._clock()) - self.started_at
            ),
            **payload,
        }
        self.events.append(event)
        return event

    def _set_state(self, state: HomingState, event_type: str, **payload) -> None:
        self.state = state
        self._event(event_type, **payload)

    def _read_motor_snapshot(self, motor_id: int) -> dict:
        motor_id = int(motor_id)
        raw_velocity = int(self.bus.read_control_item(motor_id, "present_velocity_raw"))
        return {
            "torque_enable": int(self.bus.read_control_item(motor_id, "torque_enable")),
            "operating_mode": int(self.bus.read_control_item(motor_id, "operating_mode")),
            "drive_mode": int(self.bus.read_control_item(motor_id, "drive_mode")),
            "startup_configuration": int(
                self.bus.read_control_item(motor_id, "startup_configuration")
            ),
            "current_limit_ma": int(
                self.bus.read_control_item(motor_id, "current_limit_ma")
            ),
            "hardware_error": int(
                self.bus.read_control_item(motor_id, "hardware_error")
            ),
            "position_p_gain": int(
                self.bus.read_control_item(motor_id, "position_p_gain")
            ),
            "position_i_gain": int(
                self.bus.read_control_item(motor_id, "position_i_gain")
            ),
            "position_d_gain": int(
                self.bus.read_control_item(motor_id, "position_d_gain")
            ),
            "bus_watchdog": int(
                self.bus.read_control_item(motor_id, "bus_watchdog")
            ),
            "goal_current_ma": int(
                self.bus.read_control_item(motor_id, "goal_current_ma")
            ),
            "goal_position_counts": int(
                self.bus.read_control_item(motor_id, "goal_position_counts")
            ),
            "present_current_ma": int(
                self.bus.read_control_item(motor_id, "present_current_ma")
            ),
            "present_velocity_raw": raw_velocity,
            "present_velocity_rad_s": float(
                raw_velocity * XL330_VELOCITY_UNIT_RAD_S
            ),
            "present_position_counts": int(
                self.bus.read_control_item(motor_id, "present_position_counts")
            ),
            "input_voltage_v": float(
                self.bus.read_control_item(motor_id, "input_voltage_raw") * 0.1
            ),
            "temperature_c": float(
                self.bus.read_control_item(motor_id, "temperature_c")
            ),
        }

    def snapshot(self, motor_ids: Iterable[int] | None = None) -> dict[int, dict]:
        ids = self.motor_ids if motor_ids is None else tuple(int(value) for value in motor_ids)
        snapshot = {motor_id: self._read_motor_snapshot(motor_id) for motor_id in ids}
        self.telemetry.append(
            {
                "monotonic_elapsed_seconds": max(
                    0.0, float(self._clock()) - self.started_at
                ),
                "motors": _json_telemetry(snapshot),
            }
        )
        return snapshot

    def _check_snapshot(
        self,
        snapshot: Mapping[int, Mapping[str, object]],
        *,
        require_active_configuration: bool = False,
    ) -> None:
        for motor_id, values in snapshot.items():
            if int(values["hardware_error"]) != 0:
                raise GelloActuationSafetyError(
                    f"Dynamixel ID {motor_id} hardware error is nonzero"
                )
            voltage = float(values["input_voltage_v"])
            if not self.limits.minimum_input_voltage_v <= voltage <= self.limits.maximum_input_voltage_v:
                raise GelloActuationSafetyError(
                    f"Dynamixel ID {motor_id} input voltage {voltage:.2f} V is outside {self.safety_protocol_label} limits"
                )
            temperature = float(values["temperature_c"])
            if temperature >= self.limits.maximum_temperature_c:
                raise GelloActuationSafetyError(
                    f"Dynamixel ID {motor_id} temperature {temperature:.1f} C exceeds {self.safety_protocol_label} limit"
                )
            if motor_id in self.baseline_temperature_c and (
                temperature - self.baseline_temperature_c[motor_id]
                >= self.limits.maximum_temperature_rise_c
            ):
                raise GelloActuationSafetyError(
                    f"Dynamixel ID {motor_id} temperature rise exceeds {self.safety_protocol_label} limit"
                )
            if abs(int(values["present_current_ma"])) > self.limits.maximum_present_current_ma:
                raise GelloActuationSafetyError(
                    f"Dynamixel ID {motor_id} present current exceeds {self.safety_protocol_label} limit"
                )
            if (require_active_configuration or self.check_read_only_velocity) and abs(
                float(values["present_velocity_rad_s"])
            ) > self.limits.maximum_velocity_rad_s:
                raise GelloActuationSafetyError(
                    f"Dynamixel ID {motor_id} velocity exceeds {self.safety_protocol_label} limit"
                )
            if require_active_configuration and int(values["torque_enable"]) != 0:
                if int(values["bus_watchdog"]) != self.limits.bus_watchdog_ticks:
                    raise GelloActuationSafetyError(
                        f"Dynamixel ID {motor_id} Bus Watchdog is not armed at the {self.safety_protocol_label} value"
                    )

    def read_only_preflight(self) -> dict:
        if self.state != HomingState.DISCONNECTED:
            raise GelloActuationSafetyError(
                "read-only preflight may run only once from DISCONNECTED"
            )
        self._set_state(HomingState.READ_ONLY_PREFLIGHT, "read_only_preflight_started")
        identities = {}
        for motor_id in self.motor_ids:
            identity = self.bus.ping(motor_id)
            identities[motor_id] = identity
            if int(identity["model_number"]) != self.expected_model_numbers[motor_id]:
                raise GelloActuationSafetyError(
                    f"Dynamixel ID {motor_id} model {identity['model_number']} does not match"
                )
        snapshot = self.snapshot()
        self.baseline_temperature_c = {
            motor_id: float(values["temperature_c"])
            for motor_id, values in snapshot.items()
        }
        self.baseline_voltage_v = {
            motor_id: float(values["input_voltage_v"])
            for motor_id, values in snapshot.items()
        }
        self._check_snapshot(snapshot)
        auto_torque = [
            motor_id
            for motor_id, values in snapshot.items()
            if int(values["drive_mode"]) & 0x08
        ]
        startup_torque = [
            motor_id
            for motor_id, values in snapshot.items()
            if int(values["startup_configuration"]) & 0x01
        ]
        if auto_torque:
            raise GelloActuationSafetyError(
                f"Drive Mode goal-update auto-torque is enabled on IDs {auto_torque}"
            )
        if startup_torque:
            raise GelloActuationSafetyError(
                f"Startup torque-on is enabled on IDs {startup_torque}"
            )
        already_enabled = [
            motor_id
            for motor_id, values in snapshot.items()
            if int(values["torque_enable"]) != 0
        ]
        if already_enabled:
            try:
                self.disarm_all(reason="preexisting_torque_detected")
            finally:
                raise GelloActuationSafetyError(
                    f"pre-existing torque was detected on IDs {already_enabled}; run stopped"
                )
        self._event(
            "read_only_preflight_passed",
            identities={str(key): value for key, value in identities.items()},
            snapshot=_json_telemetry(snapshot),
        )
        self.read_only_preflight_passed = True
        return {"identities": identities, "snapshot": snapshot}

    def disarm_all(self, *, reason: str) -> dict:
        self.cleanup_attempted = True
        errors: list[str] = []
        selected = tuple(self.selected_motor_ids)
        unselected = tuple(
            motor_id for motor_id in self.motor_ids if motor_id not in selected
        )
        # On a fault, disable torque on every motor before spending bus time
        # clearing current/watchdog/gain registers.  The old per-motor order
        # (current-zero -> torque-off -> watchdog clear -> gains) let the
        # 200 ms watchdog expire on later IDs while earlier IDs were being
        # cleaned, producing harmless-but-noisy Data Range errors and, more
        # importantly, delaying their torque-off packets.  Torque Enable
        # remains writable during a watchdog fault, while Goal Current can
        # become read-only until Bus Watchdog is cleared.
        for motor_id in selected:
            try:
                self.bus.set_torque_enabled(motor_id, False)
            except Exception as exc:
                errors.append(f"ID {motor_id} torque-off failed: {exc}")
        for motor_id in unselected:
            try:
                self.bus.set_torque_enabled(motor_id, False)
            except Exception as exc:
                errors.append(f"ID {motor_id} torque-off failed: {exc}")
        for motor_id in selected:
            try:
                self.bus.set_bus_watchdog_ticks(motor_id, 0)
            except Exception as exc:
                errors.append(f"ID {motor_id} watchdog-zero cleanup failed: {exc}")
            try:
                self.bus.set_goal_current_ma(motor_id, 0)
            except Exception as exc:
                errors.append(f"ID {motor_id} post-watchdog goal-current zero failed: {exc}")
            try:
                self.bus.set_position_gains(motor_id, p=0, i=0, d=0)
            except Exception as exc:
                errors.append(f"ID {motor_id} zero-gain cleanup failed: {exc}")
        for motor_id in unselected:
            try:
                self.bus.set_goal_current_ma(motor_id, 0)
            except Exception as exc:
                errors.append(f"ID {motor_id} goal-current zero failed: {exc}")
        disabled = {motor_id: False for motor_id in self.motor_ids}
        zero_current = {motor_id: False for motor_id in self.motor_ids}
        try:
            torque_values = self.bus.sync_read_control_item(
                "torque_enable", self.motor_ids
            )
            disabled = {
                motor_id: int(torque_values[motor_id]) == 0
                for motor_id in self.motor_ids
            }
        except Exception as exc:
            errors.append(f"all-ID torque SYNC_READ failed: {exc}")
        try:
            current_values = self.bus.sync_read_control_item(
                "goal_current_ma", self.motor_ids
            )
            zero_current = {
                motor_id: int(current_values[motor_id]) == 0
                for motor_id in self.motor_ids
            }
        except Exception as exc:
            errors.append(f"all-ID Goal Current SYNC_READ failed: {exc}")
        self.torque_on_started_at = None
        self.torque_off_verified = bool(all(disabled.values()))
        if self.torque_off_verified:
            cleanup_clean = bool(all(zero_current.values()) and not errors)
            self._set_state(
                HomingState.TORQUE_OFF_VERIFIED,
                "torque_off_verified"
                if cleanup_clean
                else "torque_off_verified_with_cleanup_errors",
                reason=str(reason),
                disabled={str(key): value for key, value in disabled.items()},
                goal_current_zero={
                    str(key): value for key, value in zero_current.items()
                },
                cleanup_errors=list(errors),
            )
            return {
                "verified": True,
                "clean": cleanup_clean,
                "disabled": disabled,
                "goal_current_zero": zero_current,
                "errors": list(errors),
            }
        self.physical_power_cut_required = True
        self.state = HomingState.FAULT_UNVERIFIED
        self._event(
            "torque_off_unverified",
            reason=str(reason),
            disabled={str(key): value for key, value in disabled.items()},
            errors=errors,
            physical_power_cut_required=True,
        )
        raise GelloActuationShutdownUnverified(
            "GELLO torque-off could not be verified; cut the physical 5 V motor power"
        )

    def verify_safe_off(self) -> dict:
        if (
            self.state != HomingState.READ_ONLY_PREFLIGHT
            or not self.read_only_preflight_passed
        ):
            raise GelloActuationSafetyError(
                "safe-off verification requires a passed read-only preflight"
            )
        result = self.disarm_all(reason="pre_arm_safe_off")
        if not result.get("clean", False):
            raise GelloActuationSafetyError(
                "all-ID torque was off but zero-current cleanup readback was incomplete"
            )
        self._set_state(HomingState.SAFE_OFF_VERIFIED, "safe_off_gate_passed")
        return result

    @staticmethod
    def _raise_if_operator_stop_requested(
        stop_requested: Callable[[], bool] | None, stage: str
    ) -> None:
        if stop_requested is not None and bool(stop_requested()):
            raise KeyboardInterrupt(f"operator requested stop during {stage}")

    def prepare_homing(self, plan: GelloAlignmentPlan) -> dict:
        """Latch a zero-force goal on all seven arm joints (never the gripper)."""

        if self.state != HomingState.SAFE_OFF_VERIFIED:
            raise GelloActuationSafetyError(
                "homing preparation requires the verified safe-off state"
            )
        if not all(plan.guarded_joint_limits_ok):
            raise GelloActuationSafetyError(
                "homing target lies outside guarded Panda joint limits"
            )
        selected = tuple(self.calibration.arm_ids)
        self.selected_motor_ids = selected
        self._set_state(HomingState.CONFIGURE_ZERO_GAIN, "zero_gain_configuration_started")
        positions: dict[int, int] = {}
        for motor_id in selected:
            if int(self.bus.read_control_item(motor_id, "torque_enable")) != 0:
                raise GelloActuationSafetyError("selected motor torque is not off")
            if int(self.bus.read_control_item(motor_id, "operating_mode")) != 5:
                raise GelloActuationSafetyError(
                    "selected motor is not in mode 5; run GELLO-ACT-P0 configure-mode5 first"
                )
            current_limit = int(
                self.bus.read_control_item(motor_id, "current_limit_ma")
            )
            if current_limit < self.limits.maximum_goal_current_ma:
                raise GelloActuationSafetyError(
                    "Current Limit is below the registered Goal Current cap; do not rewrite it"
                )
            self.bus.set_goal_current_ma(motor_id, 0)
            self.bus.set_position_gains(motor_id, p=0, i=0, d=0)
            self.bus.set_bus_watchdog_ticks(motor_id, self.limits.bus_watchdog_ticks)
            first = int(
                self.bus.read_control_item(motor_id, "present_position_counts")
            )
            second = int(
                self.bus.read_control_item(motor_id, "present_position_counts")
            )
            if abs(second - first) > self.limits.maximum_latch_motion_counts:
                raise GelloActuationSafetyError("GELLO moved while zero-goal latch was sampled")
            self.bus.set_goal_position_counts(motor_id, second)
            positions[motor_id] = second
        all_torque = self.bus.sync_read_control_item("torque_enable", self.motor_ids)
        if any(int(value) != 0 for value in all_torque.values()):
            raise GelloActuationSafetyError(
                "an unexpected motor became torque-enabled during preparation"
            )
        self.initial_latch_positions = dict(positions)
        joint_index_by_motor = {
            motor_id: self.calibration.arm_ids.index(motor_id) for motor_id in selected
        }
        self.target_goal_counts = {
            motor_id: int(plan.continuous_goal_counts[joint_index_by_motor[motor_id]])
            for motor_id in selected
        }
        self._set_state(
            HomingState.ZERO_GOAL_LATCHED,
            "zero_goal_latched",
            selected_motor_ids=list(selected),
            present_position_counts={str(key): value for key, value in positions.items()},
            target_goal_counts={
                str(key): value for key, value in self.target_goal_counts.items()
            },
        )
        return {"positions": positions, "target_goal_counts": dict(self.target_goal_counts)}

    def arm_homing(self, *, stop_requested: Callable[[], bool] | None = None) -> dict:
        if self.state != HomingState.ZERO_GOAL_LATCHED:
            raise GelloActuationSafetyError("homing was not zero-goal prepared")
        self._raise_if_operator_stop_requested(stop_requested, "pre-arm")
        self._set_state(HomingState.TORQUE_ON_ZERO_GAIN, "torque_enable_started")
        self.torque_on_started_at = float(self._clock())
        # The earlier pre-arm safe-off verification is no longer current once
        # any torque-enable write is attempted.  Cleanup must establish a
        # fresh all-ID torque-off readback before summary can report success.
        self.torque_off_verified = False
        watchdog_refresh_count = 0

        def refresh_watchdogs(expected_enabled: set[int], stage: str) -> None:
            """Keep the 400 ms watchdog alive during sequential arming writes.

            The normal active loop is already two aggregate reads at 5 Hz. The
            one-time arming path still performs acknowledged writes and
            readbacks per motor; without an aggregate instruction between
            joints, ID 1 can expire before J7 has received its gains. Reading
            all Torque Enable values refreshes every selected motor while also
            checking the exact partially-armed state and gripper exclusion.
            """

            nonlocal watchdog_refresh_count
            observed = self.bus.sync_read_control_item(
                "torque_enable", self.motor_ids
            )
            expected = {
                motor_id: int(motor_id in expected_enabled)
                for motor_id in self.motor_ids
            }
            normalized = {
                motor_id: int(observed[motor_id]) for motor_id in self.motor_ids
            }
            if normalized != expected:
                raise GelloActuationSafetyError(
                    f"all-ID torque-enable readback differs during {stage}"
                )
            watchdog_refresh_count += 1

        enabled: set[int] = set()
        for motor_id in self.selected_motor_ids:
            self.bus.set_torque_enabled(motor_id, True)
            enabled.add(motor_id)
            if self.refresh_watchdogs_during_arming:
                refresh_watchdogs(enabled, "sequential torque enable")
            self._raise_if_operator_stop_requested(stop_requested, "torque enable")
        if not self.refresh_watchdogs_during_arming:
            refresh_watchdogs(enabled, "torque readback")
        self._raise_if_operator_stop_requested(stop_requested, "torque readback")
        relatched: dict[int, int] = {}
        for motor_id in self.selected_motor_ids:
            fresh_position = int(
                self.bus.read_control_item(motor_id, "present_position_counts")
            )
            self._raise_if_operator_stop_requested(stop_requested, "post-enable position read")
            self.bus.set_goal_position_counts(motor_id, fresh_position)
            self._raise_if_operator_stop_requested(stop_requested, "post-enable goal latch")
            goal = int(self.bus.read_control_item(motor_id, "goal_position_counts"))
            if abs(goal - fresh_position) > self.limits.maximum_latch_motion_counts:
                raise GelloActuationSafetyError("post-enable current-goal relatch failed")
            if (
                abs(fresh_position - self.initial_latch_positions[motor_id])
                > self.limits.maximum_latch_motion_counts
            ):
                raise GelloActuationSafetyError("torque enable caused an excessive position jump")
            relatched[motor_id] = fresh_position
            if self.refresh_watchdogs_during_arming:
                refresh_watchdogs(enabled, "post-enable goal relatch")
        self._set_state(
            HomingState.POST_ENABLE_RELATCHED,
            "post_enable_goal_relatched",
            positions={str(key): value for key, value in relatched.items()},
        )
        self.current_goal_ma = int(self.limits.approach_goal_current_ma)
        self.active_position_p_gains = {
            motor_id: int(self.limits.approach_position_p_gain)
            for motor_id in self.selected_motor_ids
        }
        for motor_id in self.selected_motor_ids:
            self.bus.set_goal_current_ma(motor_id, self.current_goal_ma)
            self._raise_if_operator_stop_requested(stop_requested, "Goal Current activation")
            self.bus.set_position_gains(
                motor_id,
                p=self.limits.approach_position_p_gain,
                i=self.limits.approach_position_i_gain,
                d=self.limits.approach_position_d_gain,
            )
            if self.refresh_watchdogs_during_arming:
                refresh_watchdogs(enabled, "Goal Current and gain activation")
            self._raise_if_operator_stop_requested(stop_requested, "position-gain activation")
        if self.refresh_watchdogs_during_arming:
            self._event(
                "arming_watchdogs_refreshed",
                refresh_count=watchdog_refresh_count,
                bus_watchdog_ticks=int(self.limits.bus_watchdog_ticks),
            )
        snapshot = self.monitor_once()
        self._set_state(
            HomingState.APPROACHING,
            "homing_approach_active",
            goal_current_ma=self.current_goal_ma,
            position_gains={
                "p": self.limits.approach_position_p_gain,
                "i": self.limits.approach_position_i_gain,
                "d": self.limits.approach_position_d_gain,
            },
        )
        return {"snapshot": snapshot}

    def monitor_once(self) -> dict:
        if not self.selected_motor_ids:
            raise GelloActuationSafetyError("active monitor requires selected arm motors")
        started = float(self._clock())
        all_torque = self.bus.sync_read_control_item("torque_enable", self.motor_ids)
        expected_torque = {
            motor_id: int(motor_id in self.selected_motor_ids)
            for motor_id in self.motor_ids
        }
        observed_torque = {
            motor_id: int(all_torque[motor_id]) for motor_id in self.motor_ids
        }
        if observed_torque != expected_torque:
            raise GelloActuationSafetyError(
                "all-ID active torque state differs from the GELLO-HOME-P1 arm-joint-only contract"
            )
        snapshot = self._read_monitor_snapshots()
        elapsed = max(0.0, float(self._clock()) - started)
        self.monitor_cycle_seconds.append(elapsed)
        telemetry_sample = {
            "monotonic_elapsed_seconds": max(
                0.0, float(self._clock()) - self.started_at
            ),
            "monitor_cycle_seconds": elapsed,
            "all_id_torque_enable": {
                str(key): value for key, value in observed_torque.items()
            },
            "motors": _json_telemetry(snapshot),
        }
        self.telemetry.append(telemetry_sample)
        self.latest_active_snapshot = snapshot
        interval = 1.0 / self.limits.control_hz
        if elapsed > interval:
            self.monitor_deadline_miss_count += 1
            raise GelloActuationSafetyError(
                f"{self.safety_protocol_label} active monitor exceeded the registered cycle time"
            )
        self._check_snapshot(snapshot, require_active_configuration=True)
        for motor_id, values in snapshot.items():
            tracking_error = abs(
                int(values["goal_position_counts"])
                - int(values["present_position_counts"])
            )
            if tracking_error > self.limits.maximum_tracking_error_counts:
                raise GelloTrackingError(
                    f"Dynamixel ID {motor_id} goal-present error exceeds {self.safety_protocol_label} limit"
                )
        return snapshot

    def _read_monitor_snapshots(self) -> dict[int, dict[str, int]]:
        """Keep the registered GELLO-HOME-P1 sequential monitor unchanged."""

        return {
            motor_id: self._read_active_motor_snapshot(motor_id)
            for motor_id in self.selected_motor_ids
        }


    @staticmethod
    def _normalize_active_snapshot(raw: Mapping[str, object]) -> dict[str, int | float]:
        raw_velocity = int(raw["present_velocity_raw"])
        return {
            "torque_enable": int(raw["torque_enable"]),
            "hardware_error": int(raw["hardware_error"]),
            "position_p_gain": int(raw["position_p_gain"]),
            "position_i_gain": int(raw["position_i_gain"]),
            "position_d_gain": int(raw["position_d_gain"]),
            "bus_watchdog": int(raw["bus_watchdog"]),
            "goal_current_ma": int(raw["goal_current_ma"]),
            "goal_position_counts": int(raw["goal_position_counts"]),
            "present_current_ma": int(raw["present_current_ma"]),
            "present_velocity_raw": raw_velocity,
            "present_velocity_rad_s": float(
                raw_velocity * XL330_VELOCITY_UNIT_RAD_S
            ),
            "present_position_counts": int(raw["present_position_counts"]),
            "input_voltage_v": float(int(raw["input_voltage_raw"]) * 0.1),
            "temperature_c": float(raw["temperature_c"]),
        }

    def _read_active_motor_snapshot(self, motor_id: int) -> dict[str, int | float]:
        return self._normalize_active_snapshot(
            self.bus.read_active_snapshot(int(motor_id))
        )



    def soften_and_release(
        self, *, stop_requested: Callable[[], bool] | None = None
    ) -> dict:
        """Ramp Goal Current down before torque-off so teleop can take over.

        Never jumps straight from an active hold current to torque-off: each
        step lowers Goal Current by ``soften_step_ma`` and re-reads the
        snapshot before the next decrement, so the follower never feels a
        sudden release.
        """

        if self.state != HomingState.HOLDING_AT_TARGET:
            raise GelloActuationSafetyError(
                "GELLO-HOME-P1 soften-and-release requires the HOLDING_AT_TARGET state"
            )
        self._set_state(HomingState.SOFTENING, "softening_started")
        current = int(self.current_goal_ma)
        while current > 0:
            self._raise_if_operator_stop_requested(stop_requested, "softening")
            current = max(0, current - int(self.limits.soften_step_ma))
            for motor_id in self.selected_motor_ids:
                self.bus.set_goal_current_ma(motor_id, current)
            self.current_goal_ma = current
            self._sleep(float(self.limits.soften_step_interval_seconds))
            if current > 0:
                self.monitor_once()
        result = self.disarm_all(reason="soften_and_release_for_teleop")
        if not result.get("clean", False):
            raise GelloActuationSafetyError(
                "torque-off was verified but final zero-state cleanup was incomplete"
            )
        self._set_state(HomingState.RELEASED_FOR_TELEOP, "released_for_teleop")
        return result


    def finish(self, *, reason: str = "normal_completion") -> dict:
        result = self.disarm_all(reason=reason)
        if not result.get("clean", False):
            raise GelloActuationSafetyError(
                "torque-off was verified but final zero-state cleanup was incomplete"
            )
        self._set_state(HomingState.COMPLETE, "homing_complete")
        return result

    def summary(self) -> dict:
        command_log = getattr(self.bus, "command_log", [])
        return {
            "schema_version": "gello-homing-session-p1",
            "protocol_version": HOMING_PROTOCOL_VERSION,
            "state": self.state.value,
            "selected_motor_ids": list(self.selected_motor_ids),
            "events": list(self.events),
            "telemetry": list(self.telemetry),
            "read_only_preflight_passed": bool(self.read_only_preflight_passed),
            "cleanup_attempted": bool(self.cleanup_attempted),
            "torque_off_verified_all_ids": bool(self.torque_off_verified),
            "physical_power_cut_required": bool(self.physical_power_cut_required),
            "failure": self.failure,
            "write_packets": int(getattr(self.bus, "write_packets", 0)),
            "motor_register_writes": int(
                getattr(self.bus, "motor_register_writes", 0)
            ),
            "instruction_counts": dict(
                getattr(self.bus, "instruction_counts", {})
            ),
            "monitor_cycle_seconds": list(self.monitor_cycle_seconds),
            "monitor_deadline_miss_count": int(self.monitor_deadline_miss_count),
            "command_log": list(command_log),
            "project_limits": asdict(self.limits),
        }


@dataclass(frozen=True)
class GelloAlignmentAssistLimits:
    """Locked human-guided alignment envelope for GELLO-ALIGN-A4.

    The motor goal is always a short waypoint measured from the *current*
    position.  It therefore indicates and assists the direction the operator
    should move, but it cannot autonomously accumulate a large goal-position
    error as GELLO-HOME-P1 does.
    """

    profile: str = "GELLO-ALIGN-A4"
    control_hz: float = 5.0
    bus_watchdog_ticks: int = 20
    goal_current_ma: int = 100
    position_p_gain: int = 750
    position_i_gain: int = 0
    position_d_gain: int = 150
    maximum_guide_offset_counts: int = 20
    maximum_tracking_error_counts: int = 40
    ready_max_abs_error_rad: float = 0.05
    ready_rms_error_rad: float = 0.03
    ready_max_velocity_raw: int = 1
    ready_stable_cycles: int = 5
    post_release_stable_cycles: int = 3
    maximum_alignment_seconds: float = 180.0
    soften_step_ma: int = 10
    soften_step_interval_seconds: float = 0.1
    maximum_latch_motion_counts: int = 32
    pre_arm_max_velocity_raw: int = 2
    pre_arm_max_drift_counts: int = 4
    pre_arm_stable_cycles: int = 3
    minimum_input_voltage_v: float = 4.5
    maximum_input_voltage_v: float = 5.5
    maximum_temperature_c: float = 45.0
    maximum_temperature_rise_c: float = 10.0
    maximum_present_current_ma: int = 150
    maximum_velocity_rad_s: float = 1.50
    joint_limit_margin_rad: float = 0.02

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> "GelloAlignmentAssistLimits":
        unknown = sorted(set(payload) - set(cls.__dataclass_fields__))
        if unknown:
            raise ValueError(f"unknown GELLO-ALIGN-A4 limits: {unknown}")
        limits = cls(**payload)
        limits.validate()
        return limits

    def validate(self) -> None:
        if self.profile not in ("GELLO-ALIGN-A4", "GELLO-ALIGN-A5", "GELLO-ALIGN-A6"):
            raise ValueError("unknown GELLO alignment safety profile")
        if float(self.control_hz) != 5.0:
            raise ValueError("GELLO-ALIGN-A4 control rate is locked to 5 Hz")
        if not 1 <= int(self.bus_watchdog_ticks) <= 127:
            raise ValueError("GELLO-ALIGN-A4 Bus Watchdog ticks are invalid")
        if not 1 <= int(self.goal_current_ma) <= 100:
            raise ValueError("GELLO-ALIGN-A4 Goal Current cannot exceed 100 mA")
        if not 1 <= int(self.position_p_gain) <= 750:
            raise ValueError("GELLO-ALIGN-A4 P gain cannot exceed 750")
        if int(self.position_i_gain) != 0:
            raise ValueError("GELLO-ALIGN-A4 requires zero I gain")
        if not 0 <= int(self.position_d_gain) <= 250:
            raise ValueError("GELLO-ALIGN-A4 D gain cannot exceed 250")
        if not 1 <= int(self.maximum_guide_offset_counts) <= 20:
            raise ValueError("GELLO-ALIGN-A4 guide waypoint cannot exceed 20 counts")
        if not int(self.maximum_guide_offset_counts) < int(self.maximum_tracking_error_counts) <= 40:
            raise ValueError("GELLO-ALIGN-A4 tracking-error envelope is invalid")
        ready_cap = {"GELLO-ALIGN-A4": 0.05, "GELLO-ALIGN-A5": 0.15, "GELLO-ALIGN-A6": 0.60}[self.profile]
        if not 0.0 < float(self.ready_rms_error_rad) <= float(self.ready_max_abs_error_rad) <= ready_cap:
            raise ValueError(f"{self.profile} ready tolerances exceed the lock")
        if not 0 <= int(self.ready_max_velocity_raw) <= 1:
            raise ValueError("GELLO-ALIGN-A4 ready velocity must be at most one raw unit")
        if not 1 <= int(self.ready_stable_cycles) <= 10:
            raise ValueError("GELLO-ALIGN-A4 ready dwell is invalid")
        if not 1 <= int(self.post_release_stable_cycles) <= 10:
            raise ValueError("GELLO-ALIGN-A4 post-release dwell is invalid")
        if not 10.0 <= float(self.maximum_alignment_seconds) <= 300.0:
            raise ValueError("GELLO-ALIGN-A4 alignment timeout is invalid")
        if not 1 <= int(self.soften_step_ma) <= 25:
            raise ValueError("GELLO-ALIGN-A4 soften step is invalid")
        if not 0.0 < float(self.soften_step_interval_seconds) <= 1.0:
            raise ValueError("GELLO-ALIGN-A4 soften interval is invalid")
        if not 2 <= int(self.maximum_latch_motion_counts) <= 32:
            raise ValueError("GELLO-ALIGN-A4 zero-force latch tolerance is invalid")
        if not 0 <= int(self.pre_arm_max_velocity_raw) <= 2:
            raise ValueError("GELLO-ALIGN-A4 pre-arm velocity threshold is invalid")
        if not 0 <= int(self.pre_arm_max_drift_counts) <= 4:
            raise ValueError("GELLO-ALIGN-A4 pre-arm drift threshold is invalid")
        if not 2 <= int(self.pre_arm_stable_cycles) <= 10:
            raise ValueError("GELLO-ALIGN-A4 pre-arm stable dwell is invalid")
        if not 3.7 <= float(self.minimum_input_voltage_v) < float(self.maximum_input_voltage_v) <= 6.0:
            raise ValueError("GELLO-ALIGN-A4 voltage envelope is invalid")
        if not 0.0 < float(self.maximum_temperature_c) <= 45.0:
            raise ValueError("GELLO-ALIGN-A4 temperature cap is invalid")
        if not 0.0 < float(self.maximum_temperature_rise_c) <= 10.0:
            raise ValueError("GELLO-ALIGN-A4 temperature-rise cap is invalid")
        if not int(self.goal_current_ma) <= int(self.maximum_present_current_ma) <= 150:
            raise ValueError("GELLO-ALIGN-A4 present-current cap is invalid")
        if not 0.20 < float(self.maximum_velocity_rad_s) <= 1.50:
            raise ValueError("GELLO-ALIGN-A4 human-guided velocity cap is invalid")
        if not 0.0 <= float(self.joint_limit_margin_rad) <= 0.1:
            raise ValueError("GELLO-ALIGN-A4 joint-limit margin is invalid")

    def homing_limits(self) -> _LifecycleLimits:
        """Adapt the shared fail-closed bus lifecycle without autonomous homing."""

        self.validate()
        return _LifecycleLimits(
            control_hz=float(self.control_hz),
            bus_watchdog_ticks=int(self.bus_watchdog_ticks),
            approach_goal_current_ma=int(self.goal_current_ma),
            hold_goal_current_ma=int(self.goal_current_ma),
            maximum_goal_current_ma=int(self.goal_current_ma),
            approach_position_p_gain=int(self.position_p_gain),
            approach_position_i_gain=int(self.position_i_gain),
            approach_position_d_gain=int(self.position_d_gain),
            maximum_position_p_gain=int(self.position_p_gain),
            maximum_position_d_gain=int(self.position_d_gain),
            approach_position_p_gain_stages=(int(self.position_p_gain),),
            gain_ramp_tracking_error_counts=int(self.maximum_guide_offset_counts),
            gain_ramp_maximum_velocity_rad_s=0.0,
            maximum_tracking_error_counts=int(self.maximum_tracking_error_counts),
            gain_ramp_dwell_cycles=1,
            maximum_gain_raises_per_cycle=1,
            maximum_stall_cycles_at_max_gain=1,
            maximum_step_counts_per_tick=min(12, int(self.maximum_guide_offset_counts)),
            settle_tolerance_counts=8,
            maximum_settle_seconds=min(60.0, float(self.maximum_alignment_seconds)),
            soften_step_ma=int(self.soften_step_ma),
            soften_step_interval_seconds=float(self.soften_step_interval_seconds),
            maximum_latch_motion_counts=int(self.maximum_latch_motion_counts),
            minimum_input_voltage_v=float(self.minimum_input_voltage_v),
            maximum_input_voltage_v=float(self.maximum_input_voltage_v),
            maximum_temperature_c=float(self.maximum_temperature_c),
            maximum_temperature_rise_c=float(self.maximum_temperature_rise_c),
            maximum_present_current_ma=int(self.maximum_present_current_ma),
            maximum_velocity_rad_s=float(self.maximum_velocity_rad_s),
            joint_limit_margin_rad=float(self.joint_limit_margin_rad),
        )


class GelloAlignmentAssistController(_AlignmentLifecycle):
    """Human-led, current-limited directional assistance toward a LIBERO state."""

    safety_protocol_label = ALIGNMENT_ASSIST_PROTOCOL_VERSION
    refresh_watchdogs_during_arming = True

    def __init__(
        self,
        *,
        bus,
        calibration: GelloCalibration,
        expected_model_numbers: Mapping[int, int],
        limits: GelloAlignmentAssistLimits,
        clock: Callable[[], float] | None = None,
        sleep: Callable[[float], None] | None = None,
    ):
        limits.validate()
        self.assist_limits = limits
        self.safety_protocol_label = limits.profile
        self.ready_streak = 0
        self.guide_samples: list[dict] = []
        self.post_release_samples: list[dict] = []
        self.passive_settle_samples: list[dict] = []
        self.passive_settle_streak = 0
        self._previous_passive_settle_positions: np.ndarray | None = None
        self.alignment_target_joint_positions: np.ndarray | None = None
        self.collection_start_eligible = False
        super().__init__(
            bus=bus,
            calibration=calibration,
            expected_model_numbers=expected_model_numbers,
            limits=limits.homing_limits(),
            clock=clock,
            sleep=sleep,
        )
        # Passive human motion at Enter is followed by a stationary dwell.
        # Powered monitoring still passes require_active_configuration=True,
        # so the A4 velocity cap remains enforced after torque is enabled.
        self.check_read_only_velocity = False

    def _read_monitor_snapshots(self) -> dict[int, dict[str, int]]:
        """Use the A4 seven-ID aggregate read only for assisted alignment."""

        raw_snapshots = self.bus.sync_read_active_snapshots(self.selected_motor_ids)
        return {
            motor_id: self._normalize_active_snapshot(raw_snapshots[motor_id])
            for motor_id in self.selected_motor_ids
        }

    def prepare_alignment(self, plan: GelloAlignmentPlan) -> dict:
        self.alignment_target_joint_positions = np.asarray(
            plan.target_joint_positions, dtype=np.float64
        )
        return self.prepare_homing(plan)

    def passive_settle_once(self) -> dict:
        """Read a torque-off sample and wait for the operator to hold still.

        Human motion before torque-on is never a fault.  It merely resets the
        stable dwell, so normal backdriving cannot consume a LIBERO attempt.
        """

        if self.state != HomingState.READ_ONLY_PREFLIGHT:
            raise GelloActuationSafetyError(
                "GELLO-ALIGN-A4 passive settling requires read-only preflight"
            )
        all_torque = self.bus.sync_read_control_item("torque_enable", self.motor_ids)
        if any(int(value) != 0 for value in all_torque.values()):
            raise GelloActuationSafetyError(
                "GELLO-ALIGN-A4 passive settling detected torque-on"
            )
        # Use the same aggregate seven-ID active block as powered monitoring.
        # Seven sequential 83-byte reads made the apparent 3-sample settle
        # dwell both slow and sensitive to motion occurring between J1 and J7.
        snapshot = self._read_monitor_snapshots()
        for motor_id, values in snapshot.items():
            if int(values["hardware_error"]) != 0:
                raise GelloActuationSafetyError(
                    f"Dynamixel ID {motor_id} hardware error is nonzero"
                )
            voltage = float(values["input_voltage_v"])
            if not self.limits.minimum_input_voltage_v <= voltage <= self.limits.maximum_input_voltage_v:
                raise GelloActuationSafetyError(
                    f"Dynamixel ID {motor_id} input voltage {voltage:.2f} V is outside GELLO-ALIGN-A4 limits"
                )
            if float(values["temperature_c"]) >= self.limits.maximum_temperature_c:
                raise GelloActuationSafetyError(
                    f"Dynamixel ID {motor_id} temperature exceeds GELLO-ALIGN-A4 limit"
                )
        positions = np.asarray(
            [int(snapshot[motor_id]["present_position_counts"]) for motor_id in self.selected_motor_ids],
            dtype=np.int64,
        )
        velocities = np.asarray(
            [int(snapshot[motor_id]["present_velocity_raw"]) for motor_id in self.selected_motor_ids],
            dtype=np.int64,
        )
        drift = (
            np.zeros_like(positions)
            if self._previous_passive_settle_positions is None
            else positions - self._previous_passive_settle_positions
        )
        stable_now = bool(
            self._previous_passive_settle_positions is not None
            and int(np.max(np.abs(velocities))) <= int(self.assist_limits.pre_arm_max_velocity_raw)
            and int(np.max(np.abs(drift))) <= int(self.assist_limits.pre_arm_max_drift_counts)
        )
        self.passive_settle_streak = self.passive_settle_streak + 1 if stable_now else 0
        self._previous_passive_settle_positions = positions.copy()
        result = {
            "sample_index": len(self.passive_settle_samples),
            "monotonic_elapsed_seconds": max(0.0, float(self._clock()) - self.started_at),
            "present_position_counts": positions.tolist(),
            "present_velocity_raw": velocities.tolist(),
            "drift_counts": drift.tolist(),
            "stable_now": stable_now,
            "stable_streak": int(self.passive_settle_streak),
            "settled_for_arm": bool(
                self.passive_settle_streak >= int(self.assist_limits.pre_arm_stable_cycles)
            ),
        }
        self.passive_settle_samples.append(result)
        if result["settled_for_arm"]:
            self._event(
                "human_guided_pre_arm_settled",
                sample_index=result["sample_index"],
                stable_streak=result["stable_streak"],
                present_position_counts=result["present_position_counts"],
            )
        return result

    def arm_alignment(self, *, stop_requested: Callable[[], bool] | None = None) -> dict:
        return self.arm_homing(stop_requested=stop_requested)

    def verify_near_pose_passive_start(self) -> dict:
        """Verify torque-off without issuing any cleanup or motor write."""

        if self.state != HomingState.READ_ONLY_PREFLIGHT:
            raise GelloActuationSafetyError(
                "near-pose passive start requires read-only preflight state"
            )
        torque = self.bus.sync_read_control_item("torque_enable", self.motor_ids)
        if any(int(value) != 0 for value in torque.values()):
            raise GelloActuationSafetyError(
                "near-pose passive start detected torque-on"
            )
        if int(getattr(self.bus, "motor_register_writes", -1)) != 0 or int(
            getattr(self.bus, "write_packets", -1)
        ) != 0:
            raise GelloActuationSafetyError(
                "near-pose passive start setup unexpectedly wrote a motor register"
            )
        self.torque_off_verified = True
        self._event(
            "near_pose_passive_start_torque_off_verified",
            torque_off_verified_all_ids=True,
            motor_register_writes=0,
        )
        return {
            "passed": True,
            "torque_off_verified_all_ids": True,
            "motor_register_writes": 0,
        }

    def _alignment_diagnostic(self, snapshot: Mapping[int, Mapping[str, object]]) -> dict:
        if self.alignment_target_joint_positions is None:
            raise GelloActuationSafetyError(
                "GELLO-ALIGN-A4 target joints are unavailable"
            )
        present_counts = np.asarray(
            [
                int(snapshot[motor_id]["present_position_counts"])
                for motor_id in self.selected_motor_ids
            ],
            dtype=np.int64,
        )
        errors_counts = np.asarray(
            [
                int(self.target_goal_counts[motor_id])
                - int(snapshot[motor_id]["present_position_counts"])
                for motor_id in self.selected_motor_ids
            ],
            dtype=np.int64,
        )
        alignment = self.calibration.alignment_diagnostics(
            np.mod(present_counts, COUNTS_PER_REVOLUTION),
            self.alignment_target_joint_positions,
            tolerance_rad=float(self.assist_limits.ready_max_abs_error_rad),
        )
        mapped_joints = np.asarray(
            alignment["mapped_leader_joint_positions"], dtype=np.float64
        )
        errors_rad = np.asarray(
            alignment["alignment_error_radians"], dtype=np.float64
        )
        raw_velocity = np.asarray(
            [int(snapshot[motor_id]["present_velocity_raw"]) for motor_id in self.selected_motor_ids],
            dtype=np.int64,
        )
        max_abs = float(np.max(np.abs(errors_rad)))
        rms = float(np.sqrt(np.mean(np.square(errors_rad))))
        stable = bool(
            max_abs <= float(self.assist_limits.ready_max_abs_error_rad)
            and rms <= float(self.assist_limits.ready_rms_error_rad)
            and int(np.max(np.abs(raw_velocity))) <= int(self.assist_limits.ready_max_velocity_raw)
        )
        return {
            "present_position_counts": present_counts.tolist(),
            "target_joint_positions": self.alignment_target_joint_positions.tolist(),
            "mapped_gello_joint_positions": mapped_joints.tolist(),
            "target_error_counts": errors_counts.tolist(),
            "target_error_radians": errors_rad.tolist(),
            "required_count_deltas": np.asarray(
                alignment["required_count_deltas"], dtype=np.int64
            ).tolist(),
            "max_abs_target_error_rad": max_abs,
            "rms_target_error_rad": rms,
            "present_velocity_raw": raw_velocity.tolist(),
            "stable_now": stable,
        }

    def guide_once(self) -> dict:
        if self.state != HomingState.APPROACHING:
            raise GelloActuationSafetyError("GELLO-ALIGN-A4 guide requires active assistance")
        if float(self._clock()) - self.started_at > float(self.assist_limits.maximum_alignment_seconds):
            raise GelloActuationSafetyError("GELLO-ALIGN-A4 exceeded the alignment timeout")
        snapshot = self.latest_active_snapshot or self.monitor_once()
        commanded: dict[str, int] = {}
        for motor_id in self.selected_motor_ids:
            present = int(snapshot[motor_id]["present_position_counts"])
            target = int(self.target_goal_counts[motor_id])
            offset = int(
                np.clip(
                    target - present,
                    -int(self.assist_limits.maximum_guide_offset_counts),
                    int(self.assist_limits.maximum_guide_offset_counts),
                )
            )
            waypoint = present + offset
            self.bus.set_goal_position_counts(motor_id, waypoint)
            commanded[str(motor_id)] = waypoint
        snapshot = self.monitor_once()
        diagnostic = self._alignment_diagnostic(snapshot)
        self.ready_streak = self.ready_streak + 1 if diagnostic["stable_now"] else 0
        diagnostic.update(
            {
                "sample_index": len(self.guide_samples),
                "monotonic_elapsed_seconds": max(0.0, float(self._clock()) - self.started_at),
                "commanded_waypoint_counts": commanded,
                "ready_streak": int(self.ready_streak),
                "alignment_ready": bool(
                    self.ready_streak >= int(self.assist_limits.ready_stable_cycles)
                ),
            }
        )
        self.guide_samples.append(diagnostic)
        if diagnostic["alignment_ready"]:
            self._set_state(
                HomingState.HOLDING_AT_TARGET,
                "human_guided_alignment_ready",
                sample_index=diagnostic["sample_index"],
                ready_streak=int(self.ready_streak),
                max_abs_target_error_rad=diagnostic["max_abs_target_error_rad"],
                rms_target_error_rad=diagnostic["rms_target_error_rad"],
            )
        return diagnostic

    def release_and_verify_for_collection(
        self, *, stop_requested: Callable[[], bool] | None = None
    ) -> dict:
        if self.state != HomingState.HOLDING_AT_TARGET:
            raise GelloActuationSafetyError("GELLO-ALIGN-A4 is not ready for release")
        self.soften_and_release(stop_requested=stop_requested)
        interval = 1.0 / float(self.assist_limits.control_hz)
        stable = True
        for sample_index in range(int(self.assist_limits.post_release_stable_cycles)):
            self._raise_if_operator_stop_requested(stop_requested, "post-release verification")
            snapshot = self.snapshot(self.selected_motor_ids)
            diagnostic = self._alignment_diagnostic(snapshot)
            diagnostic.update(
                {
                    "sample_index": sample_index,
                    "monotonic_elapsed_seconds": max(0.0, float(self._clock()) - self.started_at),
                }
            )
            self.post_release_samples.append(diagnostic)
            stable = bool(stable and diagnostic["stable_now"])
            if sample_index + 1 < int(self.assist_limits.post_release_stable_cycles):
                self._sleep(interval)
        all_torque = self.bus.sync_read_control_item("torque_enable", self.motor_ids)
        torque_off = all(int(value) == 0 for value in all_torque.values())
        self.collection_start_eligible = bool(stable and torque_off)
        self._event(
            "post_release_alignment_verified" if self.collection_start_eligible else "post_release_alignment_drifted",
            stable=stable,
            torque_off_verified_all_ids=torque_off,
            post_release_samples=len(self.post_release_samples),
        )
        return {
            "passed": self.collection_start_eligible,
            "stable": stable,
            "torque_off_verified_all_ids": torque_off,
            "samples": list(self.post_release_samples),
        }

    def maintain_ready_once(self) -> dict:
        """Refresh watchdog/telemetry while the operator reviews READY."""

        if self.state != HomingState.HOLDING_AT_TARGET:
            raise GelloActuationSafetyError("GELLO-ALIGN-A4 is not in READY hold")
        snapshot = self.monitor_once()
        diagnostic = self._alignment_diagnostic(snapshot)
        diagnostic.update(
            {
                "sample_index": len(self.guide_samples),
                "monotonic_elapsed_seconds": max(0.0, float(self._clock()) - self.started_at),
                "commanded_waypoint_counts": {
                    str(motor_id): int(snapshot[motor_id]["goal_position_counts"])
                    for motor_id in self.selected_motor_ids
                },
                "ready_streak": int(self.ready_streak),
                "alignment_ready": bool(diagnostic["stable_now"]),
                "ready_hold_sample": True,
            }
        )
        self.guide_samples.append(diagnostic)
        if not diagnostic["stable_now"]:
            self.ready_streak = 0
            self._set_state(
                HomingState.APPROACHING,
                "human_guided_alignment_ready_lost",
                max_abs_target_error_rad=diagnostic["max_abs_target_error_rad"],
                rms_target_error_rad=diagnostic["rms_target_error_rad"],
            )
        return diagnostic

    def summary(self) -> dict:
        summary = super().summary()
        summary.update(
            {
                "schema_version": "gello-alignment-assist-session-" + self.assist_limits.profile.rsplit("-", 1)[-1].lower(),
                "protocol_version": self.assist_limits.profile,
                "guide_samples": list(self.guide_samples),
                "post_release_samples": list(self.post_release_samples),
                "passive_settle_samples": list(self.passive_settle_samples),
                "ready_streak": int(self.ready_streak),
                "collection_start_eligible": bool(self.collection_start_eligible),
                "assist_limits": asdict(self.assist_limits),
                "simulator_steps": 0,
                "active_action_budget_used": 0,
                "gripper_torque_enabled": False,
            }
        )
        return summary
