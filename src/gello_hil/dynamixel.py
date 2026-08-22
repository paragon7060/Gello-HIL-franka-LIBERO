"""Minimal read-only Dynamixel Protocol 2.0 transport.

Only PING, READ, and SYNC_READ packets can be constructed.  This module cannot
write torque, goal position, gain, operating mode, current, or any register.
"""

from __future__ import annotations

import fcntl
import os
import select
import termios
import time
from pathlib import Path
from typing import ClassVar

PROTOCOL2_HEADER = bytes((0xFF, 0xFF, 0xFD, 0x00))
PING = 0x01
READ = 0x02
SYNC_READ = 0x82
STATUS = 0x55
BROADCAST_ID = 0xFE
PRESENT_POSITION_ADDRESS = 132
COUNTS_PER_REVOLUTION = 4096
ALLOWED_INSTRUCTIONS = frozenset((PING, READ, SYNC_READ))


def protocol2_crc(payload: bytes) -> int:
    crc = 0
    for byte in payload:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def protocol2_packet(motor_id: int, instruction: int, parameters: bytes = b"") -> bytes:
    if not (0 <= motor_id <= 252 or motor_id == BROADCAST_ID):
        raise ValueError("Dynamixel ID must be 0..252 or broadcast ID 254")
    if instruction not in ALLOWED_INSTRUCTIONS:
        raise ValueError("read-only transport permits only PING, READ, and SYNC_READ")
    if instruction == SYNC_READ and motor_id != BROADCAST_ID:
        raise ValueError("SYNC_READ must use the broadcast ID")
    body = bytes((motor_id,)) + (len(parameters) + 3).to_bytes(2, "little")
    body += bytes((instruction,)) + parameters
    packet = PROTOCOL2_HEADER + body
    return packet + protocol2_crc(packet).to_bytes(2, "little")


class ReadOnlyDynamixelBus:
    """Exclusive serial reader for GELLO Present Position telemetry."""

    _baud_constants: ClassVar[dict[int, int]] = {
        57600: termios.B57600,
        115200: termios.B115200,
    }

    def __init__(self, port: str, *, baudrate: int = 57600, timeout_seconds: float = 0.12):
        if baudrate not in self._baud_constants:
            raise ValueError(f"unsupported baud rate: {baudrate}")
        if timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be positive")
        self.requested_port = str(port)
        self.resolved_port = str(Path(port).resolve(strict=True))
        self.timeout_seconds = float(timeout_seconds)
        self.fd = os.open(self.resolved_port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[3] = 0
            attrs[4] = self._baud_constants[baudrate]
            attrs[5] = self._baud_constants[baudrate]
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
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

    def __enter__(self):
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    def _read_status(self, expected_id: int) -> tuple[int, bytes]:
        deadline = time.monotonic() + self.timeout_seconds
        buffer = bytearray()
        while time.monotonic() < deadline:
            ready, _, _ = select.select([self.fd], [], [], max(0.0, deadline - time.monotonic()))
            if ready:
                buffer.extend(os.read(self.fd, 256))
            header = buffer.find(PROTOCOL2_HEADER)
            if header < 0:
                if len(buffer) > 3:
                    del buffer[:-3]
                continue
            if header:
                del buffer[:header]
            if len(buffer) < 7:
                continue
            total_length = 7 + int.from_bytes(buffer[5:7], "little")
            if len(buffer) < total_length:
                continue
            packet = bytes(buffer[:total_length])
            if protocol2_crc(packet[:-2]) != int.from_bytes(packet[-2:], "little"):
                raise OSError("Dynamixel status CRC mismatch")
            if packet[4] != expected_id or packet[7] != STATUS:
                raise OSError("unexpected Dynamixel status packet")
            return int(packet[8]), packet[9:-2]
        raise TimeoutError(f"Dynamixel ID {expected_id} did not answer")

    def _transaction(self, motor_id: int, instruction: int, parameters: bytes = b""):
        if instruction not in (PING, READ):
            raise ValueError("single-motor transaction permits only PING and READ")
        termios.tcflush(self.fd, termios.TCIFLUSH)
        packet = protocol2_packet(motor_id, instruction, parameters)
        if os.write(self.fd, packet) != len(packet):
            raise OSError("short Dynamixel serial write")
        termios.tcdrain(self.fd)
        return self._read_status(motor_id)

    def ping(self, motor_id: int) -> dict:
        error, parameters = self._transaction(int(motor_id), PING)
        if len(parameters) != 3:
            raise OSError("malformed PING response")
        return {
            "id": int(motor_id),
            "error": int(error),
            "model_number": int.from_bytes(parameters[:2], "little"),
            "firmware_version": int(parameters[2]),
        }

    def read(self, motor_id: int, address: int, length: int) -> tuple[int, bytes]:
        params = int(address).to_bytes(2, "little") + int(length).to_bytes(2, "little")
        error, values = self._transaction(int(motor_id), READ, params)
        if len(values) != length:
            raise OSError(f"Dynamixel ID {motor_id} returned {len(values)} of {length} bytes")
        return error, values

    def _read_many_status(self, expected_ids: tuple[int, ...], length: int) -> dict[int, bytes]:
        deadline = time.monotonic() + self.timeout_seconds
        expected = set(expected_ids)
        responses: dict[int, bytes] = {}
        buffer = bytearray()
        while time.monotonic() < deadline and set(responses) != expected:
            ready, _, _ = select.select([self.fd], [], [], max(0.0, deadline - time.monotonic()))
            if ready:
                buffer.extend(os.read(self.fd, 512))
            while True:
                header = buffer.find(PROTOCOL2_HEADER)
                if header < 0:
                    if len(buffer) > 3:
                        del buffer[:-3]
                    break
                if header:
                    del buffer[:header]
                if len(buffer) < 7:
                    break
                total = 7 + int.from_bytes(buffer[5:7], "little")
                if len(buffer) < total:
                    break
                packet = bytes(buffer[:total])
                del buffer[:total]
                if protocol2_crc(packet[:-2]) != int.from_bytes(packet[-2:], "little"):
                    raise OSError("Dynamixel SYNC_READ status CRC mismatch")
                motor_id = int(packet[4])
                if motor_id not in expected or packet[7] != STATUS:
                    raise OSError("unexpected SYNC_READ status packet")
                if packet[8]:
                    raise OSError(f"Dynamixel ID {motor_id} hardware error 0x{packet[8]:02x}")
                values = packet[9:-2]
                if len(values) != length:
                    raise OSError("malformed SYNC_READ response")
                responses[motor_id] = values
        missing = sorted(expected - set(responses))
        if missing:
            raise TimeoutError(f"Dynamixel SYNC_READ missing IDs {missing}")
        return responses

    def read_positions(self, motor_ids: tuple[int, ...] | list[int]) -> dict[int, int]:
        ids = tuple(int(value) for value in motor_ids)
        if not ids or len(set(ids)) != len(ids):
            raise ValueError("motor IDs must be non-empty and unique")
        if len(ids) == 1:
            error, values = self.read(ids[0], PRESENT_POSITION_ADDRESS, 4)
            if error:
                raise OSError(f"Dynamixel ID {ids[0]} hardware error 0x{error:02x}")
            return {ids[0]: int.from_bytes(values, "little") % COUNTS_PER_REVOLUTION}
        params = (
            PRESENT_POSITION_ADDRESS.to_bytes(2, "little") + (4).to_bytes(2, "little") + bytes(ids)
        )
        termios.tcflush(self.fd, termios.TCIFLUSH)
        packet = protocol2_packet(BROADCAST_ID, SYNC_READ, params)
        if os.write(self.fd, packet) != len(packet):
            raise OSError("short Dynamixel SYNC_READ serial write")
        termios.tcdrain(self.fd)
        responses = self._read_many_status(ids, 4)
        return {
            motor_id: int.from_bytes(responses[motor_id], "little") % COUNTS_PER_REVOLUTION
            for motor_id in ids
        }
