from typing import Iterable, Mapping
import numpy as np
from gello_hil.dynamixel import COUNTS_PER_REVOLUTION
from gello_hil.actuation import (
 CURRENT_BASED_POSITION_MODE, ACTIVE_MONITOR_ITEM_NAMES, READ_ITEMS, WRITE_ITEMS,
 XL330_VELOCITY_UNIT_RAD_S,
)

class FakeXL330Transport:
    """Deterministic in-memory XL330 backend for no-hardware tests and smoke."""

    def __init__(
        self,
        *,
        model_numbers: Mapping[int, int],
        present_positions: Mapping[int, int],
        operating_mode: int = CURRENT_BASED_POSITION_MODE,
        maximum_follow_counts_per_tick: int = 2,
        control_hz: float = 10.0,
    ):
        if int(maximum_follow_counts_per_tick) < 1:
            raise ValueError("fake XL330 follow step must be at least one count")
        if not np.isfinite(control_hz) or float(control_hz) <= 0:
            raise ValueError("fake XL330 control rate must be positive")
        self.model_numbers = {int(key): int(value) for key, value in model_numbers.items()}
        self.maximum_follow_counts_per_tick = int(maximum_follow_counts_per_tick)
        self.control_hz = float(control_hz)
        self.registers: dict[int, dict[str, int]] = {}
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
        self.fail_next_read: str | None = None
        self.fail_next_write: str | None = None
        for motor_id, model in self.model_numbers.items():
            position = int(present_positions[motor_id])
            self.registers[motor_id] = {
                "model_number": model,
                "firmware_version": 53,
                "drive_mode": 0,
                "operating_mode": int(operating_mode),
                "current_limit_ma": 1750,
                "startup_configuration": 0,
                "shutdown_mask": 52,
                "torque_enable": 0,
                "hardware_error": 0,
                "position_d_gain": 0,
                "position_i_gain": 0,
                "position_p_gain": 0,
                "bus_watchdog": 0,
                "goal_current_ma": 0,
                "goal_position_counts": position,
                "present_current_ma": 0,
                "present_velocity_raw": 0,
                "present_position_counts": position,
                "input_voltage_raw": 50,
                "temperature_c": 25,
            }

    def ping(self, motor_id: int) -> dict:
        motor_id = int(motor_id)
        self.instruction_counts["PING"] += 1
        if motor_id not in self.registers:
            raise TimeoutError(f"missing fake Dynamixel ID {motor_id}")
        return {
            "id": motor_id,
            "model_number": self.registers[motor_id]["model_number"],
            "firmware_version": self.registers[motor_id]["firmware_version"],
        }

    def _maybe_move(self, motor_id: int) -> None:
        registers = self.registers[motor_id]
        if (
            registers["torque_enable"]
            and registers["goal_current_ma"] > 0
            and registers["position_p_gain"] > 0
        ):
            error = registers["goal_position_counts"] - registers["present_position_counts"]
            # One fake dynamics update represents one controller telemetry
            # tick.  P0 keeps the default two-count follower, while the
            # separate HOME-P1 smoke opts into its registered homing pace.
            step = int(
                np.clip(
                    error,
                    -self.maximum_follow_counts_per_tick,
                    self.maximum_follow_counts_per_tick,
                )
            )
            registers["present_position_counts"] += step
            # The velocity register is not counts/tick: one raw unit is
            # XL330_VELOCITY_UNIT_RAD_S.  Convert the deterministic fake
            # displacement at the configured controller rate so HOME-P1's
            # physical velocity gate is exercised in the right units.
            registers["present_velocity_raw"] = int(
                np.rint(
                    step
                    * (2.0 * np.pi / COUNTS_PER_REVOLUTION)
                    * self.control_hz
                    / XL330_VELOCITY_UNIT_RAD_S
                )
            )
            registers["present_current_ma"] = min(
                abs(error), registers["goal_current_ma"]
            )
        else:
            registers["present_velocity_raw"] = 0
            registers["present_current_ma"] = 0

    def read_control_item(self, motor_id: int, name: str) -> int:
        motor_id = int(motor_id)
        self.instruction_counts["READ"] += 1
        if self.fail_next_read is not None:
            message = self.fail_next_read
            self.fail_next_read = None
            raise TimeoutError(message)
        if name not in READ_ITEMS:
            raise ValueError(f"fake read item is not allowlisted: {name}")
        if motor_id not in self.registers:
            raise TimeoutError(f"missing fake Dynamixel ID {motor_id}")
        # Advancing only when Present Position is sampled gives exactly one
        # dynamics update per controller snapshot instead of one update per
        # register read.
        if name == "present_position_counts":
            self._maybe_move(motor_id)
        return int(self.registers[motor_id][name])

    def read_active_snapshot(self, motor_id: int) -> dict[str, int]:
        motor_id = int(motor_id)
        self.instruction_counts["READ"] += 1
        if self.fail_next_read is not None:
            message = self.fail_next_read
            self.fail_next_read = None
            raise TimeoutError(message)
        if motor_id not in self.registers:
            raise TimeoutError(f"missing fake Dynamixel ID {motor_id}")
        self._maybe_move(motor_id)
        return {
            name: int(self.registers[motor_id][name])
            for name in ACTIVE_MONITOR_ITEM_NAMES
        }

    def sync_read_active_snapshots(
        self, motor_ids: Iterable[int]
    ) -> dict[int, dict[str, int]]:
        self.instruction_counts["SYNC_READ"] += 1
        if self.fail_next_read is not None:
            message = self.fail_next_read
            self.fail_next_read = None
            raise TimeoutError(message)
        ids = tuple(int(value) for value in motor_ids)
        if not ids or len(set(ids)) != len(ids):
            raise ValueError("fake active snapshot IDs must be unique and nonempty")
        if any(motor_id not in self.registers for motor_id in ids):
            raise TimeoutError("missing fake Dynamixel ID in active SYNC_READ")
        result: dict[int, dict[str, int]] = {}
        for motor_id in ids:
            self._maybe_move(motor_id)
            result[motor_id] = {
                name: int(self.registers[motor_id][name])
                for name in ACTIVE_MONITOR_ITEM_NAMES
            }
        return result

    def sync_read_control_item(
        self, name: str, motor_ids: Iterable[int]
    ) -> dict[int, int]:
        self.instruction_counts["SYNC_READ"] += 1
        if self.fail_next_read is not None:
            message = self.fail_next_read
            self.fail_next_read = None
            raise TimeoutError(message)
        if name not in READ_ITEMS:
            raise ValueError(f"fake sync-read item is not allowlisted: {name}")
        ids = tuple(int(value) for value in motor_ids)
        if not ids or len(set(ids)) != len(ids):
            raise ValueError("fake sync-read IDs must be unique and nonempty")
        if any(motor_id not in self.registers for motor_id in ids):
            raise TimeoutError("missing fake Dynamixel ID in SYNC_READ")
        return {
            motor_id: int(self.registers[motor_id][name]) for motor_id in ids
        }

    def _record_write(self, motor_id: int, name: str, value: int) -> None:
        if name not in WRITE_ITEMS:
            raise ValueError(f"fake write item is not allowlisted: {name}")
        motor_id = int(motor_id)
        value = int(value)
        registers = self.registers[motor_id]
        entry = {
            "instruction": "WRITE",
            "motor_ids": [motor_id],
            "item": name,
            "address": WRITE_ITEMS[name].address,
            "value": value,
            "acknowledged": False,
        }
        self.motor_register_writes += 1
        self.write_packets += 1
        self.instruction_counts["WRITE"] += 1
        self.command_log.append(entry)
        if self.fail_next_write is not None:
            message = self.fail_next_write
            self.fail_next_write = None
            raise IOError(message)
        if name == "operating_mode":
            if registers["torque_enable"]:
                raise IOError("fake mode write rejected while torque enabled")
            if value != CURRENT_BASED_POSITION_MODE:
                raise ValueError("fake backend permits mode 5 only")
            registers["operating_mode"] = value
            registers["position_p_gain"] = 0
            registers["position_i_gain"] = 0
            registers["position_d_gain"] = 0
            registers["goal_current_ma"] = 0
            registers["goal_position_counts"] = registers["present_position_counts"]
        elif name == "torque_enable":
            if value not in (0, 1):
                raise ValueError("fake torque enable must be 0 or 1")
            registers[name] = value
            if value:
                registers["goal_position_counts"] = registers[
                    "present_position_counts"
                ]
        else:
            registers[name] = value
        entry["acknowledged"] = True

    def set_operating_mode_current_based_position(self, motor_id: int) -> None:
        self._record_write(motor_id, "operating_mode", 5)

    def set_torque_enabled(self, motor_id: int, enabled: bool) -> None:
        self._record_write(motor_id, "torque_enable", int(bool(enabled)))

    def set_position_gains(self, motor_id: int, *, p: int, i: int, d: int) -> None:
        self._record_write(motor_id, "position_d_gain", int(d))
        self._record_write(motor_id, "position_i_gain", int(i))
        self._record_write(motor_id, "position_p_gain", int(p))

    def set_bus_watchdog_ticks(self, motor_id: int, ticks: int) -> None:
        self._record_write(motor_id, "bus_watchdog", int(ticks))

    def set_goal_current_ma(self, motor_id: int, current_ma: int) -> None:
        self._record_write(motor_id, "goal_current_ma", int(current_ma))

    def set_goal_position_counts(self, motor_id: int, counts: int) -> None:
        self._record_write(motor_id, "goal_position_counts", int(counts))

    def set_goal_positions_counts(self, goals: Mapping[int, int]) -> None:
        if len(goals) != 1:
            raise ValueError(
                "GELLO-ACT-P0 Goal Position SYNC_WRITE requires exactly one motor"
            )
        for motor_id, value in goals.items():
            self._record_write(int(motor_id), "goal_position_counts", int(value))
