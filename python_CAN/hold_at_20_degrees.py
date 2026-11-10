"""Ramp to 20 degrees and log motor torque until Ctrl+C, including after faults.

Run from the repository root: python python_CAN/hold_at_20_degrees.py
"""

from datetime import datetime, timezone
import logging
import math
import os
from pathlib import Path
import signal
import struct
import sys
import time

# Use the RobstrideBus implementation included in this repository.
sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / "RobStride_Control" / "python")
)
from robstride_dynamics import CommunicationType, Motor, ParameterType, RobstrideBus


# Calibration from python_CAN/test; homing offsets are in radians.
CALIBRATION = {
    "motor_1": {"direction": -1, "homing_offset": -0.34},
    "motor_2": {"direction": 1, "homing_offset": -0.85},
}
MOTORS = {
    "motor_1": Motor(id=1, model="rs-01"),
    "motor_2": Motor(id=2, model="rs-01"),
}
CHANNEL = "COM15"
COMMON_TARGET = math.radians(20.0)
RAMP_DURATION = 6.0  # Seconds to reach 20 degrees from the measured start angles.
KP = 20.0  # Position stiffness (Nm/rad); tune for the actual load.
KD = 0.2  # Damping (Nm/(rad/s)).
PERIOD = 0.02  # Target 50 Hz per motor, including CAN response time.
RESPONSE_TIMEOUT = 0.1  # Bound all reads, including startup parameter reads.
MONITOR_PERIOD = 1.0  # Poll failed motors once per second without re-enabling them.


def create_logger():
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    log_path = Path(__file__).resolve().parent / f"motor_torque_{stamp}.txt"
    logger = logging.getLogger(f"motor_torque.{stamp}")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    formatter = logging.Formatter(
        "%(asctime)s.%(msecs)03dZ %(levelname)s %(message)s",
        datefmt="%Y-%m-%dT%H:%M:%S",
    )
    formatter.converter = time.gmtime
    file_handler = logging.FileHandler(log_path, mode="x", encoding="utf-8")
    console_handler = logging.StreamHandler(sys.stdout)
    for handler in (file_handler, console_handler):
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    # FileHandler flushes after every record, including every torque sample.
    logger.info("Log file: %s (timestamps in UTC)", log_path)
    return logger, file_handler


def sync_log(file_handler):
    """Also ask the OS to persist buffered data on faults and shutdown."""
    try:
        file_handler.flush()
        os.fsync(file_handler.stream.fileno())
    except OSError as exc:
        print(f"WARNING: could not sync torque log: {exc}", file=sys.stderr, flush=True)


class DiagnosticBus(RobstrideBus):
    """Bound reads and match replies so late frames cannot swap motor torques."""

    def __init__(self, *, logger, **kwargs):
        super().__init__(**kwargs)
        self.logger = logger
        self.faulted_motors = set()
        self._expected_motor = None
        self._expected_parameter = None
        self._last_status = {}

    def read(self, motor, parameter_type):
        self._expected_motor = motor
        self._expected_parameter = parameter_type[0]
        try:
            return super().read(motor, parameter_type)
        finally:
            self._expected_motor = self._expected_parameter = None

    def receive_status_frame(self, motor):
        self._expected_motor = motor
        try:
            return super().receive_status_frame(motor)
        finally:
            self._expected_motor = None

    def receive(self, timeout=None):
        deadline = time.monotonic() + (RESPONSE_TIMEOUT if timeout is None else timeout)
        names_by_id = {motor.id: name for name, motor in self.motors.items()}
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            frame = self.channel_handler.recv(timeout=remaining)
            if frame is None:
                break
            if not frame.is_extended_id:
                continue
            kind = (frame.arbitration_id >> 24) & 0x1F
            extra = (frame.arbitration_id >> 8) & 0xFFFF
            host_id = frame.arbitration_id & 0xFF
            source = names_by_id.get(extra & 0xFF)
            data = frame.data
            if host_id != self.host_id or source is None:
                continue

            # Preserve asynchronous faults even during another motor's request.
            if kind == CommunicationType.FAULT_REPORT:
                faults, warnings = struct.unpack("<LL", data)
                detail = f"{source} fault_report faults=0x{faults:08X} warnings=0x{warnings:08X}"
                self.logger.error(detail)
                self.faulted_motors.add(source)
                if source == self._expected_motor:
                    raise RuntimeError(detail)
                continue

            if kind == CommunicationType.OPERATION_STATUS:
                status = extra >> 8
                if self._last_status.get(source) != status:
                    self._last_status[source] = status
                    flag_names = (
                        "undervoltage", "overcurrent", "overtemperature",
                        "magnetic_encoder_fault", "stall", "uncalibrated",
                    )
                    flags = [name for bit, name in enumerate(flag_names) if status & (1 << bit)]
                    if flags:
                        self.faulted_motors.add(source)
                    self.logger.log(
                        logging.WARNING if flags else logging.INFO,
                        "%s status_mode=%d fault_flags=%s",
                        source, status >> 6, ",".join(flags) or "none",
                    )

            if source != self._expected_motor:
                continue
            if self._expected_parameter is not None:
                if kind != CommunicationType.READ_PARAMETER or len(data) != 8:
                    continue
                if struct.unpack_from("<H", data)[0] != self._expected_parameter:
                    continue
            elif kind != CommunicationType.OPERATION_STATUS:
                continue
            return kind, extra, host_id, data

        raise TimeoutError(f"No matching response from {self._expected_motor} within timeout")


def request_exit(signum, frame):
    """Handle SIGTERM through the same cleanup path as Ctrl+C."""
    raise KeyboardInterrupt


def main() -> int:
    try:
        logger, file_handler = create_logger()
    except OSError as exc:
        print(f"Cannot create torque log: {exc}", file=sys.stderr)
        return 1
    bus = DiagnosticBus(channel=CHANNEL, motors=MOTORS, calibration=CALIBRATION, logger=logger)
    signal.signal(signal.SIGTERM, request_exit)
    active_motors = set()

    def motor_error(name, stage, exc):
        active_motors.discard(name)
        logger.error(
            "%s torque=unavailable mode=monitor_only stage=%s error=%s: %s",
            name, stage, type(exc).__name__, exc,
        )
        sync_log(file_handler)
        if stage != "monitor":
            # A timeout alone does not prove that the motor stopped exerting torque.
            try:
                bus.disable(name)
                logger.info("%s disable confirmed; continuing to monitor without re-enabling", name)
            except Exception as disable_exc:
                logger.error("%s disable not confirmed: %s", name, disable_exc)
            sync_log(file_handler)

    try:
        logger.info(
            "channel=%s target_deg=%.1f ramp_seconds=%.1f KP=%.3f KD=%.3f calibration=%s",
            CHANNEL, math.degrees(COMMON_TARGET), RAMP_DURATION, KP, KD, CALIBRATION,
        )
        while not bus.is_connected:
            try:
                bus.connect()
                logger.info("CAN bus connected")
            except Exception as exc:
                logger.error("CAN connection failed; retrying: %s", exc)
                sync_log(file_handler)
                time.sleep(MONITOR_PERIOD)

        configured_motors = []
        for name in MOTORS:
            try:
                bus.disable(name)
                bus.write(name, ParameterType.MODE, 0)  # MIT position/torque control.
                configured_motors.append(name)
            except Exception as exc:
                motor_error(name, "setup", exc)

        start_positions = {}
        for name in configured_motors:
            try:
                # Parameter reads are raw; convert to the calibrated target frame.
                raw_position = bus.read(name, ParameterType.MECHANICAL_POSITION)
                calibration = CALIBRATION[name]
                position = (raw_position - calibration["homing_offset"]) * calibration["direction"]
                if not math.isfinite(position):
                    raise ValueError("Non-finite startup position")
                start_positions[name] = position
                logger.info("%s starting_angle_deg=%.3f", name, math.degrees(position))
            except Exception as exc:
                motor_error(name, "startup_position", exc)

        for name in start_positions:
            try:
                if name in bus.faulted_motors:
                    raise RuntimeError("Fault reported during setup; keeping motor disabled")
                bus.enable(name)
                active_motors.add(name)
            except Exception as exc:
                motor_error(name, "enable", exc)

        logger.info(
            "Ramping available motors to %.1f degrees over %.1f seconds, then holding. "
            "Ctrl+C stops monitoring and disables both motors.",
            math.degrees(COMMON_TARGET), RAMP_DURATION,
        )

        ramp_started = time.monotonic()
        next_monitor = {name: 0.0 for name in MOTORS}
        while True:
            cycle_started = time.monotonic()
            ramp_fraction = min(1.0, (cycle_started - ramp_started) / RAMP_DURATION)
            for name in MOTORS:
                if name in active_motors and name in bus.faulted_motors:
                    motor_error(name, "control", RuntimeError("Motor reported a fault; switching to monitoring only"))
                if name not in active_motors and cycle_started < next_monitor[name]:
                    continue
                stage = "control" if name in active_motors else "monitor"
                try:
                    if name in active_motors:
                        target_position = (
                            COMMON_TARGET
                            if ramp_fraction >= 1.0
                            else start_positions[name]
                            + ramp_fraction * (COMMON_TARGET - start_positions[name])
                        )
                        bus.write_operation_frame(
                            motor=name,
                            position=target_position,
                            kp=KP,
                            kd=KD,
                            velocity=0.0,
                            torque=4.0,  # Feedback torque below includes the KP/KD effort.
                        )
                        position, velocity, torque, temperature = bus.read_operation_frame(name)
                        logger.info(
                            "%s mode=control torque=%.4f Nm angle_deg=%.3f target_deg=%.3f "
                            "velocity_rad_s=%.4f temperature_C=%.1f",
                            name, torque, math.degrees(position), math.degrees(target_position),
                            velocity, temperature,
                        )
                    else:
                        # Read telemetry only; never reset faults or automatically re-enable.
                        next_monitor[name] = time.monotonic() + MONITOR_PERIOD
                        raw_torque = bus.read(name, ParameterType.MEASURED_TORQUE)
                        torque = raw_torque * CALIBRATION[name]["direction"]
                        logger.info("%s mode=monitor_only torque=%.4f Nm source=parameter", name, torque)
                except Exception as exc:
                    motor_error(name, stage, exc)
            time.sleep(max(0.0, PERIOD - (time.monotonic() - cycle_started)))
    except KeyboardInterrupt:
        logger.info("User requested stop; disabling motors")
    except Exception as exc:
        logger.exception("Unexpected program error: %s", exc)
        return 1
    finally:
        if bus.is_connected:
            # Attempt both disables even if one motor fails to respond.
            for name in MOTORS:
                try:
                    bus.disable(name)
                    logger.info("%s shutdown disable confirmed", name)
                except Exception as exc:
                    logger.error("%s shutdown disable not confirmed: %s", name, exc)
            try:
                bus.disconnect(disable_torque=False)
                logger.info("CAN bus disconnected")
            except Exception as exc:
                logger.error("CAN disconnect failed: %s", exc)
        logger.info("Monitoring stopped; saving log")
        sync_log(file_handler)
        for handler in list(logger.handlers):
            handler.close()
            logger.removeHandler(handler)

    return 0


if __name__ == "__main__":
    sys.exit(main())
