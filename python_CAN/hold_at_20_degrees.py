"""Run motor ID 2 at 40 RPM clockwise for 10 seconds.

Run from the repository root: python python_CAN/hold_at_20_degrees.py
"""

import math
import struct
import sys
import time

# Use the RobstrideBus implementation included in this repository.
sys.path.insert(0, r'C:\Users\mk3ow\OneDrive\Documents\PlatformIO\Projects\sdp27\RobStride_Control\python')

from robstride_dynamics import CommunicationType, Motor, ParameterType, RobstrideBus


CHANNEL = "COM15"
MOTOR_NAME = "motor_2"
MOTOR_ID = 1
MOTOR_MODEL = "rs-05"
TARGET_RPM = 400.0
RUN_SECONDS = 10.0
RESPONSE_TIMEOUT = 0.1
VELOCITY_KP = 2.0
VELOCITY_KI = 0.5
CURRENT_LIMIT_A = 5.0  # limit_cur (0x7018): max phase current in velocity mode, 0-23 A

# Calibration from the previous motor-2 setup.
CALIBRATION_DIRECTION = -1
HOMING_OFFSET = -0.972
TARGET_VELOCITY = CALIBRATION_DIRECTION * TARGET_RPM * (2.0 * math.pi / 60.0)
MOTORS = {MOTOR_NAME: Motor(id=MOTOR_ID, model=MOTOR_MODEL)}


class MotorControlBus(RobstrideBus):
    """Bound responses and match them to the motor and request in progress."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._expected_device_id = None
        self._expected_kind = None
        self._expected_parameter = None

    def read(self, motor, parameter_type):
        self._expected_device_id = self.motors[motor].id
        self._expected_kind = CommunicationType.READ_PARAMETER
        self._expected_parameter = parameter_type[0]
        try:
            return super().read(motor, parameter_type)
        finally:
            self._clear_expectation()

    def receive_status_frame(self, motor):
        self._expected_device_id = self.motors[motor].id
        self._expected_kind = CommunicationType.OPERATION_STATUS
        try:
            return super().receive_status_frame(motor)
        finally:
            self._clear_expectation()

    def _clear_expectation(self):
        self._expected_device_id = None
        self._expected_kind = None
        self._expected_parameter = None

    def receive(self, timeout=None):
        deadline = time.monotonic() + (RESPONSE_TIMEOUT if timeout is None else timeout)
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
            if (
                host_id != self.host_id
                or kind != self._expected_kind
                or (extra & 0xFF) != self._expected_device_id
            ):
                continue
            if self._expected_kind == CommunicationType.READ_PARAMETER and (
                len(frame.data) != 8
                or struct.unpack_from("<H", frame.data)[0] != self._expected_parameter
            ):
                continue
            return kind, extra, host_id, frame.data

        raise TimeoutError(
            f"No matching response from motor ID {self._expected_device_id} "
            f"within {RESPONSE_TIMEOUT:.2f} seconds"
        )


def print_motor_angle(bus, label):
    raw_position = bus.read(MOTOR_NAME, ParameterType.MECHANICAL_POSITION)
    angle = (raw_position - HOMING_OFFSET) * CALIBRATION_DIRECTION
    print(
        f"Motor ID {MOTOR_ID} angle {label}: "
        f"{angle:.6f} rad ({math.degrees(angle):.2f} deg)"
    )


def main() -> int:
    bus = MotorControlBus(channel=CHANNEL, motors=MOTORS)
    enabled = False
    try:
        bus.connect()
        # run_mode is only accepted while the motor is disabled, so set
        # velocity mode (2) first and enable afterwards.
        bus.write(MOTOR_NAME, ParameterType.MODE, 2)
        enabled = True
        bus.enable(MOTOR_NAME)
        bus.write(MOTOR_NAME, ParameterType.CURRENT_LIMIT, CURRENT_LIMIT_A)
        bus.write(MOTOR_NAME, ParameterType.VELOCITY_LIMIT, abs(TARGET_VELOCITY))
        bus.write(MOTOR_NAME, ParameterType.VELOCITY_KP, VELOCITY_KP)
        bus.write(MOTOR_NAME, ParameterType.VELOCITY_KI, VELOCITY_KI)
        bus.write(MOTOR_NAME, ParameterType.VELOCITY_TARGET, 0.0)
        bus.write(MOTOR_NAME, ParameterType.VELOCITY_TARGET, TARGET_VELOCITY)
        print(
            f"Motor ID {MOTOR_ID} commanded to {TARGET_RPM:.0f} RPM clockwise "
            f"for {RUN_SECONDS:.0f} seconds."
        )
        print_motor_angle(bus, "after command")
        time.sleep(RUN_SECONDS)
        bus.write(MOTOR_NAME, ParameterType.VELOCITY_TARGET, 0.0)
        time.sleep(0.2)
        print_motor_angle(bus, "after run")
        bus.disable(MOTOR_NAME)
        enabled = False
        print("Motor stopped and disabled.")
    except KeyboardInterrupt:
        print("\nInterrupted; attempting to stop the motor.")
        return 130
    except Exception as exc:
        print(f"Motor control failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        if bus.is_connected:
            if enabled:
                try:
                    bus.write(MOTOR_NAME, ParameterType.VELOCITY_TARGET, 0.0)
                except Exception as exc:
                    print(f"Could not confirm zero-speed command: {exc}", file=sys.stderr)
                try:
                    bus.disable(MOTOR_NAME)
                except Exception as exc:
                    print(f"Could not confirm motor disable: {exc}", file=sys.stderr)
            try:
                bus.disconnect(disable_torque=False)
            except Exception as exc:
                print(f"CAN disconnect failed: {exc}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
