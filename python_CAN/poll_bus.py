"""Poll motor IDs 1-127 for their mechanical position without commanding motion.

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
RESPONSE_TIMEOUT = 0.1
POLL_INTERVAL = 0.5
MOTORS = {
    f"motor_{device_id}": Motor(id=device_id, model="rs-05")
    for device_id in range(1, 128)
}


class PositionPollingBus(RobstrideBus):
    """Bound position reads and ignore responses for other IDs or parameters."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._expected_device_id = None
        self._expected_parameter = None

    def read(self, motor, parameter_type):
        self._expected_device_id = self.motors[motor].id
        self._expected_parameter = parameter_type[0]
        try:
            return super().read(motor, parameter_type)
        finally:
            self._expected_device_id = None
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
                or kind != CommunicationType.READ_PARAMETER
                or (extra & 0xFF) != self._expected_device_id
                or len(frame.data) != 8
                or struct.unpack_from("<H", frame.data)[0] != self._expected_parameter
            ):
                continue
            return kind, extra, host_id, frame.data

        raise TimeoutError(
            f"No position response from motor ID {self._expected_device_id} "
            f"within {RESPONSE_TIMEOUT:.2f} seconds"
        )


def main() -> int:
    bus = PositionPollingBus(channel=CHANNEL, motors=MOTORS)
    try:
        bus.connect()
        print(f"Polling motor IDs 1-127 on {CHANNEL}; press Ctrl+C to stop.")
        while True:
            for name, motor in MOTORS.items():
                try:
                    position = bus.read(name, ParameterType.MECHANICAL_POSITION)
                    print(
                        f"ID {motor.id:03d}: position={position:.6f} rad "
                        f"({math.degrees(position):.2f} deg)"
                    )
                except TimeoutError:
                    print(f"ID {motor.id:03d}: no response")
            print(f"Scan complete; polling again in {POLL_INTERVAL:.1f} seconds.\n")
            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        print("\nPolling stopped.")
    except Exception as exc:
        print(f"CAN polling failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
