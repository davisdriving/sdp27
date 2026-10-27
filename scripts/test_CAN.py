import math
import struct
import time

import can

PORT = "COM15"
CAN_BITRATE = 1_000_000
SERIAL_BAUDRATE = 115200

POSITION = 0x7019
BUS_VOLTAGE = 0x701C


def host_id_for(motor_id):
    return 0xFD if motor_id != 0xFD else 0xFC


def send_command(bus, command, motor_id, extra, data=bytes(8)):
    arbitration_id = (command << 24) | (extra << 8) | motor_id

    bus.send(
        can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=True,
            data=data,
        ),
        timeout=0.5,
    )
    return arbitration_id


def read_parameter(bus, motor_id, parameter, timeout=0.15):
    host_id = host_id_for(motor_id)
    payload = struct.pack("<H", parameter) + bytes(6)
    request_id = send_command(bus, 0x11, motor_id, host_id, payload)

    deadline = time.monotonic() + timeout

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None

        msg = bus.recv(timeout=remaining)
        if msg is None:
            return None

        if (
            not msg.is_extended_id
            or msg.is_error_frame
            or msg.is_remote_frame
            or len(msg.data) != 8
            or msg.arbitration_id == request_id
        ):
            continue

        response_type = (msg.arbitration_id >> 24) & 0x1F
        responding_motor = (msg.arbitration_id >> 8) & 0xFF
        destination = msg.arbitration_id & 0xFF
        returned_parameter = struct.unpack("<H", msg.data[:2])[0]

        if (
            response_type == 0x11
            and responding_motor == motor_id
            and destination == host_id
            and returned_parameter == parameter
        ):
            value = struct.unpack("<f", msg.data[4:8])[0]
            if math.isfinite(value):
                return value


def probe(bus, motor_id, attempts=2):
    for _ in range(attempts):
        position = read_parameter(bus, motor_id, POSITION)
        if position is not None:
            return position
    return None


def scan(bus):
    found = set()
    print("\nScanning IDs 0–255. Allow about 40 seconds...")

    for motor_id in range(256):
        position = read_parameter(bus, motor_id, POSITION)

        if position is not None:
            found.add(motor_id)
            voltage = read_parameter(bus, motor_id, BUS_VOLTAGE)

            voltage_text = (
                f"{voltage:.2f} V" if voltage is not None else "unavailable"
            )
            print(
                f"FOUND ID {motor_id:3d} (0x{motor_id:02X})"
                f" | position {position:.4f} rad"
                f" | voltage {voltage_text}"
            )

        if (motor_id + 1) % 32 == 0:
            print(f"  Checked {motor_id + 1}/256 IDs")

    return found


def disable_motor(bus, motor_id):
    host_id = host_id_for(motor_id)
    send_command(bus, 0x04, motor_id, host_id)
    deadline = time.monotonic() + 1.0

    while time.monotonic() < deadline:
        msg = bus.recv(timeout=max(0, deadline - time.monotonic()))

        if msg is None:
            break

        if (
            not msg.is_extended_id
            or msg.is_error_frame
            or msg.is_remote_frame
            or len(msg.data) != 8
        ):
            continue

        command = (msg.arbitration_id >> 24) & 0x1F
        source = (msg.arbitration_id >> 8) & 0xFF
        destination = msg.arbitration_id & 0xFF
        state = (msg.arbitration_id >> 22) & 0x03

        # Type-2 status, reset/disabled state.
        if (
            command == 0x02
            and source == motor_id
            and destination == host_id
            and state == 0
        ):
            return

    raise RuntimeError(
        "Could not confirm the motor is disabled. ID was not changed."
    )


def parse_id(text):
    text = text.strip()
    return int(text, 16) if text.lower().startswith("0x") else int(text)


def main():
    bus = None

    try:
        bus = can.Bus(
            interface="slcan",
            channel=PORT,
            tty_baudrate=SERIAL_BAUDRATE,
            bitrate=CAN_BITRATE,
        )

        found = scan(bus)

        while found:
            print("\nResponding IDs:", sorted(found))
            selection = input(
                "Current ID to change, 'r' to rescan, or 'q' to quit: "
            ).strip()

            if selection.lower() == "q":
                break

            if selection.lower() == "r":
                found = scan(bus)
                continue

            try:
                old_id = parse_id(selection)
                if old_id not in found:
                    print("Select an ID from the list.")
                    continue

                new_id = parse_id(input("New ID (1–127): "))
                if not 1 <= new_id <= 127:
                    print("Use a new ID between 1 and 127.")
                    continue
            except ValueError:
                print("Enter a decimal ID or hexadecimal such as 0x02.")
                continue

            if new_id == old_id:
                print("That is already the motor's ID.")
                continue

            # Refuse IDs found during scanning or a fresh probe.
            if new_id in found or probe(bus, new_id) is not None:
                print(f"ID {new_id} is already occupied. Choose another.")
                continue

            if probe(bus, old_id) is None:
                print("The selected motor is no longer responding. Rescan.")
                continue

            confirm = input(
                f"Disable motor {old_id} and change its ID to {new_id}? "
                "Type yes: "
            ).strip().lower()

            if confirm != "yes":
                print("Cancelled.")
                continue

            disable_motor(bus, old_id)

            # RobStride type 7:
            # bits 23–16 = new ID
            # bits 15–8  = host ID
            # bits 7–0   = current motor ID
            extra = (new_id << 8) | host_id_for(old_id)
            send_command(bus, 0x07, old_id, extra)

            time.sleep(0.2)

            new_position = probe(bus, new_id, attempts=3)
            old_position = probe(bus, old_id, attempts=2)

            if new_position is not None and old_position is None:
                found.remove(old_id)
                found.add(new_id)
                print(f"SUCCESS: ID {old_id} -> {new_id}")
                print(f"Position at new ID: {new_position:.4f} rad")
            else:
                print("\nChange could not be verified cleanly.")
                print(f"New ID responds: {new_position is not None}")
                print(f"Old ID responds: {old_position is not None}")
                print("Stopping. Check connections and rescan before retrying.")
                break

        if not found:
            print("No motors found. Check power, wiring, and CAN bitrate.")

    except KeyboardInterrupt:
        print("\nCancelled.")
    except Exception as exc:
        print(f"\nERROR: {type(exc).__name__}: {exc}")
    finally:
        if bus is not None:
            bus.shutdown()


if __name__ == "__main__":
    main()