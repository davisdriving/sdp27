import time

from robstride_dynamics import Motor, RobstrideBus

# --------- CONSTANTS ---------

CHANNEL = "COM15"  # serial port for CANable
MOTOR_MODEL = "rs-01"  # model string used to look up the scaling tables in `table.py`
CURRENT_LIMIT = 3.0  # phase current limit (A) used by the velocity loop
ACCELERATION = 10.0  # velocity-mode acceleration (rad/s^2)
COMMANDS_PER_RUN = 5
HOLD_SECONDS = 6
STOP_SECONDS = 1

motor_ids = [1, 2] # CAN ids of the motors to control. set these w/ read_n_set_CAN_ids.py 


def main():

    motor_names = {motor_id: f"motor_{motor_id}" for motor_id in motor_ids}
    bus = RobstrideBus(
        channel=CHANNEL,
        motors={name: Motor(id=motor_id, model=MOTOR_MODEL) for motor_id, name in motor_names.items()},
    )

    try:
        bus.connect()

        for name in motor_names.values():
            bus.disable(name)
            bus.set_run_mode(name, 2)
            bus.enable(name)

        for command_number in range(1, COMMANDS_PER_RUN + 1):
            while True:
                try:
                    motor_id = int(
                        input(
                            f"Command motor ID {motor_ids} "
                            f"(command {command_number}/{COMMANDS_PER_RUN}): "
                        )
                    )
                except ValueError:
                    print(f"Enter one of the listed motor IDs: {motor_ids}.")
                    continue
                if motor_id not in motor_names:
                    print(f"Motor ID must be one of {motor_ids}.")
                    continue
                break

            target_velocity = float(input("Input target velocity in rad/s: "))
            motor_name = motor_names[motor_id]
            bus.set_target_velocity(
                motor=motor_name,
                velocity=target_velocity,
                acceleration=ACCELERATION,
                current_limit=CURRENT_LIMIT,
            )
            time.sleep(HOLD_SECONDS)
            bus.set_target_velocity(motor_name, velocity=0.0)
            time.sleep(STOP_SECONDS)
    finally:
        if bus.is_connected:
            try:
                for name in motor_names.values():
                    bus.set_target_velocity(name, velocity=0.0)
                time.sleep(0.5)
            finally:
                try:
                    for name in motor_names.values():
                        bus.disable(name)
                finally:
                    if bus.is_connected:
                        bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()