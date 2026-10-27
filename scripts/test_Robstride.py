from robstride_dynamics import RobstrideBus, Motor, ParameterType

bus = RobstrideBus(
    channel="COM15",
    motors={"motor": Motor(id=127, model="rs-01")},
)

try:
    bus.connect()

    position = bus.read("motor", ParameterType.MECHANICAL_POSITION)
    print(f"Position: {position:.4f} rad")

finally:
    if bus.is_connected:
        bus.disconnect(disable_torque=False)