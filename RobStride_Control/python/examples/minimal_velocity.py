import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.speed_control import SpeedController

controller = SpeedController(motor_id=1)

if controller.connect():
    controller.set_velocity(1.0)
    try:
        controller.loop()
    except KeyboardInterrupt:
        controller.stop_and_exit()
