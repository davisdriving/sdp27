#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride Basic Usage Example
Demonstrates basic motor control operations
"""

import sys
import os
import time
import math

# Add the python directory to path
sys.path.insert(0, os.path.join(r'C:\Users\mk3ow\OneDrive\Documents\PlatformIO\Projects\sdp27\RobStride_Control\python'))

try:
    from src.position_control import PositionControllerMIT
    from src.speed_control import SpeedController
except ImportError:
    print("❌ Cannot import motor control modules")
    sys.exit(1)

def test_position_control(motor_id=1):
    """Test position control with basic movements"""
    print(f"\n🎯 Testing Position Control (Motor {motor_id})")
    print("=" * 50)

    controller = PositionControllerMIT(motor_id)

    try:
        # Connect to motor
        if not controller.connect():
            print("❌ Failed to connect to motor")
            return False

        print("✅ Connected successfully!")

        # Test movements
        movements = [
            (0.0, "Home position"),
            (90.0, "90 degrees"),
            (-90.0, "-90 degrees"),
            (180.0, "180 degrees"),
            (0.0, "Back to home")
        ]

        for angle, description in movements:
            print(f"📍 Moving to {description}: {angle}°")
            controller.set_angle(angle)
            time.sleep(2)  # Wait for movement to complete

        print("✅ Position control test completed!")
        return True

    except Exception as e:
        print(f"❌ Position control test failed: {e}")
        return False
    finally:
        controller.stop_and_exit()

def test_speed_control(motor_id=1):
    """Test speed control with basic movements"""
    print(f"\n⚡ Testing Speed Control (Motor {motor_id})")
    print("=" * 50)

    controller = SpeedController(motor_id)

    try:
        # Connect to motor
        if not controller.connect():
            print("❌ Failed to connect to motor")
            return False

        print("✅ Connected successfully!")

        # Test speeds
        speeds = [
            (2.0, "Forward 2.0 rad/s"),
            (5.0, "Forward 5.0 rad/s"),
            (0.0, "Stop"),
            (-2.0, "Reverse 2.0 rad/s"),
            (-5.0, "Reverse 5.0 rad/s"),
            (0.0, "Stop")
        ]

        for speed, description in speeds:
            print(f"🚀 {description}: {speed} rad/s")
            controller.set_velocity(speed)
            time.sleep(3.0)  # Run for 3 seconds

        print("✅ Speed control test completed!")
        return True

    except Exception as e:
        print(f"❌ Speed control test failed: {e}")
        return False
    finally:
        controller.stop_and_exit()

def interactive_demo(motor_id=11):
    """Interactive demo for user experimentation"""
    print(f"\n🎮 Interactive Demo (Motor {motor_id})")
    print("=" * 50)
    print("Choose control mode:")
    print("1. Position Control")
    print("2. Speed Control")
    print("3. Both (alternating)")

    try:
        choice = input("Enter choice (1-3): ").strip()

        if choice == '1':
            return test_position_control(motor_id)
        elif choice == '2':
            return test_speed_control(motor_id)
        elif choice == '3':
            result1 = test_position_control(motor_id)
            time.sleep(1.0)
            result2 = test_speed_control(motor_id)
            return result1 and result2
        else:
            print("❌ Invalid choice")
            return False

    except KeyboardInterrupt:
        print("\n👋 Demo interrupted by user")
        return False

def main():
    """Main function"""
    print("🎯 RobStride Basic Usage Example")
    print("=" * 50)

    # Get motor ID from command line or use default
    motor_id = 1
    if len(sys.argv) > 1:
        try:
            motor_id = int(sys.argv[1])
        except ValueError:
            print("❌ Invalid motor ID. Using default (1)")

    print(f"Using Motor ID: {motor_id}")

    # Check if CAN interface is available
    if not os.path.exists('/sys/class/net/can0'):
        print("⚠️ Warning: can0 interface not found")
        print("Please setup CAN interface:")
        print("  sudo ip link set can0 type can bitrate 1000000")
        print("  sudo ip link set up can0")
        print()

    try:
        # Run interactive demo
        success = interactive_demo(motor_id)

        if success:
            print("\n🎉 Demo completed successfully!")
            return 0
        else:
            print("\n❌ Demo failed!")
            return 1

    except Exception as e:
        print(f"\n💥 Unexpected error: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())