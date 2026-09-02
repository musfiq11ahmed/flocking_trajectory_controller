"""
main.py — MicroPython entry point for the ESP32-S3 motor controller.

This file auto-runs when the ESP32-S3 boots.  It creates a Robot instance
and enters the control loop.

Upload this file (along with pid.py, motor.py, robot.py) to the ESP32-S3
flash using Thonny, mpremote, or ampy.
"""

from robot import Robot


def main():
    bot = Robot()
    try:
        bot.run()          # blocking — never returns normally
    except KeyboardInterrupt:
        pass
    finally:
        bot.stop()
        print("Motors stopped.  Ctrl-C again for REPL.")


main()
