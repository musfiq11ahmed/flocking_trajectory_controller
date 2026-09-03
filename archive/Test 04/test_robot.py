"""
Root runner for Robot Component Diagnostics.
Delegates to pc/test_robot.py.
"""
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PC_DIR = os.path.join(SCRIPT_DIR, "pc")
if PC_DIR not in sys.path:
    sys.path.insert(0, PC_DIR)

from test_robot import main

if __name__ == "__main__":
    main()
