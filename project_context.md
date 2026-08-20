# Swarm Control Architecture: System Overview

This document outlines the closed-loop tracking system designed to validate differential-drive kinematics before executing Dr. Iftekhar's multi-robot flocking trajectories.

## 1. Offline Generation & Tuning

*   **Trajectory Generator (`generate_s_curve.py`):** A Python script that generates a parametric mathematical trajectory tailored to the 2.7m physical arena bounds. It calculates heading angles using trigonometric derivatives and outputs a hardware-ready CSV file, acting as a controlled substitute for complex experimental data.
*   **Manual Tuner (`manual_tune.py`):** An independent diagnostic tool used to bypass the central PC's vision loop. It allows direct UDP injection of target velocities and on-the-fly adjustment of the ESP32's internal PID parameters to mechanically tune hardware responses.

## 2. The Digital Brain (PC Controller)

*   **Vision Node (`calibrate_and_track_v3.py`):** Converts Rapoo camera frames to grayscale to reduce sensor noise and extracts ArUco marker poses using relaxed polygon approximations. It applies a homography matrix to map pixel coordinates to the physical floor and uses an Exponential Moving Average (EMA) filter to reject impossible physics glitches.
*   **Kinematics Engine (`unicycle_model_v2.py`):** Calculates spatial and heading errors to generate standard unicycle control signals. It translates these into independent wheel velocities, applying proportional deadband scaling to ensure both wheels receive enough minimum speed to overcome static floor friction without breaking the turning radius.
*   **Master Dispatcher (`swarm_controller.py`):** Loops over the generated trajectory CSV and requests tracking data. If the vision node loses sight of the robot, this script uses optical dead reckoning to predict the robot's continuing path. It calculates target velocities via the kinematics engine and transmits them over UDP to the hardware.

## 3. The Physical Muscle (ESP32 Firmware)

*   **Microcontroller Firmware:** A dual-core FreeRTOS application. Core 0 handles asynchronous UDP packet reception from the PC. Core 1 runs a strict 50ms control loop, reading hardware encoder ticks and executing a feedforward-assisted positional PID controller to force the physical N20 motors to match the PC's digital velocity targets.