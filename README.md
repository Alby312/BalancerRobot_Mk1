# RP2040 SimpleFOC Self-Balancing Robot

A compact self-balancing robot project built around the **RP2040** microcontroller and the **SimpleFOC** library. This project utilizes the dual-core capabilities of the RP2040 to ensure high-performance motor control alongside robust stability algorithms.

## Hardware Components

* **MCU:** RP2040
* **Drivers:** 2x SimpleFOC Mini
* **Motors:** 2x 2804 Gimbal Motors
* **Encoders:** 2x MT6701 Magnetic Sensors (SSI interface)
* **IMU:** Adafruit BNO055
* **Radio:** ELRS Receiver (Optional, handled via AlfredoCRSF)
* **Battery:** Any Lithium battery should work

## Project Structure

* `BalancerRobot_PID_code/`: Code for PID controller version (PlatformIO).
* `3dFiles/`: Source and STL files for 3D printing the robot chassis and mounts.
* `Electronics/`: Schematics and wiring diagrams.

## Software Architecture

This project is developed using **PlatformIO** and utilizes the **Arduino** framework. The code takes advantage of the RP2040's dual-core architecture to separate high-frequency motor commutation from the control logic.

### Dual-Core Implementation
* **Core 1 (Motor Control):** Dedicated strictly to the `SimpleFOC` loop. It handles the FOC algorithms, sensor readings, and driver commutation to ensure smooth motor movement without interruption.
* **Core 0 (Robot Logic):** Handles the IMU readings (BNO055), Radio communication (ELRS), and the main stabilization control loop.

### Control Algorithm: Cascaded PID

The current implementation uses a **Cascaded PID Controller** strategy to maintain balance:

1.  **Velocity Loop (Outer Loop):** Reads the current motor velocity and compares it to the target speed (from RC remote or 0 for station keeping). The output of this loop determines the **Target Pitch Angle**.
2.  **Stabilization Loop (Inner Loop):** Compares the actual Pitch (from IMU) with the Target Pitch. The output determines the **Voltage** sent to the motors to correct the error.
3.  **Yaw/Steering Loop:** Adds a differential voltage to the motors to allow the robot to turn based on RC input.

*Note: This is the baseline PID control algorithm. More controllers are planned in the future.*

## Getting Started

1.  Clone this repository.
2.  Open the project in **PlatformIO**.
3.  Check `BalancerRobot_PID_code/main.cpp` to verify pin definitions match your wiring.
4.  Upload the code to your RP2040 board.
5.  **Calibration:** Ensure the robot is held upright when powering on for IMU initialization.