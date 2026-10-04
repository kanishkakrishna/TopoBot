# TopoBot: Embedded Firmware & Simulation Framework

TopoBot is a production-ready C++ firmware architecture designed for deterministic, point-to-point indoor material transport. Targeted for the ESP32 microcontroller, this project replaces computationally expensive LiDAR-SLAM architectures with a highly efficient two-tier navigation system: a reactive PID control loop for lateral path correction and an RFID-based finite state machine (FSM) for topological routing.

Note: This repository focuses on the embedded software architecture, control theory, and Software-in-the-Loop (SITL) validation of the system prior to physical hardware deployment.

# Features
Two-Tier Control Architecture: Combines continuous discrete-time PID control with an event-driven Finite State Machine (FSM).

Sensor Fusion & Processing: Computes a normalized weighted centroid from simulated 8-channel TCRT5000 IR sensor data to generate continuous-valued lateral error signals.

Discrete Event Routing: FSM executes complex branching logic (90-degree pivots, 180-degree delivery reorientations, timed payload dwells) based on SPI-triggered MFRC522 RFID interrupts.

Hardware-Software Interfacing: Modulates dual-channel PWM duty cycles (1kHz frequency, 8-bit resolution) for differential drive motor control while preventing integer overflow via dynamic output constraining.

Live Telemetry & Tuning: Integrated Bluetooth serial command parsing enables real-time tuning of Kp, Ki, and Kd parameters without requiring firmware recompilation.

Dynamic Route Reconfiguration: Routing logic is entirely reconfigurable through RFID tag mapping, requiring zero codebase modifications.

 ## Software-in-the-Loop (SITL) Simulation Guide
The firmware logic is validated without physical hardware dependency using the Wokwi ESP32 virtual environment.

Environment Setup: Load the main.ino code into the Wokwi ESP32 Simulator.

Sensor Mocking: Toggle virtual digital GPIO states to simulate the IR array drifting off the guide path.

Actuation Validation: Monitor the Left PWM (Pin 11) and Right PWM (Pin 23) outputs using virtual logic analyzers or LEDs to verify the proportional PID control response.

FSM Triggers: Inject mock RFID UIDs via the Serial Terminal to verify that the State Machine safely suspends the PID loop and transitions into intersection-turning logic.

## Repository Contents
src/main.ino: Complete ESP32 C++ firmware containing the PID control loop, SPI hardware interfacing, and FSM routing logic.

docs/SmartLogisticsBot_Hardware_Proposal.pdf: Theoretical system wiring schematic and documentation outlining the hardware architecture for future physical deployment.
