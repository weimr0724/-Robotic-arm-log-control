# Robotic-arm-log-control
Human-in-the-loop robot arm control system with real-time joint mapping, safety constraints, and serial communication.
## Overview
This project is designed to support a human-in-the-loop (HITL) robot arm control system that maps human input to robotic joint commands in real time.
The system focuses on safe, responsive control, safety constraints, and serial communication between software and hardware.


## System Workflow
Human Input → Mapping → Safety Filter → Serial → Robot Feedback

## Key Features
- Real-time joint angle mapping from human input
- Safety constraints with joint limit clamping
- Manual freeze and resume control for safe operation
- Step-based workflow execution
- Modular and extensible control architecture

## Design Decisions

### Human-in-the-Loop Control
The system is intentionally designed as a human-in-the-loop controller rather than a fully autonomous system.
This allows direct human supervision, intuitive control, and safer operation during early-stage development and testing.

### Real-Time Angle Mapping
Joint commands are generated through direct angle mapping to minimize latency and improve responsiveness.
This design choice prioritizes predictable behavior over complex motion planning.

### Safety Filtering Before Actuation
All joint commands pass through a safety filter before being sent to the robot.
Joint limit clamping is applied to prevent unsafe motions caused by sensor noise, sudden input changes, or communication delays.

### Serial Communication Architecture
A lightweight serial communication protocol is used to simplify integration between the host controller and embedded hardware.
This keeps the system hardware-agnostic and easy to debug.

### Modular Control Structure
The control logic is separated into mapping, safety, and communication modules.
This structure improves readability and allows future extensions such as additional degrees of freedom, sensor fusion, or alternative input devices.


## Tech Stack
- Python(pygame)
- Arduino
- Gazebo


## How to Run
1. Connect the robot controller and host computer via serial port
2. Upload the Arduino firmware to the controller
3. Configure the serial port in the Python script
4. Run the Python control program
5. Follow on-screen instructions to execute control workflows

## Safety Considerations
Joint commands are filtered through predefined limits before being sent to the robot.
This prevents unsafe motions caused by sensor noise, sudden input changes, or communication delays.

## Project Status
This project is under active development.
Future improvements may include additional degrees of freedom, sensor fusion, and higher-level control strategies.
