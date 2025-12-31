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
