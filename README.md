# Automated Greenhouse Project - Automata Vitalis

## Overview
This Automated Greenhouse Project integrates advanced IoT technologies with M5Stack units to create a smart, self-regulating environment for indoor gardening. Designed for urban dwellers, it automates essential aspects of plant care, ensuring optimal growth conditions.

## Features
- **Automated Lighting Control**: Adjusts LED strips based on ambient light.
- **Precision Watering System**: Water pump activation according to soil moisture.
- **Temperature Regulation**: Monitors and adjusts greenhouse temperature.
- **Real-time Monitoring**: Environmental data displayed on M5Stack Core2’s LCD.
- **IoT Integration**: Uses MQTT for data transmission and Node-RED for workflow automation.

## Hardware Components
- `M5Stack Core2`
- `M5Stack DLight Unit` - connected to PAHUB on CH3
- `M5Stack ENV III Unit` - connected to PAHUB on CH0
- `M5Stack Ultrasonic Sensor Unit` - connected to PAHUB on CH1
- `M5Stack NCIR Unit` - connected to PAHUB on CH2
- `M5Stack PAHUB2` - connected to Port A on Core2
- `M5Stack Watering Unit` - connected to Port B on Core2
- `M5Stack RGB LED Strip Unit` - connected to Port C on Core2

## Software
The system is powered by a custom C++ program (`main.cpp`). This software is responsible for reading sensor data, processing it, and executing actions such as lighting adjustment, water pump control, and temperature regulation. 

### Installation
1. **Hardware Setup**: Assemble the greenhouse structure and connect all M5Stack units as per the Hardware Components section.
2. **Software Configuration**: Upload the `main.cpp` file to the M5Stack Core2. Ensure all necessary C++ libraries are installed.

## Usage
1. Power on the M5Stack Core2.
2. The system will automatically begin monitoring and adjusting the greenhouse environment.
3. Use the Node-RED dashboard for real-time data visualization and manual control.

## Contributing
Contributions to the project are welcome! Please read `CONTRIBUTING.md` for details on our code of conduct, and the process for submitting pull requests.

## License
This project is licensed under the MIT License - see the `LICENSE.md` file for details.

## Acknowledgments
- M5Stack Community https://github.com/m5stack
