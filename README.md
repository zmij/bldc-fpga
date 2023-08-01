# bldc-fpga
## Tangnano4K BLDC-driver

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

### Description
A work-in-progress project aimed at driving a BLDC motor using a Gowin GW1NSR-4C FPGA chip with a hard Cortex-M3 MCU core. This project provides an affordable solution for BLDC motor control.

### Technologies Used
- MCU Part:
  - Programming Language: C++
  - Build System: CMake + arm-none-eabi-gcc toolchain
- FPGA Part:
  - Programming Language: SystemVerilog
  - Tools: Gowin EDA

### Key Features
- Table-based BLDC communication
- Field-Oriented Control (FOC) and Vector BLDC driving using external ADC and current sensors
- MCU communication via UART
- RS-485 interface to MCU (or FPGA directly, to be decided)

### Installation
Instructions for installation and running will be provided in later stages of the project.

### Hardware Requirements
- Sipeed TangNano 4K board
- Motor driver board with 6 PWM inputs and 3 power terminals
- ADC and current sensors (specifics to be specified later)
- BLDC motor

### MCU Part Setup
1. Install CMake 3.25+ and arm-none-eabi-gcc toolchain.
2. Include the armpp library (included as a submodule).

### FPGA Part Setup
1. Use Gowin EDA for synthesizing and FPGA board programming.

### Firmware Programming
- MCU Firmware: Build the firmware using CMake and program the firmware binary to the board flash using the Gowin programmer.
- FPGA Bitstream: Use the Gowin programmer to flash the bitstream.

### Contributing
- Pull requests and issues are welcome.
- Comments and feedback are appreciated.

### Coding Conventions
- C++ and SystemVerilog code follow snake_case naming convention.
- SCREAMING_SNAKE_CASE is used exclusively for macros.
- CamelCase is used only for template parameters.
- Code formatting guidelines are provided in the repository. Instructions for formatting from the command line and setting up auto-formatting in VS Code IDE will be provided.

### Known Issues
As the project is in an early stage, there may be bugs, compilation issues, or non-functional behavior. Please feel free to report any issues.

### Issue Reporting
To report issues, please submit an issue on the project's [GitHub repository](link-to-repo).

### License
This project is licensed under the [MIT License](https://opensource.org/licenses/MIT).
