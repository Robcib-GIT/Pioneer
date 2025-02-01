# PlatformIO Projects

This directory contains various PlatformIO projects related to the Pioneer Robot Project. These projects have been instrumental in gradually developing and refining the code, as well as for debugging purposes. Below is a brief description of each project and some useful links on how to work with PlatformIO.

## Projects
- [**encoderCount**](projects/encoderCount): For calculating the encoder count of a DC motor.
- [**encoderPID**](projects/encoderPID): Velocity control of two DC motors using a PID class.
- [**microROS**](projects/microROS): cmd_vel subscriber.
- [**motorTest**](projects/motorTest): Tests 2-motor movement.

## Working with PlatformIO
PlatformIO is a powerful development environment for microcontrollers. Here are some resources to help you get started:

- [PlatformIO Quick Start Guide](https://docs.platformio.org/en/latest/core/quickstart.html): A comprehensive guide to getting started with PlatformIO.
- [PlatformIO Tutorials and Examples](https://docs.platformio.org/en/latest/tutorials/index.html): Official tutorials and examples for various boards and frameworks.
- [Getting Started with PlatformIO (Video)](https://www.youtube.com/watch?v=JmvMvIphMnY): A detailed video tutorial on how to use PlatformIO with Visual Studio Code.

## How to Use These Projects
1. **Clone the Repository**:
    ```bash
    git clone https://github.com/jmkaz16/Pioneer.git
    cd Pioneer/projects
    ```

2. **Open the Project in PlatformIO**:
    - Open Visual Studio Code.
    - Install the PlatformIO extension if you haven't already.
    - Open the [`projects`](projects) folder in Visual Studio Code.

3. **Build and Upload**:
    - Select the desired project from the PlatformIO project explorer.
    - Build the project using the PlatformIO build button.
    - Upload the firmware to your microcontroller using the PlatformIO upload button.

For more detailed instructions, refer to the PlatformIO documentation and tutorials linked above.
