# Readme

This Readme file provides instructions for installing and running the gesture controlled robot arm system with Leap Motion and Robotis Manipulator-H. This project was initialy developed on Windows

## Installation

To install the necessary components, please follow the steps below:

1. Install LeapDeveloper Kit 5.0.0 from the official Leap Motion website.
2. Install Python 3.9 on your system.
3. Open a terminal or command prompt and execute the following commands:

   ```shell
   pip install numpy
   pip install dynamixel_sdk
   pip install PySocks
   ```
These commands will install the required Python packages for the project.

## Running the System

To run the gesture-controlled robot arm system, please follow the steps below:

1. Ensure that the Manipulator-H Robotis and Leap Motion devices are connected to your computer.
3. Run the ImageSample.cpp to launch the Leap Motion program.
4. If you want to view the simulation, run the `Scene-robotis-griper` program in Copellia-Sim.
6. Run the `RobotControl.py` Python script.
7. Follow the on-screen instructions to stabilize your hand upside the Leap Motion for a few seconds. This process will define the zero position of the robot arm.

## Demo Video

Click [here](https://youtu.be/AQPZxMb1I2k) to watch a demo video showcasing the gesture-controlled robot arm system in action.

## License

This project is licensed under the [MIT License](LICENSE).