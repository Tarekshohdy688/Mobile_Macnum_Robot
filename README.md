# **Mobile_Macnum_Robot**
<img src="Images/Mecanumrobot.png" alt="mecanumrobot X4" width="400" align="right" caption="MecanumRobot"/>

The Macnum 4WD Mobile Robot represents a cutting-edge platform designed for versatile applications in various fields, including research, education, and industrial automation. Equipped with advanced robotics technology and controlled by ROS2 (Robot Operating System 2) and ROS1 as well, this platform offers seamless integration and robust performance. The integration of a joystick control system adds a user-friendly interface, allowing precise and intuitive manipulation of the robot's movements.

---------------------------
# Introduction

A Macnum wheel robot equipped with four omnidirectional wheels, designed to achieve seamless
omnidirectional motion. The robotic system employs advanced control mechanisms, utilizing a PID
controller for precise regulation of angular velocities. The key focus is on the calibration and testing
process, emphasizing sensor calibration, PID tuning, and kinematic model validation. The exclusion
of specific hardware components streamlines the calibration procedure, showcasing the significance
of gyroscopes and accelerometers for accurate internal state estimation. The closed-loop control
system demonstrates the robot's ability to move with some precise movement compared to the
hardware used. The successful calibration and testing efforts lay the groundwork for the robot's
reliable performance, offering promising prospects for further advancements in omnidirectional
robotic control systems.

--------------------------

# Assembly

To assemble this 4WD Macnum Mobile Robot, you need the following components:

- Raspberry Pi 3 model B+   
- Mecanum wheel x4
- Arduino Mega
- Motor driver shield
- IMU (MPU6050)
- DC motors x4
- Battery
- Power bank

Follow these steps to assemble YDPIbot:

1. Connect the Arduino Mega to the motor driver shield and the optical encoders and MPU6050 according to the pinout diagram provided in the documentation.
2. Connect the IMU to Raspberry Pi using the I2C interface and follow the instructions in the [mpu6050 pkg](https://github.com/PigeonSensei/pigeon_imu_driver/tree/master/mpu6050).
3. Mount the Raspberry Pi and the motor driver shield on the chassis of the robot using screws and standoffs.
4. Connect the DC motors to the motor driver shield and attach the wheels to the motor shafts.
5. Install the battery on the robot and connect it to the motor driver shield.
6. Install the power bank on the robot and connect it to the Raspberry Pi connected to the Arduino Mega.
