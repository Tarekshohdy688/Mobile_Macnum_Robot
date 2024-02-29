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

---------------------------
# Installation

To use this mobile platform, you will need to install the following dependencies:

- [Python 3](https://www.python.org/downloads/)
- [Ubiquity Robotics image - Ubuntu 20.04](https://learn.ubiquityrobotics.com/noetic_pi_image_downloads) - for rasberry pi
- [ROS Noetic](http://wiki.ros.org/noetic/Installation) - the ubiquity robotics image have ROS noetic already
- [ROS Foxy](https://docs.ros.org/en/foxy/Installation.html) - On your Remote Device (Laptop for example).
- [MPU6050](https://github.com/PigeonSensei/pigeon_imu_driver/tree/master/mpu6050)
- [Joystick](https://www.youtube.com/watch?v=F5XlNiCKbrY) - use this youtube tutorial for controlling the robot with joystick
- Rosserial

---------------------------
### Installing Ubuntu20.04 on Raspberry PI

We recommend installing Ubuntu 20.04 and ROS noetic on the Raspberry Pi using the Ubiquity Robotics image. Follow these steps to install Ubuntu 20.04 using the Ubiquity Robotics image:

1. Download the Ubiquity Robotics image for Raspberry Pi from the [Ubiquity Robotics](https://learn.ubiquityrobotics.comnoetic_pi_image_downloads)

2. Flash the image to an SD card using a tool like Raspberry Pi Imager.

3. Insert the SD card into the Raspberry Pi and power it on.

4. Follow the on-screen instructions to complete the Ubuntu 20.04 installation process.

---------------------------
### Installing MPU6050 pkg

To install the MPU6050 pkg, follow these steps:

1. Open a terminal on your Raspberry Pi.

2. Clone the MPU6050 repository using the following command:

    `https://github.com/PigeonSensei/pigeon_imu_driver/tree/master/mpu6050`

3. Install wiringPi library from [here](http://wiringpi.com/download-and-install/)

4. Run some demo with the IMU to check if it's working.

--------------------------
### Installing Mecanum Mobile Robot pkg

To install the Mecanum Mobile Robot package, follow these steps:

1. Open a terminal on your PC.

2. Navigate to your `catkin_ws/src` directory

3. Clone the Mecanum Robot ROS1 Package using the following command:

    `git clone https://github.com/Tarekshohdy688/Mobile_Macnum_Robot/tree/main/ROS1%20Package.git`

4. Navigate back to the main catkin_ws to build the pkg using the command:

    `catkin_make`

5. Navigate to your `colcon_ws/src` directory

6. Clone the Mecanum Robot ROS2 Package using the following command:

    `git clone https://github.com/Tarekshohdy688/Mobile_Macnum_Robot/tree/main/ROS2%20Package.git`

7. Navigate back to the main catkin_ws to build the pkg using the command:

    `colcon build`

-----------------------------
### Uploading Arduino Code

1. Open Arduino IDE and connect the arduino to your PC

2. Choose from tools/boards Arduino Mega 2560 and choose the port (Usually **/dev/ttyACM0** if you are running from Ubuntu)

3. Open the code in `~/catkin_ws/src/ROS1Package/Motors_code/Motors_code.ino`

4. Upload the code and connect the Arduino to the raspberry pi.


-----------------------------
## Work Process

1) ROS1-2 Bridge:

-cancel the default sourcing from bashrc
```
sudo apt-get install ros-foxy-ros1*

source /opt/ros/noetic/setup.bash

source /opt/ros/foxy/local_setup.bash 

source /home/tarekshohdy/catkin_workspace/devel/setup.bash

source /home/tarekshohdy/colcon_ws/install/local_setup.bash

export ROS_MASTER_URI=http://tarekshohdy-IdeaPad-Gaming-3-15ACH6:11311/

ros2 run ros1_bridge dynamic_bridge
```
Then try with:
Testing code ros1: 
```
rosrun rospy_tutorials listener
```
Testing code ros2:
```
ros2 run demo_nodes_cpp talker
```

2) SSH into Pi:

Username: *****

Password: *****

Connect both laptop and pi on the same network

- On pi:
```
hostname -I
```
```
sudo raspi-config
```
>> 3 Interface Options >> P2 SSH >> yes >> Ok

- On laptop:
```
ssh ubuntu@192.168.43.237
```
password: *****


3) joystick control:

Follow the youtube tutorial mentioned earlier in the Installation section.
```
sudo apt install joystick*

evtest
```
```
sudo apt install jstest-gtk

jstest-gtk
```
```
ros2 run joy joy_enumerate_devices 
```
```
ros2 run joy joy_node 
```
```
ros2 topic echo /joy
```
```
ros2 param list
```
```
ros2 launch articubot_one joystick.launch.py
```
```
ros2 topic echo /joy
```
```
ros2 topic echo /cmd_vel
```
Check Point 1: 


4) whole system so far:

-bashrc pi:

- export ROS_MASTER_URI=http://192.168.43.177:11311/

- export ROS_IP=192.168.43.237

-bashrc lap:

- export ROS_MASTER_URI=http://192.168.43.177:11311/

- export ROS_IP=192.168.43.177

-Noetic terminal:

- roscore


-foxy terminal:

- ros2 run ros1_bridge dynamic_bridge

-foxy terminal:

- ros2 launch articubot_one joystick.launch.py

or 

- ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
  
  x: 0.0
  
  y: 0.0
  
  z: 0.0
  
angular:

  x: 0.0
  
  y: 0.0
  
  z: 0.0" 

-noetic terminal:

- ros2 run my_first_package mecanum

-noetic terminal:

- rostopic echo /wheel_vel

-ssh terminal:

- rosrun rosserial_python serial_node.py /dev/ttyACM0 57600

5) Arduino Code cpp:
<img src="Images/Arduino_Code_1.jpeg" alt="Arduino_Code_1 X4" width="600" align="center" caption="Arduino_Code_1"/>
<img src="Images/Arduino_Code_2.jpeg" alt="Arduino_Code_2 X4" width="600" align="center" caption="Arduino_Code_2"/>
<img src="Images/Arduino_Code_3.jpeg" alt="Arduino_Code_3 X4" width="600" align="center" caption="Arduino_Code_3"/>

6) Kinematic_node

7) IMU:
   
-ros2 run basic_pkg quat_to_eul

-ros2 topic echo /euler_angles

```
ros2 topic pub /imu_data sensor_msgs/msg/Imu "header:

  stamp:

    sec: 0

    nanosec: 0

  frame_id: 'base_link'

orientation:

  x: 1.0
  
  y: 0.0
  
  z: 2.0
  
  w: 1.0
  
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

angular_velocity:

  x: 0.0
  
  y: 0.0
  
  z: 0.0
  
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

linear_acceleration:

  x: 0.0
  
  y: 0.0
  
  z: 0.0

linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

-On raspberry pi:

- roslaunch mpu6050 mpu6059.launch

8) PID Control:

-ros2 run basic_pkg pid_params

-ros2 param list

-ros2 param set pid_params kp 0.9

-ros2 run basic_pkg pid_params --ros-args --params-file /home/youmna/colcon_ws/src/basic_pkg/config/pid_params.yaml

10) Speed Conversion:

-ros2 run basic_pkg mecanum_bot

-ros2 topic pub /diff_cont/cmd_vel_unstamped --rate 1 geometry_msgs/msg/Twist "linear:

  x: 0.0
  
  y: 0.0
  
  z: 0.0
  
angular:

  x: 0.0
  
  y: 0.0" 
  
  z: 2.0"
  
Check Point 3:

10) Gazebo:
    
-Include the IMU Plugin by running the following command:

<img src="Images/Gazebo_Visual.jpeg" alt="Gazebo_Visual X4" width="500" align="center" caption="Gazebo_Visual"/>

11) RVIZ:
    
<img src="Images/RVIZ_Visual.jpeg" alt="RVIZ_Visual X4" width="500" align="center" caption="RVIZ_Visual"/>


Finally just Run the following commands:
