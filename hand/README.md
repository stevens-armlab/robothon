# Quad-axes motor drive shield with Teensy

This project provides the motor drive method utilizing Teensy and ROS serial communication. There are two parts: the Teensy code and the ROS code. 

In the Teensy code, the motor communication takes care of the current and desired position. ROS nodes are estabilished on both Teensy and linux.

## Setting up Repositories
To download all repos needed, first, we must download the version control software tool vcstool:

    sudo apt install python3-vcstool

Then, download the hand_ur.repos file (raw file) to your catkin_ws folder, and run the following commands in the catkin_ws folder:
    
    vcs import --input hand_ur.repos
    rosdep install --from-paths src --ignore-src -y
    export CPLUS_INCLUDE_PATH="$CPLUS_INCLUDE_PATH:/usr/include/python2.7/"
    catkin_make


## Install & Software Environment
* Install ROS (Kinetic tested), see [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Install Arduino, see [Arduino Software IDE on linux](https://www.arduino.cc/en/Guide/Linux)
* Install Teensyduino, see [Teensyduino plugin](https://www.pjrc.com/teensy/td_download.html)  
Note 0: always check on Teensyduino for the supported version of Arduino  
Note 1: you need to run the following before you can run the downloaded file  
`chmod +x TeensyduinoInstall.linux64`  
Note 2: when asked to select Arduino folder, it is where you extracted the Arduino, which is likely to be under your Download  
Note 3: you need to also download the rules to run Teensy, see [https://www.pjrc.com/teensy/49-teensy.rules](https://www.pjrc.com/teensy/49-teensy.rules).  
    * Right click link and select "Save Link As".  
    * Run in terminal (must be in folder where you saved the rules): `sudo cp 49-teensy.rules /etc/udev/rules.d/`  
* Verify & Upload code to Teensy   
Tool -> Board -> Teensy 4.0  
Tool -> Port -> `/dev/ttyACM0` or `/dev/ttyS4`  
Verify -> Upload

## A quick motor spin test
 * Download the Dynamixel official repo at [here](https://github.com/ROBOTIS-GIT/Dynamixel2Arduino) and git clone it under Arduino/libraries  
   `cd ~/Arduino/libraries/`  
    `git clone git@github.com:ROBOTIS-GIT/Dynamixel2Arduino.git`
 * Connect a motor for testing on J1, and run the default example `scan_dynamixel`
 * Write down all the information about this motor including: `protocol`, `id`, `baud rate`, and `model number`. 
 * [optional] Use `set_baudrate` to change the baud rate of the motor
 * [optional] Use `set_id` to change the id of the motor
 * Replace all of the information in the position_control test code, and run it

## Setup development environment for ROS messages (Teensy 4.0)
* Create a catkin workspace (see ROS beginner tutorials)
* Clone the quad-motor-teensy repository into the src folder of this workspace:  
`cd <ws>/src`  
`git clone https://<username>@bitbucket.org/stevens-robotics/quad-motor-teensy.git`  
* Install rosserial_arduino. NOTE that the default installation method using command line `sudo apt-get` would fail Teensy 4.0, and the issue is solved by shakeebbb in [issue 456](https://github.com/ros-drivers/rosserial/pull/456)
* Following this thread, we have to clone shakeebbb's teensy_4_0_support branch into the same catkin workspace:  
`cd <ws>/src`  
`git clone https://github.com/shakeebbb/rosserial.git`  
`cd rosserial`  
`git checkout teensy_4_0_support`  
`cd <ws>`  
`catkin_make`  
`catkin_make install`  
* Make sure that the modified rosserial package reside in your workspace/src folder
* see [rosserial Arduino IDE setup](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
* Set up Arduino ros_lib with the custom ros messages generated  
`cd <sketchbook>/libraries`  
`rm -rf ros_lib`  
`source ~/catkin_ws/devel/setup.bash`  
`rosrun rosserial_arduino make_libraries.py .`  
you may double check by `ls ros_lib | grep teensy_quad`, you should see `teensy_quad_motor_msgs`
Run Rosserial test following [rosserial_python](http://wiki.ros.org/rosserial_python):  
`roscore`  
`source devel/setup.bash`  
`rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200`  

## Known Issues  
* If you cannot use serial monitor in your Arduino IDE, and getting an error of relating to permissions for serial port, try the folloinwg command:  
`sudo chmod a+rw /dev/ttyACM0`  
 
## Running Instructions
 
### Demo 1 - Display Motor Position Readings in ROS  
* Upload Arduino code "motor_communication_test_ros.ino" to test communications. Motor 1 should spin in a sinusoidal pattern.  
* In a blank terminal, run the following (roscore is a prerequisite for any communication between ROS nodes):  
`roscore`  
* Open a new terminal and run the following to begin serial communications and create /motor_positions as a ROS topic:  
`source <ws>/devel/setup.bash`  
`rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200`
* Open a new terminal and run the following. You should see /motor_positions as an available topic.  
`source <ws>/devel/setup.bash`  
`rostopic list`  
`rostopic echo /motor_positions`  
* This should display the position, velocity, torque, and mode of all four motors, although only motor 1 will be used in this test.  
* **Debugging:**  
    * Make sure you run `source <ws>/devel/setup.bash` on EVERY terminal you create.  
    * If `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200` yields an error:  
    `[ERROR] [1587668348.997823]: Error opening serial: [Errno 2] could not open port /dev/ttyACM0: [Errno 2] No such file or directory: '/dev/ttyACM0'`  
        * Make sure Teensy is plugged in an running the sinusoidal command from the Arduino script.
        * Try reconnecting Teensy board, sometimes the port can change from ttyACM0.  

### Demo 2 - Give continous motor position commands as a ros publisher  
* Step 1: Open the "motor_goto_test_ros.ino" with the Arduino IDE and upload it to teensy. This code will publish to the "motor_positions" topic, and subscribe the "motor_goto_cmd" topic which will be used to command motor position.
* Step 2: In the first terminal, source the ws, and run the following:
`roscore`
* Step 3: In the second terminal, source the ws, and using the following command to run rosserial, please change the the port address and the baudrate cooresponding to your setup:
`rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200`
* Step 4: In the third terminal, source the ws, using the following command to run "motor_command_test" node to publish motor position commands:
`rosrun teensy_quad_motor_msgs motor_communication_test.py`
* Step 5: Up to this point, you have everything started, motor should start spinning slowly. Now, you can use `rostopic list` to check available topics and use `rostopic echo /topic_name` to get more details.


### Demo 3 - Keyboard control motor positions (1 motor for now)
* Step 1: Open the "keyboard_control_ros.ino" with the Arduino IDE and upload it to teensy. This code will publish to the "motor_positions" topic, and subscribe the "motor_goto_cmd" topic which will be used to command motor position. Currently only connect one motor.
* Step 2: Git clone the "ros_teleop/teleop_tools" package for taking keyboard inputs
`git clone https://github.com/ros-teleop/teleop_tools.git` to the catkin_ws/src folder, parallel with "quad-motor-teensy", and then, run `catkin_make` and source the ws
* Step 3: In the first terminal, source the ws, and run the following:
`roscore`
* Step 4: In the second terminal, source the ws, and using the following command to run rosserial, please change the the port address and the baudrate cooresponding to your setup:
`rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200`
* Step 5: In the third terminal, source the ws, and using the following command to start the "keyboard_control.py script", which subscribe to "/motor_positions" and "/key_vel", and publish to "/motor_goto_cmd":
`rosrun teensy_quad_motor_msgs keyboard_control.py`
* Step 6: In the fourth terminal, source the ws, and using the following command to start teleop_tools, and using the arrow keys to control motors:
`rosrun key_teleop key_teleop.py`
* Step 7: Up to this point, you have everything started. Echo "/motor_positions" and "/motor_goto_cmd" to view current motor status and cmd motor status while using arrow keys to control motor. 
