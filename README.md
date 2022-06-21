# Position Based Visual Servoing With KINOVA Robotic arm in virtual Gazebo model and real robot

This project is the implementation of Position Based Visual Servoing on the Kinova robotic arm. 

# Process: 
1. Build the ROS Kortex package for the kinova simulation
2. Insert the aruco markers in the gazebo world
3. Attach Realsense Camera on the robot end effector
4. Read the image and Convert Camera velocity into base frame of manipulator
5. Get the transformation of camera w.r.t. Base frame 
6. Get the frame orientation of camera and calculate rotation matrix wrt wrist mounted
7. Camera calibration to get intrinsic
8. Check from GUI and velocity control script for the transformation of velocity from effector frame to base frame
9. Check manual for DH upto wrist on which camera is mounted
10 We have jacobian upto end effector,one can transform velocity camera frame to end effector.

# Description:
# Building Kinova gen3_lite Gazebo simulation package: ros_kortex
ROS Kortex is the official ROS package to interact with Kortex and its related products. It is built upon the Kortex API, documentation for which can be found in the https://github.com/Kinovarobotics/kortex repository.
[https://github.com/Kinovarobotics/kortex]

Download links-
You can refer to the Kortex repository "Download links" section to download the firmware package and the release notes.
Accessing the color and depth streams
To access the color and depth streams, you will need to clone and follow the instructions to install the ros_kortex_vision repository .
[https://github.com/Kinovarobotics/ros_kortex_vision]


# Installation-
Robot Operating System (ROS) (middleware for robotics)
This package has been tested under ROS Kinetic (Ubuntu 16.04) and ROS Melodic (Ubuntu 18.04). You can find the instructions to install ROS Kinetic [http://wiki.ros.org/kinetic/Installation/Ubuntu] here and ROS Melodic here.

# Setup & Build 
These are the instructions to run in a terminal to create the workspace, clone the ros_kortex repository and install the necessary ROS dependencies:

```
sudo apt install python3 python3-pip
sudo python3 -m pip install conan
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default

mkdir -p catkin_workspace/src
cd catkin_workspace/src
git clone https://github.com/Kinovarobotics/ros_kortex.git
cd ../
rosdep install --from-paths src --ignore-src -y
catkin_make
source devel/setup.bash

```

# Launch simulation:
To launch the simulation type the following command on the terminal
``` roslaunch kortex_gazebo spawn_kortex_robot.launch start_rviz:=false arm:=gen3_lite gripper:=gen3_lite_2f```

![Launched Window](/pbvs.png)

After launching the simulation run the <PBVS_gazebo.py> file
We can see now the position of the end effector is changing and reducing the error between the camera and marker.




# control Law -
![pbvs_archi](https://user-images.githubusercontent.com/58929684/174854598-403f01b4-8426-4664-9704-03e78149e9ae.png)


![frames](https://user-images.githubusercontent.com/58929684/174852932-65321116-d6ec-41eb-bd22-e010ffd68096.png)


![controllaw](https://user-images.githubusercontent.com/58929684/174853128-be222fdb-c0d7-4f38-99b2-447d70172f9e.png)

# Results - 
This figure shows the error converges to minimum. 
![error_vec](https://user-images.githubusercontent.com/58929684/174854709-ffbccb48-398e-422f-9c9c-b8c4fa208dbd.png)
mum. 
