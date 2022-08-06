# BucketTruckSensor

Florida Atlantic University 

FPL Bucket Truck Sensor Project 

Group 2



## Prerequisites 

[Install Virtual Box](https://www.virtualbox.org/wiki/Downloads)

[Ubuntu Desktop](https://ubuntu.com/download/desktop)

[Virtual Box Setup](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview)

[ROS 1](https://wiki.ros.org/noetic/Installation/Ubuntu)

[Download Foxglove Studio](https://foxglove.dev/download)
Install Foxglove Studio
```
sudo dpkg -I foxglove-studio-<version>-linux-amd64.deb
```

Terminator
```
sudo apt-get install terminator
```

## Setup

1. Create and build the catkin workspace
```
mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws/ 
catkin_make
```

2. Source the correct packages
```
source devel/setup.bash 
source ~/.bashrc 
```

3. Clone repo and build package
```
cd catkin_ws
git clone https://github.com/JamesRyanMacDonald/BucketTruckSensor.git src/
catkin_make
```

4. Install audio package
```
sudo apt-get install ros-noetic-audio-common
```

5. Move Alert Sound.wav file from “other” folder into the home directory of the Virtual Machine 

6. Configure VM Ethernet port to a static IP address of 192.168.1.123 
```
sudo ifconfig eth0 192.168.1.123 netmask 255.255.255.0
```

7. Connect the sensor to computer with the ethernet cable from the interface box.

8. Test the Livox sensor by running the “roslaunch” command to start publishing the LiDAR Data.
```
roslaunch livox_ros_driver livox_lidar.launch
```

9. Open Foxglove Studio, add data source, select ROS1 and click Open.

10. In terminator, split vertically twice and horizontally twice to create 4 windows.

11. In the top right terminator window, start the object detection algorithm
```
roslaunch object_detection adpt_detection.launch
```

12. In the bottom left terminator window, start the sound_play node
```
rosrun sound_play soundplay_node.py
```

13. In the bottom right terminator window, start the rostopic echo of the lidar topic
```
rostopic echo /livox/lidar
```
