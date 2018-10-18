# Capstone Project

## Usage
1. Clone the project repository
	
	```
	git clone --recursive https://github.com/Clara-YR/CarND-T3P4-System_Integration 000
	```
2. Install python&catkin dependencies
	
	```
	# install pip dependencies
	cd 000
	pip install -r requirements.txt
	
	# install dbw_mkz_msgs package
	apt-get update
	apt install ros-kinetic-dbw-mkz-msgs

	# install catkin_tools package to use catkin_build
	sudo apt-get install python-catkin-tools
	```
3. Make and run styx
	
	```
	cd ros
	catkin_make -DCMAKE_BUILD_TYPE=Release
	source devel/setup.sh
	roslaunch launch/styx.launch
	```
	
4. Run the simulator

## Strucure

![](Readme_Images/code_structure.png)


- [More information about the message type of each Topic](TopicInfo.md)
- [Notes about Darknet_ros Node](DarknetInfo.md)

### Waypoint Updater
![](Readme_Images/wp_updater.png)

### DBW
![](Readme_Images/DBW.png)

### Traffict Light Detection
![](Readme_Images/tl_detection.png)

### Darknet
#### launch.tcl
catkin_make __-DCMAKE_BUILD\_TYPE=Release__

#### ros/launch/styx.launch
ADD Darknet ROS Node into frame work

```
<!--Darknet ROS Node -->
<include file="$(find darknet_ros)/launch/yolo_v3.launch"/>
```

#### ros/src/darknet_ros/darknet_ros/config/ros.yaml
Change the topic of Subscriber `camera_reading` from `/camera/rgb/image_raw` to `image_color`

#### ros/src/darknet_ros
add `LICENSE`,  `README.md`, `darknet`, `darknet_ros/CHANGELOG.rst`
'''
