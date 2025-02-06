## Cruise search and rescue robot based on Nav2 and YOLOv5
This project is the final project of the course design for intelligent robots at the School of Artificial Intelligence, South China Normal University
![image](https://github.com/user-attachments/assets/807df56d-70c1-4bd2-a791-a4aeec27d6e9)

##### **1.Related package versions**
Environment：Ubuntu22.04 + ROS2 humble + python3.10
Package                                      Version
------------------------------------ --------------------
cartographer-ros-msgs                2.0.9002

nav-2d-msgs                               1.1.17

nav-msgs                                    4.2.4

nav2-common                            1.1.17

nav2-msgs                                  1.1.17

nav2-simple-commander           1.0.0

opencv-python                           4.9.0.80

opencv-python-headless            4.10.0.84

pandas                                        2.2.3

pip                                              22.0.2

rclpy                                           3.3.14

setuptools                                  65.5.1

torch                                          2.5.1

yolov5                                       7.0.14

##### **2.Necessary installation**

cartographer mapping

```
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-nav2-map-server   # Used to save map files, only running the cruise package does not require installation
```

nav2 navigation

```
 sudo apt install ros-humble-nav2-*
```

YOLOv5 image recognition

```
sudo apt install python3-pip ros-humble-vision-msgs
pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple yolov5  
```

##### **3.Operation Guide**

Hint: To reproduce the correct results of the project, the commands for each task are independent, and there is no necessary command line execution sequence between tasks

Copy the RobotProject file from the compressed file to the~/directory, then go to colcon build in the~/RobotProject directory and refresh the environment variables

```
cd ~/RobotProject/
colcon build
source install/setup.bash
```

###### **3.1 Model presentation**

Loading robots and world models in Gazebo

```
cd src/
ros2 launch robot_description gazebo.launch.py
```

###### **3.2 mapping**

Load the robot and world model in Gazebo and start Rviz.

```
ros2 launch robot_mapping mapping.launch.py
ros2 run rviz2 rviz2
```

Add a map topic in rviz and use the keyboard to control the car to walk on the map until the complete map is created.

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Save the map file to the local directory.

```
ros2 run nav2_map_server map_saver_cli -t map -f map
```

###### **3.3 Cruise mission (no need to complete the first two steps)**

Launch navigation package.

```
ros2 launch robot_navigation navigating.launch.py
```

Launch the ros_2yolov5 package.

```
ros2 run yolo_detection yolo_detect_2d 
```

Observe the camera topic in rqt, add camera related plugins, and return the image recognition results in the result_img topic.

```
rqt
```

Finally, activate the cruise task node to start the navigation task.

```
ros2 run yolo_detection navigating 
```
