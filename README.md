## 基于Nav2与YOLOv5的巡航搜救机器人 课程设计说明文档

小组于12月21日已提交项目相关文档，后于1月4日补充提交了项目的运行视频，望老师知晓
![image-20250104202717781](C:\Users\33030\AppData\Roaming\Typora\typora-user-images\image-20250104202717781.png)

运行环境：Ubuntu22.04 + ROS2 humble + python3.10



##### **1.相关包版本**

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

#####**2.必需的安装**

cartographer建图相关

```
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-nav2-map-server   # 用于保存地图文件，只运行巡航包可不安装
```

nav2导航相关

```
 sudo apt install ros-humble-nav2-*
```

YOLOv5图像识别相关

```
sudo apt install python3-pip ros-humble-vision-msgs
pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple yolov5  
```

##### **3.运行指南**

注：为复现项目的正确结果，每个任务的命令是独立的，任务之间不存在必需的命令行执行先后顺序。

将压缩包中RobotProject文件拷贝至~/目录下，在~/RobotProject目录下colcon build并刷新环境变量。

```
cd ~/RobotProject/
colcon build
source install/setup.bash
```

######**3.1 展示模型**

在gazebo中加载机器人与世界模型

```
cd src/
ros2 launch robot_description gazebo.launch.py
```

###### **3.2 建图**

在gazebo中加载机器人与世界模型，并启动Rviz。

```
ros2 launch robot_mapping mapping.launch.py
ros2 run rviz2 rviz2
```

在rviz中添加map的topic，键盘操纵小车在地图中行走直至建成地图全貌。

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

将地图文件保存至本地目录

```
ros2 run nav2_map_server map_saver_cli -t map -f map
```

###### **3.3 巡航任务（无需执行完前两步）**

启动导航包

```
ros2 launch robot_navigation navigating.launch.py
```

启动ros2_yolov5包

```
ros2 run yolo_detection yolo_detect_2d 
```

在rqt中观察摄像机话题，添加camera相关plugin，返回的图像识别结果在result_img话题中

```
rqt
```

最后启动巡航任务节点，即可开始导航任务。

```
ros2 run yolo_detection navigating 
```
