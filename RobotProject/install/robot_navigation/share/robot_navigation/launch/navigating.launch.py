
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #=============================1.定位到包的地址=============================================================
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    nav_bringup_dir = get_package_share_directory('nav2_bringup')
    
    
    #=============================2.声明参数，获取配置文件路径===================================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') 
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(robot_navigation_dir,'maps','map.yaml'))
    nav_param_path = LaunchConfiguration('params_file',default=os.path.join(robot_navigation_dir,'param','robot_nav.yaml'))
    rviz_config_dir = os.path.join(nav_bringup_dir,'rviz','nav2_default_view.rviz')

    #=============================3.声明启动launch文件，传入：地图路径、是否使用仿真时间以及nav2参数文件==============
    nav_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav_bringup_dir,'/launch','/bringup_launch.py']),
        launch_arguments={
        'map': map_yaml_path,
        'use_sim_time': use_sim_time,
        'params_file': nav_param_path}.items(),
    )
    
    rviz_node =  Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    gazebo_robot_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_description'),'launch','gazebo.launch.py'
        )])
    )
    
    return LaunchDescription([nav_bringup_launch,rviz_node,gazebo_robot_launch])
