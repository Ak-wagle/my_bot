import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    ''' we need to launch robot_state_publisher with use_sim_time:=true,
    and then launch gazebo, then spawn the robot '''
    
    package_name = 'my_bot'
    
    # robot_state_publisher launch file
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # Gazebo launch file, provided by the gazebo_ros package
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
    )
    
    # spawner node from the gazebo_ros pkg, the entity name can be anything as we've single robot
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity
        ])
    
        
# try executing this in the bash if the entity is still there and causing error,  while launching the script aagian in the future
# ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'my_bot'}"
# if the time is not running or gazebo is crashing then try killing the processess with below command
# sudo killall -9 gazebo gzserver gzclient