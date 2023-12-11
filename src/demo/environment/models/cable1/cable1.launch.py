import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro
import random

# This is the function launch  system will look for

def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'cable1.urdf'
    xacro_file = "cable1.xacro"
    package_description = "pkg_arscontrol_internship"
    use_urdf = False
    # Position and orientation
    # [X, Y, Z]
    position = [0.00, 0.00, 0.00]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "cable1_base"
    ####### DATA INPUT END ##########

    if use_urdf:
        # print("URDF URDF URDF URDF URDF URDF URDF URDF URDF URDF URDF ==>")
        robot_desc_path = os.path.join(get_package_share_directory(
            package_description), "environment/models/cable1", urdf_file)
    else:
        # print("XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO XACRO ==>")
        robot_desc_path = os.path.join(get_package_share_directory(
            package_description), "environment/models/cable1", xacro_file)

    robot_desc = xacro.process_file(robot_desc_path)
    xml = robot_desc.toxml()

    entity_name = robot_base_name+"-"+str(random.random())

    # Spawn cable1 Set Gazebo
    spawn_object = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/cable1_description'
                   ]
    )

    # Publish cable1 Desciption in String form in the topic /cable1_description
    publish_cable1_description = Node(
        package='pkg_arscontrol_internship',
        executable='src/demo/environment/models/cable1/cable1_description_publisher.py',
        name='cable1_description_publisher',
        output='screen',
        arguments=['-xml_string', xml,
                   '-cable1_description_topic', '/cable1_description_publisher'
                   ]
    )

    # Robot State Publisher
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'cable1_description': xml}],
        output="screen"
    )

    # create and return launch description object
    return LaunchDescription(
        [
            spawn_object,
            publish_cable1_description,
            robot_state_publisher_node
        ]
    )
