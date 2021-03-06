import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

namespace_ = 'exomy_pkg'

def generate_launch_description():
    # Get the path to the exomy.yaml parameter file
    exomy_config = os.path.join(get_package_share_directory('exomy_pkg'),'exomy.yaml')

    robot = Node(
        package='exomy_pkg',
        executable='robot_node',
        name='robot_node',
        namespace=namespace_,
        parameters=[exomy_config],
        output='screen'
    )
    motors = Node(
        package='exomy_pkg',
        executable='motor_node',
        name='motor_node',
        namespace=namespace_,
        parameters=[exomy_config],
        output='screen'
    )
    gamepad = Node(
        package='exomy_pkg',
        executable='gamepad_parser_node',
        name='gamepad_parser_node',
        namespace=namespace_,
        output='screen'
    )

    return LaunchDescription([
        robot,
        motors,
        gamepad
    ])
