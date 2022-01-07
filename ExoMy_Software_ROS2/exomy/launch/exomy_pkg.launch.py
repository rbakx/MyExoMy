import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

namespace_ = 'exomy_pkg'

def generate_launch_description():
    # Get the path to the exomy.yaml parameter file
    exomy_config = os.path.join(get_package_share_directory('exomy_pkg'),'exomy.yaml')

    motors = Node(
        package='exomy_pkg',
        executable='motor_node',
        name='motor_node',
        namespace=namespace_,
        parameters=[exomy_config],
        output='screen'
    )

    return LaunchDescription([
        motors
    ])
