import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    ))

    debug = LaunchConfiguration('debug')

    # Extract the CCA ros setup parameters for the robot
    cca_kinova_gen3_7dof_ros_setup = os.path.join(
        get_package_share_directory('cca_kinova_gen3_7dof'),
        'config',
        'cca_kinova_gen3_7dof_ros_setup.yaml'
    )

    # Run xterm in debug mode
    node_prefix = PythonExpression([
        "'xterm -e gdb -ex run --args' if '", debug, "' == 'true' else ''"
    ])

    # Emulate teletypewriter in debug mode to take user input
    emulate_tty = PythonExpression([
        "'", debug, "' == 'true'"
    ])

    # Create a Node instance for the cca_kinova_gen3_7dof node
    cc_affordance_planner_ros_node_with_params = Node(
        package="cca_kinova_gen3_7dof",
        executable="cca_kinova_gen3_7dof_node",
        name="cc_affordance_planner_ros",
        prefix=[node_prefix],
        emulate_tty=emulate_tty,
        output="screen",
        parameters=[cca_kinova_gen3_7dof_ros_setup],
    )

    ld.add_action(cc_affordance_planner_ros_node_with_params)
    return ld
