import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Define LaunchConfiguration arguments with clear descriptions
    cca_kinova_gen3_7dof_ros_setup = os.path.join(
        get_package_share_directory('cca_kinova_gen3_7dof'),
        'config',
        'cca_kinova_gen3_7dof_ros_setup.yaml'
        )

    # Create a Node instance for the cca_kinova_gen3_7dof node
    cc_affordance_planner_ros_node_with_params = Node(
        package="cca_kinova_gen3_7dof",
        executable="cca_kinova_gen3_7dof_node",
        name="cc_affordance_planner_ros",
        prefix=["xterm -e gdb -ex run --args"],
        emulate_tty=True,
        output="screen",
        parameters=[
           cca_kinova_gen3_7dof_ros_setup
        ],
        # Add node-specific parameters and additional launch actions if needed
    )
    ld.add_action(cc_affordance_planner_ros_node_with_params)
    return ld
