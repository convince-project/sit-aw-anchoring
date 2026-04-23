from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    simulation_script_arg = DeclareLaunchArgument(
        'simulation_script',
        default_value='',
        description='Path to the DT simulation Python script'
    )

    simulation_script = LaunchConfiguration('simulation_script')

    pybullet_dt_node = Node(
        package='pybullet_dt',
        executable='pybullet_dt',
        name='pybullet_dt',
        output='screen'
    )

    start_pybullet_simulation = ExecuteProcess(
        cmd=[
            'python3',
            simulation_script
        ],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(simulation_script_arg)
    ld.add_action(pybullet_dt_node)
    ld.add_action(start_pybullet_simulation)

    return ld
