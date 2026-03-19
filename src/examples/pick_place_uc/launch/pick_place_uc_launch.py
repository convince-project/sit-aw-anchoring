from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory

share_dir = get_package_share_directory('pick_place_uc')

def generate_launch_description():

  # Create and declare entities
  anchoring_process_node = LifecycleNode(
    name='anchoring_process',
    package='anchoring_process', executable='anchoring_process',
    namespace='',
    remappings=[
		],
    parameters=[share_dir+'/launch/cfg/params.yaml'],
    output='screen',
    emulate_tty=True  # assure that RCLCPP output gets flushed
  )

  # Launch Description
  ld = LaunchDescription()
  ld.add_entity(anchoring_process_node)

  return ld
