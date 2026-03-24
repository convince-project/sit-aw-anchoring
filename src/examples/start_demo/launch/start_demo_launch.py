from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

share_dir = get_package_share_directory('start_demo')

def generate_launch_description():
	# Terminal 1 : Start TypeDB server
	typedb_server = ExecuteProcess(
		cmd=[[
			'typedb server'
		]],
		shell=True
	)

	# Terminal 3 : Start DT environment
	start_simulation = ExecuteProcess(
		cmd=[[
			'python3 /tmp/dt/DT_Simulation.py'
		]],
		shell=True
	)

	# Terminal 5 : Launch pick_place_uc
	anchoring_process_node = IncludeLaunchDescription(
		PathJoinSubstitution([
			FindPackageShare('pick_place_uc'),
			'launch',
			'pick_place_uc_launch.py'
		])
	)

	# Terminal 6 : Setup the anchoring process
	anchoring_configure_and_activate = ExecuteProcess(
		cmd=[[
			'ros2 lifecycle set /anchoring_process configure &&',
			'ros2 lifecycle set /anchoring_process activate &&',
			'ros2 action send_goal /anchoring_process/set_ontology anchoring_process_interfaces/action/SetOntology "{knowledge_domain: \'CubesWorld\'}" &&',
			'ros2 action send_goal /anchoring_process/populate_instances anchoring_process_interfaces/action/PopulateInstances "{knowledge_domain: \'CubesWorld\', instances: \'/tmp/dt/setup.json\'}"',
		]],
		shell=True
	)
#ros2 action send_goal /anchoring_process/update_state anchoring_process_interfaces/action/UpdateState "{knowledge_domain: 'CubesWorld', instances: '/tmp/dt/runtime.json'}"

	# Terminal 4 : Export DT data
	pybullet_data = Node(
		package = 'get_pybullet_data',
		executable = 'get_data'
	)

	# For now, the json still exists
	#export_json = ExecuteProcess(
	#	cmd=[[
	#		'cd /tmp/dt &&',
	#		'python3 update_json.py'
	#	]],
	#	shell=True
	#)

	# Terminal 2 : Start TypeDB studio
	typedb_studio = ExecuteProcess(
		cmd=[[
			'/opt/typedb-studio/bin/typedb-studio'
		]],
		shell=True
	)

	# Launch Description
	ld = LaunchDescription()
	ld.add_entity(typedb_server)
	ld.add_entity(start_simulation)
	ld.add_entity(anchoring_process_node)
	ld.add_entity(anchoring_configure_and_activate)
	ld.add_entity(pybullet_data)
	ld.add_entity(typedb_studio)

	return ld
