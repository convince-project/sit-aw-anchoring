from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, EmitEvent, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch.events import matches_action

from launch.event_handlers import OnProcessStart

from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.events.lifecycle import ChangeState

from launch_ros.event_handlers import OnStateTransition

from lifecycle_msgs.msg import Transition

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
	anchoring_process_launch = IncludeLaunchDescription(
		PathJoinSubstitution([
			FindPackageShare('pick_place_uc'),
			'launch',
			'pick_place_uc_launch.py'
		])
	)

	anchoring_process_node = [x for x in anchoring_process_launch.get_sub_entities()[0].visit(1) if isinstance(x, LifecycleNode)][0]

	# Terminal 6 : Setup the anchoring process
	configure = RegisterEventHandler(
		OnProcessStart(
			target_action = typedb_server,
			on_start = [
				TimerAction(
					period = 4.0,
					actions = [
						EmitEvent(
							event=ChangeState(
								lifecycle_node_matcher=matches_action(anchoring_process_node),
								transition_id=Transition.TRANSITION_CONFIGURE
							)
						)
					],
				)
			]
		)
	)

	activate = RegisterEventHandler(
		OnStateTransition(
			target_lifecycle_node=anchoring_process_node,
			goal_state="inactive",
			entities=[
				EmitEvent(
					event=ChangeState(
						lifecycle_node_matcher=matches_action(anchoring_process_node),
						transition_id=Transition.TRANSITION_ACTIVATE
					)
				)
			],
		)
	)


	anchoring_set_ontology_and_populate = RegisterEventHandler(
		OnStateTransition(
			target_lifecycle_node=anchoring_process_node,
			goal_state="active",
			entities=[
				ExecuteProcess(
					cmd=[[
						'ros2 action send_goal /anchoring_process/set_ontology anchoring_process_interfaces/action/SetOntology "{knowledge_domain: \'CubesWorld\'}" &&',
						'ros2 action send_goal /anchoring_process/populate_instances anchoring_process_interfaces/action/PopulateInstances "{knowledge_domain: \'CubesWorld\', instances: \'/tmp/dt/setup.json\'}"',
					]],
					shell=True
				)
			],
		)
	)
	
#ros2 action send_goal /anchoring_process/update_state anchoring_process_interfaces/action/UpdateState "{knowledge_domain: 'CubesWorld', instances: '/tmp/dt/runtime.json'}"

	# Terminal 4 : Export DT data
	pybullet_data = Node(
		package = 'pybullet_dt',
		executable = 'pybullet_dt'
	)

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
	ld.add_entity(pybullet_data)
	ld.add_entity(anchoring_process_node)
	ld.add_entity(configure)
	ld.add_entity(activate)
	ld.add_entity(anchoring_set_ontology_and_populate)
	ld.add_entity(typedb_studio)

	return ld
