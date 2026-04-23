from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    ExecuteProcess,
    EmitEvent,
    TimerAction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.events import matches_action
from launch.event_handlers import OnProcessStart

from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

from lifecycle_msgs.msg import Transition


def generate_launch_description():

    default_params_file = PathJoinSubstitution([
        FindPackageShare('anchoring_process'),
        'launch',
        'cfg',
        'params.yaml'
    ])

    knowledge_domain_arg = DeclareLaunchArgument(
        'knowledge_domain',
        default_value='',
        description='Override knowledge domain'
    )

    instances_setup_arg = DeclareLaunchArgument(
        'instances_setup',
        default_value='',
        description='Override instances setup file'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='External YAML file overriding default params'
    )

    knowledge_domain = LaunchConfiguration('knowledge_domain')
    instances_setup = LaunchConfiguration('instances_setup')
    params_file = LaunchConfiguration('params_file')

    typedb_server = ExecuteProcess(
        cmd=[['typedb server']],
        shell=True,
        output='screen'
    )

    typedb_studio = ExecuteProcess(
        cmd=[['/opt/typedb-studio/bin/typedb-studio']],
        shell=True,
        output='screen'
    )

    anchoring_node = LifecycleNode(
        package='anchoring_process',
        executable='anchoring_process',
        name='anchoring_process',
        namespace='',
        output='screen',
        parameters=[
            default_params_file,   # base config
            params_file,           # optional override YAML
            {                      # CLI overrides (highest priority)
                'knowledge_domain': knowledge_domain,
                'instances_setup': instances_setup
            }
        ]
    )

    configure = RegisterEventHandler(
        OnProcessStart(
            target_action=typedb_server,
            on_start=[
                TimerAction(
                    period=4.0,
                    actions=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(anchoring_node),
                                transition_id=Transition.TRANSITION_CONFIGURE
                            )
                        )
                    ]
                )
            ]
        )
    )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=anchoring_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(anchoring_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    set_ontology_and_populate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=anchoring_node,
            goal_state='active',
            entities=[
                ExecuteProcess(
                    cmd=[[
                        'ros2 action send_goal /anchoring_process/set_ontology '
                        'anchoring_process_interfaces/action/SetOntology '
                        '"{knowledge_domain: \'', knowledge_domain, '\'}" && ',

                        'ros2 action send_goal /anchoring_process/populate_instances '
                        'anchoring_process_interfaces/action/PopulateInstances '
                        '"{knowledge_domain: \'', knowledge_domain,
                        '\', instances: \'', instances_setup, '\'}"'
                    ]],
                    shell=True,
                    output='screen'
                )
            ]
        )
    )

    ld = LaunchDescription()

    # arguments
    ld.add_action(knowledge_domain_arg)
    ld.add_action(instances_setup_arg)
    ld.add_action(params_file_arg)

    # processes
    ld.add_action(typedb_server)
    ld.add_action(typedb_studio)
    ld.add_action(anchoring_node)

    # lifecycle orchestration
    ld.add_action(configure)
    ld.add_action(activate)
    ld.add_action(set_ontology_and_populate)

    return ld
