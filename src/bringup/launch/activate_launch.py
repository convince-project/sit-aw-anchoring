import launch_ros
import launch.actions
import lifecycle_msgs.msg

import os

from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch_ros.actions import lifecycle_node
from ament_index_python.packages import get_package_share_directory

def generate_manual_lifecycle_nodes_for_bringup_launch_description():
  ld = launch.LaunchDescription()
  nodes_map = {}
  ontology_manager_node = launch_ros.actions.LifecycleNode(
    name='ontology_manager',
    package='ontology_manager', executable='ontology_manager',
    namespace='sitaw_cluster',
    parameters=[],
    output='screen',
    emulate_tty=True  # assure that RCLCPP output gets flushed
  )
  digital_twin_integrator_node = launch_ros.actions.LifecycleNode(
    name='digital_twin_integrator',
    package='digital_twin_integrator', executable='digital_twin_integrator',
    namespace='sitaw_cluster',
    parameters=[],
    output='screen',
    emulate_tty=True  # assure that RCLCPP output gets flushed
  )
  nodes_map = {
    "ontology_manager_node": ontology_manager_node,
    "digital_twin_integrator": digital_twin_integrator_node,
  }
  ld.add_action(ontology_manager_node)
  ld.add_action(digital_twin_integrator_node)
  return ld, nodes_map

def generate_configure_transition(node):

  return launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

def generate_activate_transition(node):

  return launch.actions.RegisterEventHandler(
    launch_ros.event_handlers.OnStateTransition(
      target_lifecycle_node=node,
      start_state='configuring', goal_state='inactive',
      entities=[
        launch.actions.EmitEvent(
          event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
          )
        )
      ]
    )
  )

def generate_launch_description():

  # Generate the launch description and nodes map for bringup_launch
  ld, nodes_map = generate_manual_lifecycle_nodes_for_bringup_launch_description()

  # add transitions to configure after startup
  configure_transition_for_digital_twin_integrator_node  = generate_configure_transition(nodes_map.get("digital_twin_integrator_node"))
  configure_transition_for_ontology_manager_node         = generate_configure_transition(nodes_map.get("ontology_manager_node"))

  # add transitions to activate after configure
  activate_transition_for_digital_twin_integrator_node  = generate_activate_transition(nodes_map.get("digital_twin_integrator_node"))
  activate_transition_for_ontology_manager_node         = generate_activate_transition(nodes_map.get("ontology_manager_node"))

  # now add all obtained entities to the bringup launch description ld
  # configure transitions
  ld.add_action(configure_transition_for_digital_twin_integrator_node)
  ld.add_action(configure_transition_for_ontology_manager_node)
  # activate transitions
  ld.add_action(activate_transition_for_digital_twin_integrator_node)
  ld.add_action(activate_transition_for_ontology_manager_node)

  return ld
