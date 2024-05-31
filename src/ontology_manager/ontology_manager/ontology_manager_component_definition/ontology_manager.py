# --------------------------------------------------------
# Copyright (c)
#
# contributions by author
#                  author@somewhere.net
# maintained by    maintainer
#                  maintainer@somewhere.net

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.service import Service
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.lifecycle import State
from rclpy.lifecycle import Node
from ontology_management_msgs.action import ValidateSetOntologyCS
from ontology_management_msgs.action import GetOntologyCS
from ontology_management_msgs.action import PopulateInstances
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.lifecycle import TransitionCallbackReturn
from ontology_management_msgs.srv import ExecuteCommands

class ontology_manager(Node):

  def on_configure(self, state):
    
    return TransitionCallbackReturn.SUCCESS

  def on_activate(self, state):
    
    return TransitionCallbackReturn.SUCCESS

  def on_deactivate(self, state):
    
    return TransitionCallbackReturn.SUCCESS

  def on_cleanup(self, state):
    pass

  def on_shutdown(self, state):
    
    return TransitionCallbackReturn.SUCCESS

  def __init__(self, instName):
    self.cbg_Activity01_: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
    self.cbg_Activity02_: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
    self.cbg_Activity03_: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
    self.execute_owlready2_commands_srv_: Service = None
    self.validate_set_ontology_actcli_: ActionClient = None
    self.validate_set_ontology_actsrv_: ActionServer = None
    self.get_ontology_actsrv_: ActionServer = None
    super().__init__(instName)
    self.validate_set_ontology_actsrv_ = ActionServer(
      self, ValidateSetOntologyCS, '~/validate_set_ontology',
      execute_callback = self.validate_set_ontology_accepted,
      goal_callback = self.validate_set_ontology_goal,
      cancel_callback = self.validate_set_ontology_cancel,
      callback_group = self.cbg_Activity01_)

    self.validate_set_ontology_actcli_ = ActionClient(
      self, ValidateSetOntologyCS, '~/validate_set_ontology',
      callback_group = self.cbg_Activity03_)

    self.get_ontology_actsrv_ = ActionServer(
      self, GetOntologyCS, '~/get_ontology',
      execute_callback = self.get_ontology_accepted,
      goal_callback = self.get_ontology_goal,
      cancel_callback = self.get_ontology_cancel,
      callback_group = self.cbg_Activity01_)

    self.populate_instances_actsrv_ = ActionServer(
      self, PopulateInstances, '~/populate_instances',
      execute_callback = self.populate_instances_accepted,
      goal_callback = self.populate_instances_goal,
      cancel_callback = self.populate_instances_cancel,
      callback_group = self.cbg_Activity01_)

    self.execute_owlready2_commands_srv_ = self.create_service(ExecuteCommands, "~/execute_owlready2_commands", self.execute_owlready2_commands_handler, callback_group = self.cbg_Activity02_)
    
    self.declare_parameter('overwriteOntology', True) #Leon's addition
    #self.declare_parameter('populator_setup_path', '/tmp/tests/ontology/setupfile.json') #Leon's addition
    self.declare_parameter('DT_conf_path', '/tmp/tests/DT_config.json') #Leon's addition

