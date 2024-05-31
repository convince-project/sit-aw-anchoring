# --------------------------------------------------------
# Copyright (c)
#
# contributions by author
#                  author@somewhere.net
# maintained by    maintainer
#                  maintainer@somewhere.net

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.lifecycle import State
from rclpy.lifecycle import Node
from ontology_management_msgs.action import GetOntologyCS
from ontology_management_msgs.action import ValidateSetOntologyCS
from digital_twin_integrator_msgs.action import GetData
from digital_twin_integrator_msgs.action import TestGraspability
from digital_twin_integrator_msgs.action import DigitalTwinIntegratorUpdateState
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.lifecycle import TransitionCallbackReturn

class digital_twin_integrator(Node):

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
    self.get_ontology_actcli_: ActionClient = None
    self.validate_set_ontology_actcli_: ActionClient = None
    self.get_data_actcli_: ActionClient = None
    self.test_graspability_actcli_: ActionClient = None
    self.update_state_actsrv_: ActionServer = None
    super().__init__(instName)
    self.get_ontology_actcli_ = ActionClient(
      self, GetOntologyCS, '/sitaw_cluster/ontology_manager/get_ontology',
      callback_group = self.cbg_Activity01_)
    
    self.validate_set_ontology_actcli_ = ActionClient(
      self, ValidateSetOntologyCS, '/sitaw_cluster/ontology_manager/validate_set_ontology',
      callback_group = self.cbg_Activity01_)
        
    self.update_state_actsrv_ = ActionServer(
      self, DigitalTwinIntegratorUpdateState, '~/update_state',
      execute_callback = self.update_state_accepted,
      goal_callback = self.update_state_goal,
      cancel_callback = self.update_state_cancel,
      callback_group = self.cbg_Activity01_)
    
    self.declare_parameter('synthetic_DT_data_path', '/tmp/tests/synthetic_DT_Data_0.json') #Leon's addition
