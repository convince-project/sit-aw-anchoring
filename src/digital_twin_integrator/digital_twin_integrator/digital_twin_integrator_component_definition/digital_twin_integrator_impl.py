# --------------------------------------------------------
# Copyright (c)
#
# contributions by author
#                  author@somewhere.net
# maintained by    maintainer
#                  maintainer@somewhere.net

import threading
import time

from rclpy.action import GoalResponse
from rclpy.action import CancelResponse
from action_msgs.msg import GoalStatus
from digital_twin_integrator.digital_twin_integrator_component_definition.digital_twin_integrator import digital_twin_integrator

from digital_twin_integrator.digital_twin_integrator_component_definition.func.integrator import integrator
from digital_twin_integrator.digital_twin_integrator_component_definition.func.utils import get_request_compute

from digital_twin_integrator_msgs.action import DigitalTwinIntegratorUpdateState
from digital_twin_integrator_msgs.action import GetData
from digital_twin_integrator_msgs.action import TestGraspability
from ontology_management_msgs.action import GetOntologyCS
from ontology_management_msgs.action import ValidateSetOntologyCS

class digital_twin_integrator_impl(digital_twin_integrator):

  def __init__(self, instName):
    super().__init__(instName)
    self._get_result_future = None
    self._get_true_result_future = None
    self._go_result  = GetOntologyCS.Result()
    self._gd_result  = GetData.Result()
    self._vso_result = ValidateSetOntologyCS.Result()
    self._tg_result  = TestGraspability.Result()
    self._integrator_result_success = False
    self._integrator_result_message = ""

  def get_ontology_goal(self, future):
    self.goal_callback_core_code(future, 'go', self.get_ontology_result)

  def get_ontology_feedback(self, handle, feedback):
    self.get_logger().info('Got a call to get_ontology_feedback...')

  def get_ontology_result(self, future):
    self.result_callback_core_code(future, '_go_result')

  def validate_set_ontology_goal(self, future):
    self.goal_callback_core_code(future, 'vso', self.validate_set_ontology_result)

  def validate_set_ontology_feedback(self, handle, feedback):
    self.get_logger().info('Got a call to validate_set_ontology_feedback...')

  def validate_set_ontology_result(self, future):
    self.result_callback_core_code(future, '_vso_result')

  def get_data_goal(self, future):
    self.goal_callback_core_code(future, 'gd', self.get_data_result)

  def get_data_feedback(self, handle, feedback):
    self.get_logger().info('Got a call to get_data_feedback...')

  def get_data_result(self, future):
    self.result_callback_core_code(future, '_gd_result')

  def test_graspability_goal(self, future):
    self.goal_callback_core_code(future, 'tg', self.test_graspability_result)

  def test_graspability_feedback(self, handle, feedback):
    self.get_logger().info('Got a call to test_graspability_feedback...')

  def test_graspability_result(self, future):
    self.result_callback_core_code(future, '_tg_result')

  def update_state_goal(self, goal):
    self.get_logger().info('Received update_state_goal request')
    return GoalResponse.ACCEPT

  def update_state_cancel(self, goal_handle):
    self.get_logger().info('Received update_state_cancel request')
    self._goal_handle.cancel_goal_async()
    return CancelResponse.ACCEPT

  def update_state_accepted(self, goal_handle):
    self.get_logger().info('Doing update_state_accepted...')
    self.do_work_thread = threading.Thread(target=self.update_state_work, args=(goal_handle,))
    self.do_work_thread.start()
    self.do_work_thread.join()
    self.get_logger().info('Done.')
    result = DigitalTwinIntegratorUpdateState.Result()
    result.result.success = self._integrator_result_success
    if result.result.success == True:
      goal_handle.succeed()
      result.result.message = "^^ kawaii ^^"
    else:
      goal_handle.abort()
      result.result.message = self._integrator_result_message
    return result

#################################################################################################
# Utilities #
#############

  def update_state_work(self, goal_handle):
    self.get_logger().info('Executing a update_state_goal...')

    ###
    ### - execute action client "get_ontology"
    ###
    go_goal   = GetOntologyCS.Goal()
    go_goal.knowledge_domain = goal_handle.request.knowledge_domain
    go_call_status = self.client_send_goal_and_wait_result(
            self.get_ontology_actcli_,
            goal_handle,
            self.get_ontology_feedback,
            self.get_ontology_goal,
            go_goal)
    if not self.has_send_goal_succeeded(go_call_status, self._go_result, 'get_ontology'):
      return  # if went wrong then stop

    ###
    ### - compute list of entities
    ###
    ###
    ###
    ### - execute action client "get_data"
    ###
    ### FIXME: both action calls are not implemented for now,
    ### as the DT brick is not yet ready for integration

    ###
    ### - missing action in the sequence diagram: integration of the get_data results
    ###
    ontology_path = self._go_result.ontology
    synthetic_DT_data_path = self.get_parameter('synthetic_DT_data_path').get_parameter_value().string_value
    ontology_integrated_path = integrator(ontology_path, synthetic_DT_data_path)

    ###
    ### - execute action client "validate_set_ontology"
    ###
    vso_goal = ValidateSetOntologyCS.Goal()
    vso_goal.knowledge_domain = goal_handle.request.knowledge_domain
    vso_goal.ontology = ontology_integrated_path
    vso_call_status = self.client_send_goal_and_wait_result(
            self.validate_set_ontology_actcli_,
            goal_handle,
            self.validate_set_ontology_feedback,
            self.validate_set_ontology_goal,
            vso_goal)
    if not self.has_send_goal_succeeded(vso_call_status, self._vso_result, 'validate_set_ontology'):
      return  # if went wrong then stop

    ###
    ### - execute action client "get_ontology"
    ###
    go_goal   = GetOntologyCS.Goal()
    go_goal.knowledge_domain = goal_handle.request.knowledge_domain
    go_call_status = self.client_send_goal_and_wait_result(
            self.get_ontology_actcli_,
            goal_handle,
            self.get_ontology_feedback,
            self.get_ontology_goal,
            go_goal)
    if not self.has_send_goal_succeeded(go_call_status, self._go_result, 'get_ontology'):
      return  # if went wrong then stop

    tmp = get_request_compute(self._go_result.ontology)
    cmp_req = tmp[0][0]
    robt    = tmp[0][1]
    obj     = tmp[0][2]
    self.get_logger().info('Computed \033[94m compute_request {0} \033[0m for \033[94m robot {1} \033[0m on \033[94m object {2} \033[0m'.format(cmp_req, robt, obj))

    ###
    ### - while "compute" functions ...
    ###
    ### loop starts
    ###    iteration example: "compute" function is "test_graspability"
#    tg_goal   = TestGraspability.Goal()
#    tg_goal.robot  = # must be computed from tmp above
#    tg_goal.object = # must be computed from tmp above
#    tg_call_status = self.client_send_goal_and_wait_result(
#            self.test_graspability_actcli_,
#            goal_handle,
#            self.test_graspability_feedback,
#            self.test_graspability_goal,
#            tg_goal)
#    if not self.has_send_goal_succeeded(tg_call_status, self._tg_result, 'test_graspability'):
#      return  # if went wrong then stop
#
#    vso_goal = ValidateSetOntologyCS.Goal()
#    vso_goal.knowledge_domain = goal_handle.request.knowledge_domain
#    vso_goal.ontology = # self._go_result.ontology
#    vso_call_status = self.client_send_goal_and_wait_result(
#            self.validate_set_ontology_actcli_,
#            goal_handle,
#            self.validate_set_ontology_feedback,
#            self.validate_set_ontology_goal,
#            vso_goal)
#    if not self.has_send_goal_succeeded(vso_call_status, self._vso_result, 'validate_set_ontology'):
#      return  # if went wrong then stop
    ### loop ends

    # Could finish update_state with success
    self._integrator_result_success = True

#################################################################################################
## Core functions (don't touch) ##
##################################

  def client_send_goal_and_wait_result(self, client, goal_handle, feedback_callback, done_callback, goal):
    self.get_logger().info('Executing client_send_goal_and_wait_result...')
    self._send_goal_future = client.send_goal_async(
            goal,
            feedback_callback=feedback_callback)
    self._send_goal_future.add_done_callback(done_callback)
    # synch wait
    # 1) send goal
    while self._get_result_future is None or not self._get_result_future.done:
      self.get_logger().info('not yet done...')
      if not goal_handle.is_active:
        self.get_logger().info('Goal aborted.')
        return
      if goal_handle.is_cancel_requested:
        goal_handle.canceled()
        self.get_logger().info('Goal canceled.')
        return
      time.sleep(0.5)
    self._get_result_future = None
    # 2) get the real result
    while self._get_true_result_future is None:# or not self._get_true_result_future.done:
      self.get_logger().info('not yet done...')
      if not goal_handle.is_active:
        self.get_logger().info('Goal aborted.')
        return
      if goal_handle.is_cancel_requested:
        goal_handle.canceled()
        self.get_logger().info('Goal canceled.')
        return
      time.sleep(0.5)
    status = self._get_true_result_future
    self._get_true_result_future = None
    return status

  def has_send_goal_succeeded(self, status, result, last_executed_action):
    if status != GoalStatus.STATUS_SUCCEEDED or result.result.success is not True:
        self.get_logger().info('Cannot finish update_state as {0} action has failed ({1}). Exiting...'.format(last_executed_action, result.result.message))
        return False
    return True

  def goal_callback_core_code(self, future, current_action, done_callback):
    goal_handle = future.result()
    if not goal_handle.accepted:
      self.get_logger().info('[{0}] Goal rejected :('.format(current_action))
      return
    self.get_logger().info('[{0}] Goal accepted :)'.format(current_action))
    self._goal_handle = goal_handle
    self._get_result_future = goal_handle.get_result_async()
    self._get_result_future.add_done_callback(done_callback)

  def result_callback_core_code(self, future, local_result_name):
    result = future.result().result
    status = future.result().status
    self._get_true_result_future = status
    if status == GoalStatus.STATUS_SUCCEEDED:
      self.get_logger().info('Goal succeeded! Result: {0}'.format(result.result.message))
      self._integrator_result_success = True
      setattr(self, local_result_name, result)
    else:
      self.get_logger().info('Goal failed with status: {0}'.format(status))
      self._integrator_result_success = False
      self._integrator_result_message = result.result.message

