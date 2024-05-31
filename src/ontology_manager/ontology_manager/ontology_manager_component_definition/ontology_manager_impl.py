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
from ontology_management_msgs.action import ValidateSetOntologyCS
from ontology_management_msgs.action import GetOntologyCS
from ontology_management_msgs.action import PopulateInstances
from ontology_management_msgs.srv import ExecuteCommands
from ontology_manager.ontology_manager_component_definition.ontology_manager import ontology_manager

from ontology_manager.ontology_manager_component_definition.func.utils import create_tmp_file
from ontology_manager.ontology_manager_component_definition.func.populator import populator
from ontology_manager.ontology_manager_component_definition.func.validate_set_ontology import validate_set_ontology

ontology_directory = '/tmp/tests/ontology/'
# ontology_list = [] # [{'knowledge_domain': '', 'path': ''}

class ontology_manager_impl(ontology_manager):

  def __init__(self, instName):
    super().__init__(instName)
    self.ontology_list = [] # [{'knowledge_domain': '', 'path': ''}
    # attributes used by Populate Instances
    self._get_result_future = None
    self._get_true_result_future = None
    self._vso_result = ValidateSetOntologyCS.Result()
    self._pi_result_success = False
    self._pi_result_message = ""

  # --- Validate and set ontology ---

  # Server part

  def validate_set_ontology_goal(self, goal):
    self.get_logger().info('Received validate_set_ontology_goal request')
    return GoalResponse.ACCEPT

  def validate_set_ontology_cancel(self, goal_handle):
    self.get_logger().info('Received validate_set_ontology_cancel request')
    return CancelResponse.ACCEPT

  def validate_set_ontology_accepted(self, goal_handle):
    self.get_logger().info('Executing a validate_set_ontology_goal. Doing computations...')
    
    # ---
    # Load request
    knowledge_domain, ontology_path = goal_handle.request.knowledge_domain, goal_handle.request.ontology

    print(f'knowledge_domain: {knowledge_domain}, ontology_path: {ontology_path}')
    self.ontology_list, message = validate_set_ontology(self, goal_handle, ontology_path, knowledge_domain)
    
    # ---
    self.get_logger().info('Done.')
    goal_handle.succeed()
    result = ValidateSetOntologyCS.Result()
    result.result.success = True
    result.result.message = message
    return result

  # Client part
  def validate_set_ontology_send_goal(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
      self.get_logger().info('[vso] Goal rejected :(')
      return
    self.get_logger().info('[vso] Goal accepted :)')
    self._goal_handle = goal_handle
    self._get_result_future = goal_handle.get_result_async()
    self._get_result_future.add_done_callback(self.validate_set_ontology_result)

  def validate_set_ontology_feedback(self, handle, feedback):
    self.get_logger().info('Got a call to validate_set_ontology_feedback...')

  def validate_set_ontology_result(self, future):
    result = future.result().result
    status = future.result().status
    self._get_true_result_future = status
    if status == GoalStatus.STATUS_SUCCEEDED:
      self.get_logger().info('Goal succeeded! Result: {0}'.format(result.result.message))
      self._pi_result_success = True
      self._vso_result = result
    else:
      self.get_logger().info('Goal failed with status: {0}'.format(status))
      self._pi_result_success = False
      self._pi_result_message = result.result.message

  # --- Get ontology ---

  def get_ontology_goal(self, goal):
    self.get_logger().info('Received get_ontology_goal request')
    return GoalResponse.ACCEPT

  def get_ontology_cancel(self, goal_handle):
    self.get_logger().info('Received get_ontology_cancel request')
    return CancelResponse.ACCEPT

  def get_ontology_accepted(self, goal_handle):
    self.get_logger().info('Executing a get_ontology_goal. Doing computations...')
    # ---

    # Load request
    knowledge_domain = goal_handle.request.knowledge_domain
    # get path of the ontology
    ontology_path = ''
    for ontology_dictionnary in self.ontology_list:
      if knowledge_domain in ontology_dictionnary['knowledge_domain']:
        ontology_path = ontology_dictionnary['path']
    
    # if ontology_path exists,
    if ontology_path!= '':
      new_ontology_path = create_tmp_file(self, ontology_path)
      self.get_logger().info('Done.')
      goal_handle.succeed()
      result = GetOntologyCS.Result()
      result.ontology = new_ontology_path
      result.result.success = True
      result.result.message = '^^ Done get_ontology ^^'
      return result
    
    # if ontology_path doesn't exist,
    else:
      self.get_logger().info(f'Error: Ontology : {knowledge_domain} not found')
      goal_handle.abort()
      result = GetOntologyCS.Result()
      result.result.success = False
      result.result.message = f'^^ Error: Ontology : {knowledge_domain} not found ^^'
      return result
   
    

  # --- Execute owlready2 commands ---

  def execute_owlready2_commands_handler(self, request, response):
    self.get_logger().info('Get a request to execute_owlready2_commands. Doing computations...')
    # ---
    exec(request.commands)
    
    # ---
    response.output = '^^ Done execute_owlready2_commands ^^'


  # --- Populate instances ---
  def populate_instances_goal(self, goal):
    self.get_logger().info('Received populate_instances_goal request')
    return GoalResponse.ACCEPT
  
  def populate_instances_cancel(self, goal_handle):
    self.get_logger().info('Received populate_instances_cancel request')
    return CancelResponse.ACCEPT
  
  def populate_instances_accepted(self, goal_handle):
    self.get_logger().info('Doing populate_instances_accepted...')
    self.do_work_thread = threading.Thread(target=self.populate_instances_work, args=(goal_handle,))
    self.do_work_thread.start()
    self.do_work_thread.join()
    self.get_logger().info('Done.')
    result = PopulateInstances.Result()
    result.result.success = self._pi_result_success
    if result.result.success == True:
      goal_handle.succeed()
      result.result.message = "^^ hurray ^^"
    else:
      goal_handle.abort()
      result.result.message = self._pi_result_message
    return result

  def populate_instances_work(self, goal_handle):
    self.get_logger().info('Executing a populate_instances_goal...')

    ###
    ### - create instances
    ###
    knowledge_domain = goal_handle.request.knowledge_domain
    ontology_path = self.ontology_list['knowledge_domain' == knowledge_domain]['path']
    setup_path = goal_handle.request.instances
    DT_conf_path = self.get_parameter('DT_conf_path').get_parameter_value().string_value
    ontology_populated_path = populator(ontology_path, setup_path, DT_conf_path)

    ###
    ### - execute action client "validate_set_ontology"
    ###
    vso_goal = ValidateSetOntologyCS.Goal()
    vso_goal.knowledge_domain = goal_handle.request.knowledge_domain
    vso_goal.ontology = ontology_populated_path
    vso_call_status = self.client_send_goal_and_wait_result(
            self.validate_set_ontology_actcli_,
            goal_handle,
            self.validate_set_ontology_feedback,
            self.validate_set_ontology_send_goal,
            vso_goal)
    if not self.has_send_goal_succeeded(vso_call_status, self._vso_result, 'validate_set_ontology'):
      return  # if went wrong then stop

    # Could finish update_state with success
    self._pi_result_success = True

#################################################################################################
## Core class functions (don't touch) ##
########################################

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

