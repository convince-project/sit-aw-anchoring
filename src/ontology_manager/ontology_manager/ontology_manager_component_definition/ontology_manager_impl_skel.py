# --------------------------------------------------------
# Copyright (c)
#
# contributions by author
#                  author@somewhere.net
# maintained by    maintainer
#                  maintainer@somewhere.net

from rclpy.action import GoalResponse
from rclpy.action import CancelResponse
from ontology_management_msgs.srv import ExecuteCommands
from ontology_manager.ontology_manager_component_definition.ontology_manager import ontology_manager

class ontology_manager_impl(ontology_manager):

	def __init__(self, instName):
		super().__init__(instName)

	def validate_set_ontology_goal(self, goal):
		pass

	def validate_set_ontology_cancel(self, goal_handle):
		pass

	def validate_set_ontology_accepted(self, goal_handle):
		pass

	def get_ontology_goal(self, goal):
		pass

	def get_ontology_cancel(self, goal_handle):
		pass

	def get_ontology_accepted(self, goal_handle):
		pass

	def execute_owlready2_commands_handler(self, request, response):
		pass

