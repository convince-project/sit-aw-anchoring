# --------------------------------------------------------
# Copyright (c)
#
# contributions by author
#                  author@somewhere.net
# maintained by    maintainer
#                  maintainer@somewhere.net

from rclpy.action import GoalResponse
from rclpy.action import CancelResponse
from digital_twin_integrator.digital_twin_integrator_component_definition.digital_twin_integrator import digital_twin_integrator

class digital_twin_integrator_impl(digital_twin_integrator):

	def __init__(self, instName):
		super().__init__(instName)

	def get_ontology_goal(self, future):
		pass

	def get_ontology_feedback(self, handle, feedback):
		pass

	def get_ontology_result(self, result):
		pass

	def validate_set_ontology_goal(self, future):
		pass

	def validate_set_ontology_feedback(self, handle, feedback):
		pass

	def validate_set_ontology_result(self, result):
		pass

	def get_data_goal(self, future):
		pass

	def get_data_feedback(self, handle, feedback):
		pass

	def get_data_result(self, result):
		pass

	def test_graspability_goal(self, future):
		pass

	def test_graspability_feedback(self, handle, feedback):
		pass

	def test_graspability_result(self, result):
		pass

	def update_state_goal(self, goal):
		pass

	def update_state_cancel(self, goal_handle):
		pass

	def update_state_accepted(self, goal_handle):
		pass

