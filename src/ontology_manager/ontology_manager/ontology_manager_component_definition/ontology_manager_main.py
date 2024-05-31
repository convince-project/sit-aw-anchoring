# --------------------------------------------------------
# Copyright (c)
#
# contributions by author
#                  author@somewhere.net
# maintained by    maintainer
#                  maintainer@somewhere.net

import rclpy
from ontology_manager.ontology_manager_component_definition.ontology_manager_impl import ontology_manager_impl

class ontology_manager_main:

	def __init__(self):
		pass


def main():
	rclpy.init()
	ontology_manager = ontology_manager_impl('ontology_manager_node')
	ontology_manager.get_logger().info('ontology_manager node has been created')
	executor = rclpy.executors.MultiThreadedExecutor()
	executor.add_node(ontology_manager)
	executor.spin()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
