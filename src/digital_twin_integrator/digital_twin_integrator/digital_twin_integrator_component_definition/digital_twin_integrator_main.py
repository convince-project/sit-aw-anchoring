# --------------------------------------------------------
# Copyright (c)
#
# contributions by author
#                  author@somewhere.net
# maintained by    maintainer
#                  maintainer@somewhere.net

import rclpy
from digital_twin_integrator.digital_twin_integrator_component_definition.digital_twin_integrator_impl import digital_twin_integrator_impl

class digital_twin_integrator_main:

	def __init__(self):
		pass


def main():
	rclpy.init()
	digital_twin_integrator = digital_twin_integrator_impl('digital_twin_integrator_node')
	digital_twin_integrator.get_logger().info('digital_twin_integrator node has been created')
	executor = rclpy.executors.MultiThreadedExecutor()
	executor.add_node(digital_twin_integrator)
	executor.spin()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
