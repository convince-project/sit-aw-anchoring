from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import json


class FailPlaceExecutorMockup(Node):

    def __init__(self):
        super().__init__('fail_place_executor_mockup')

        # declare parameters (no defaults)
        self.declare_parameter('who', Parameter.Type.STRING)
        self.declare_parameter('what', Parameter.Type.STRING)
        self.declare_parameter('where', Parameter.Type.STRING)

        for p in ['who', 'what', 'where']:
            if self.get_parameter(p).value is None:
                raise RuntimeError(f"Missing required parameter: {p}")

        self.srv = self.create_service(
            Trigger,
            '~/get_info',
            self.get_info_callback
        )

    def get_info_callback(self, request, response):

        who = self.get_parameter('who').value
        what = self.get_parameter('what').value
        where = self.get_parameter('where').value

        data = [{
            "dt_id": "place44",
            "place_def": {
                "param": {
                    "who": who,
                    "what": what,
                    "where": where
                },
                "result": "failure"
            }
        }]

        result = json.dumps(data)
        response.success = True
        response.message = result

        self.get_logger().info(result)

        return response


def main():
    rclpy.init()
    node = FailPlaceExecutorMockup()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
