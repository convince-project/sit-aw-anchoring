from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node

import pybullet
import json
import time
import math
import os

GRIPPER_LINK  = 11
HAND_BODY     = 8
AGENT_NAME   = "panda"
MAX_REACH     = 0.855

def get_data(client_id):
    body_count = pybullet.getNumBodies(physicsClientId=client_id)
    print(body_count)
    data = []
    letter_id = 0
    for i in range(body_count):
        bid = pybullet.getBodyUniqueId(i, physicsClientId=client_id)
        # pos, _ = pybullet_client.getBasePositionAndOrientation(bid, physicsClientId=client_id)


        # Robotic arm claws
        body_name = pybullet.getBodyInfo(bid, physicsClientId=client_id)[1].decode("utf-8")
        if AGENT_NAME in body_name.lower():
#            for i in range(pybullet.getNumJoints(bid)):
#              info = pybullet.getJointInfo(bid, i)
#              link_name = info[12].decode("utf-8")
#              print(i, link_name)
            base_pos, _ = pybullet.getBasePositionAndOrientation(bid, physicsClientId=client_id)
            grip_pos    = pybullet.getLinkState(bid, GRIPPER_LINK, physicsClientId=client_id)[0]

            aabb_min, aabb_max = pybullet.getAABB(
                bid,
                linkIndex=HAND_BODY,
                physicsClientId=client_id
            )
            size = [aabb_max[j] - aabb_min[j] for j in range(3)]

            current_dist = math.dist(base_pos, grip_pos)          # Current extended length
            remaining    = max(0.0, MAX_REACH - current_dist)

            data.append({                             
                "dt_id": "gripper",
                "gripper": {
                    "center": [round(grip_pos[0], 3),
                               round(grip_pos[1], 3),
                               round(grip_pos[2], 3)],
                    "remaining_reach": round(remaining, 3),
                },
                "bounding_box": {
                    "width":  round(size[0], 3),
                    "length": round(size[1], 3),
                    "height": round(size[2], 3),
                },
            })
            continue



        # Filter out the ground wall
        mass = pybullet.getDynamicsInfo(bid, -1)[0]
        if mass == 0:
            continue 
        # get position
        pos, _ = pybullet.getBasePositionAndOrientation(bid, physicsClientId=client_id)

        # get bounding box
        aabb_min, aabb_max = pybullet.getAABB(bid, physicsClientId=client_id)
        size = [aabb_max[j] - aabb_min[j] for j in range(3)]
        
        # Position of "pose"
        top_center = [                                  
            round((aabb_min[0] + aabb_max[0]) / 2, 3),   
            round((aabb_min[1] + aabb_max[1]) / 2, 3),   
            round(aabb_max[2], 3)
        ]

        data.append({
            "dt_id": chr(65 + letter_id),              # A, B, C…
            "position": [
                round(pos[0], 3),
                round(pos[1], 3),
                round(pos[2], 3)
            ],
            "bounding_box": {
                "width":  round(size[0], 3),
                "length": round(size[1], 3),
                "height": round(size[2], 3)
            },
            "grasp_point": {                                  
                "position": top_center
            }
        })
        letter_id += 1 

        # mockup failure of place action
        data.append({
                "dt_id": "place44",
                "place_def": {
                    "param": {
                        "who"  : "gripper",
                        "what" : "A",
                        "where": "C"
                    },
                    "result": "failure"
                }
        })

    return json.dumps(data)


class PyBulletDT(Node):

    def __init__(self):
        super().__init__('pybullet_dt')
        self.srv = self.create_service(Trigger, '~/get_data', self.get_data_callback)

    def get_data_callback(self, request, response):
        cid = pybullet.connect(pybullet.SHARED_MEMORY)
        if cid < 0:
            response.success = False
            response.message = ""
            self.get_logger().info("Can't connect to Pybullet")

        else:
            result = get_data(cid)
            response.success = True
            response.message = result
            self.get_logger().info(result)
			
        return response


def main():
    rclpy.init()

    pybullet_dt = PyBulletDT()

    rclpy.spin(pybullet_dt)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

