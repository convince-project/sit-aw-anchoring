import pybullet as p
import time
import json
import math

GRIPPER_LINK  = 11
AGENT_NAME   = "panda"
MAX_REACH     = 0.855

def save_to_json(client_id, file_path="runtime.json"):
    body_count = p.getNumBodies(physicsClientId=client_id)
    print(body_count)
    data = []
    letter_id = 0
    for i in range(body_count):
        bid = p.getBodyUniqueId(i, physicsClientId=client_id)
        # pos, _ = pybullet_client.getBasePositionAndOrientation(bid, physicsClientId=client_id)


        # Robotic arm claws
        body_name = p.getBodyInfo(bid, physicsClientId=client_id)[1].decode("utf-8")
        if AGENT_NAME in body_name.lower():
            base_pos, _ = p.getBasePositionAndOrientation(bid, physicsClientId=client_id)
            grip_pos    = p.getLinkState(bid, GRIPPER_LINK, physicsClientId=client_id)[0]

            current_dist = math.dist(base_pos, grip_pos)          # Current extended length
            remaining    = max(0.0, MAX_REACH - current_dist)

            data.append({                             
                "dt_id": "gripper",
                "gripper": {
                    "center": [round(grip_pos[0], 3),
                               round(grip_pos[1], 3),
                               round(grip_pos[2], 3)],
                    "remaining_reach": round(remaining, 3)
                }
            })
            continue



        # Filter out the ground wall
        mass = p.getDynamicsInfo(bid, -1)[0]
        if mass == 0:
            continue 
        # get position
        pos, _ = p.getBasePositionAndOrientation(bid, physicsClientId=client_id)

        # get bounding box
        aabb_min, aabb_max = p.getAABB(bid, physicsClientId=client_id)
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
    
    with open(file_path, "w") as f:
        json.dump(data, f, indent=4)


cid = p.connect(p.SHARED_MEMORY)
if cid < 0:
    exit()

# update every 3 seconds
while True:
    save_to_json(cid)
    time.sleep(3)
