# test_sim.py

import pybullet as p
import pybullet_data
import time
# import json
# import update_json as up

#
p.connect(p.GUI_SERVER)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#
p.setGravity(0, 0, -9.8)

p.loadURDF("plane.urdf")

panda = p.loadURDF("franka_panda/panda.urdf", [-0.75, 0, 0],useFixedBase=True)

box_size = [0.2, 0.2, 0.2]
base_position = [0, 0, box_size[2] / 2]

#
colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]  # red green blue

#
# box_ids = []

#
# for i in range(3):
#     box_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s / 2 for s in box_size])
#     visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[s / 2 for s in box_size], rgbaColor=colors[i])
#     box = p.createMultiBody(baseMass=1,
#                             baseCollisionShapeIndex=box_id,
#                             baseVisualShapeIndex=visual_id,
#                             basePosition=[base_position[0], base_position[1], base_position[2] + i * box_size[2]])
#     box_ids.append(box)

# Create marker
# marker_vis = p.createVisualShape(p.GEOM_SPHERE,
#                                  radius=0.01,
#                                  rgbaColor=[1,0,0,1])

for i in range(3):
    box_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s / 2 for s in box_size])
    visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[s / 2 for s in box_size], rgbaColor=colors[i])
    # z_box = base_position[2] + i * box_size[2]
    p.createMultiBody(baseMass=1,
                      baseCollisionShapeIndex=box_id,
                      baseVisualShapeIndex=visual_id,
                      basePosition=[base_position[0], base_position[1], base_position[2] + i * box_size[2]])
    # box_ids.append(box)

    # Attach a marker for this box (Easy to get position of 'pose' after)
    # Create a marker
    # mid = p.createMultiBody(baseMass=0,
    #                         baseCollisionShapeIndex=-1,     # no collision
    #                         baseVisualShapeIndex=marker_vis,
    #                         basePosition=[base_position[0],base_position[1],z_box + box_size[2]/2])           

    # Attach the marker
    # p.createConstraint(parentBodyUniqueId=bid, parentLinkIndex=-1,
    #                    childBodyUniqueId=mid, childLinkIndex=-1,
    #                    jointType=p.JOINT_FIXED, jointAxis=[0,0,0],
    #                    parentFramePosition=[0, 0, box_size[2]/2],  # center of top face
    #                    childFramePosition=[0, 0, 0])


# last_call_time = time.time()

while True:
    p.stepSimulation()
    time.sleep(1. / 240.)

    # if time.time() - last_call_time >= 5:
        # up.save_to_json(box_ids, p, "box_positions.json")
        # last_call_time = time.time()


# p.disconnect()
