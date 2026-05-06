# test_sim.py

import pybullet as p
import pybullet_data
import time

#
p.connect(p.GUI_SERVER)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#
p.setGravity(0, 0, -9.8)

# --- Load plane ---
plane_id = p.loadURDF("plane.urdf")

# --- Load Franka Panda ---
robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# Reset robot to a neutral pose
for i in range(7):
    p.resetJointState(robot_id, i, targetValue=0.0)

# Open gripper
p.resetJointState(robot_id, 9, 0.04)
p.resetJointState(robot_id, 10, 0.04)

# --- Cube parameters ---
cube_size = 0.12
half_extents = [cube_size/2]*3

collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)

def create_cube(position, color):
    visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=color
    )
    return p.createMultiBody(
        baseMass=0.1,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position
    )

# --- Create stacked cubes (red bottom, green top) ---
red_cube_pos = [0.6, 0.1, cube_size/2]
green_cube_pos = [0.6, 0.1, cube_size * 1.5]

red_cube = create_cube(red_cube_pos, [1, 0, 0, 1])
green_cube = create_cube(green_cube_pos, [0, 1, 0, 1])

# --- Create blue cube near gripper ---
blue_cube_pos = [0.4, 0.0, 0.2]
blue_cube = create_cube(blue_cube_pos, [0, 0, 1, 1])

# --- Attach blue cube to gripper (fake grasp using constraint) ---
end_effector_index = 11  # Panda hand link

constraint_id = p.createConstraint(
    parentBodyUniqueId=robot_id,
    parentLinkIndex=end_effector_index,
    childBodyUniqueId=blue_cube,
    childLinkIndex=-1,
    jointType=p.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0.05],
    childFramePosition=[0, 0, 0]
)

# Target position: above green cube
target_pos = [
    green_cube_pos[0],
    green_cube_pos[1],
    green_cube_pos[2] + 4*cube_size  # hover 4 cube height above
]

# Orientation: gripper pointing down
target_orn = p.getQuaternionFromEuler([3.1416, 0, 0])

# Compute IK
joint_positions = p.calculateInverseKinematics(
    robot_id,
    end_effector_index,
    target_pos,
    target_orn
)

# Apply IK solution (first 7 joints are arm joints)
for i in range(7):
    p.resetJointState(robot_id, i, joint_positions[i])

# Limit drift effects
p.setRealTimeSimulation(0)
for j in range(p.getNumJoints(robot_id)):
    p.changeDynamics(robot_id, j, mass=0)
    
while True:
    p.stepSimulation()
    time.sleep(1. / 240.)

