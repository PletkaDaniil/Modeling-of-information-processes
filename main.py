import pybullet as p
import time
import pybullet_data
import numpy as np

guiFlag = True
target_position = [0.3, 0.5, 2.4]
sim_time = 3
dt = 1 / 240

physicsClient = p.connect(p.GUI if guiFlag else p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(dt)

robot_id = p.loadURDF("two-link.urdf.xml", useFixedBase=True)

for j in range(p.getNumJoints(robot_id)):
    p.changeDynamics(robot_id, j, linearDamping=0, angularDamping=0)

num_joints = p.getNumJoints(robot_id)
joint_indices = [0, 1, 3]
initial_joint_positions = [0.0, 0.0, 0.0]

p.setJointMotorControlArray(bodyIndex=robot_id,
                            jointIndices=joint_indices,
                            controlMode=p.POSITION_CONTROL,
                            targetPositions=initial_joint_positions)

for _ in range(240):
    p.stepSimulation()
    if guiFlag:
        time.sleep(dt)

ik_solution = p.calculateInverseKinematics(robot_id, 4, target_position)
num_steps = int(sim_time / dt)
interpolated_trajectories = [
    np.linspace(initial_joint_positions[i], ik_solution[i], num_steps)
    for i in range(len(joint_indices))
]

for step in range(num_steps):
    target_pos = [interpolated_trajectories[i][step] for i in range(len(joint_indices))]
    p.setJointMotorControlArray(bodyIndex=robot_id,
                                jointIndices=joint_indices,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=target_pos)
    p.stepSimulation()
    if guiFlag:
        time.sleep(dt)

time.sleep(1)
# if guiFlag:
#     while True:
#         p.stepSimulation()
#         time.sleep(dt)
