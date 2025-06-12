import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

guiFlag = True
target_position = [1.2, 1, 2.3]
sim_time = 3
dt = 1 / 240

physicsClient = p.connect(p.GUI if guiFlag else p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(dt)

robot_id = p.loadURDF("two-link.urdf.xml", useFixedBase=True)

for j in range(p.getNumJoints(robot_id)):
    p.changeDynamics(robot_id, j, linearDamping=0, angularDamping=0)

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

joint_trajectory = [[] for _ in joint_indices]

for step in range(num_steps):
    target_pos = [interpolated_trajectories[i][step] for i in range(len(joint_indices))]
    p.setJointMotorControlArray(bodyIndex=robot_id,
                                jointIndices=joint_indices,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=target_pos)
    p.stepSimulation()

    joint_states = p.getJointStates(robot_id, joint_indices)
    for ind, state in enumerate(joint_states):
        joint_trajectory[ind].append(state[0])

    if guiFlag:
        time.sleep(dt)

eef_state = p.getLinkState(robot_id, 4)
eef_position = eef_state[4]

print("--"*20)
print("Target position:     ", target_position)
print("Final position:  ", np.round(eef_position, 4))
print("Position error: ", np.linalg.norm(np.array(target_position) - np.array(eef_position)))
print("--"*20)

time_array = np.linspace(0, sim_time, num_steps)
plt.figure(figsize=(10, 6))
for i, joint_traj in enumerate(joint_trajectory):
    plt.plot(time_array, joint_traj, label=f'Joint {joint_indices[i]}')
plt.xlabel("Time")
plt.ylabel("Joint Angle)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

time.sleep(3)