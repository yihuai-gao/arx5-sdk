import time
import arx5_interface as arx5
import numpy as np

solver = arx5.Arx5Solver("../models/arx5_gopro.urdf")


joint_pos = np.array([0, 0, np.pi / 2, -np.pi / 2, 0, 0])
joint_vel = np.array([0, 0, 0, 0, 0, 0])
joint_acc = np.array([0, 0, 0, 0, 0, 0])
fk_pose = solver.forward_kinematics(joint_pos)
print(fk_pose)

print(solver.inverse_kinematics(fk_pose, np.array([0, 0, 0, 0, 0, 0])))

start_time = time.monotonic()
solver.inverse_dynamics(joint_pos, joint_vel, joint_acc)
print(f"ID: {time.monotonic() - start_time:.05f}s")
