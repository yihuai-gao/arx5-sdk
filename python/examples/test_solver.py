import arx5_interface as arx5
import numpy as np

solver = arx5.Arx5Solver()

solver.init_solver("../models/arx5_gopro.urdf")

joint_pos = np.array([0, 0, np.pi / 2, -np.pi / 2, 0, 0])
joint_vel = np.array([0, 0, 0, 0, 0, 0])
joint_acc = np.array([0, 0, 0, 0, 0, 0])
fk_pose = solver.forward_kinematics(joint_pos)
print(fk_pose)

print(solver.inverse_kinematics(fk_pose, np.array([0, 0, 0, 0, 0, 0])))
print(solver.inverse_dynamics(joint_pos, joint_vel, joint_acc))
