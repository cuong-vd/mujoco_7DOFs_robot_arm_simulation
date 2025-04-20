import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

def trvec2tform(translation):
    T = np.eye(4)
    T[:3, 3] = translation
    return T

def axang2tform(axis_angle):
    axis, angle = np.array(axis_angle[:3]), axis_angle[3]
    R_mat = R.from_rotvec(axis * angle).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R_mat
    return T

def forward_kinematics_dh(joint_angles):
    t1, t2, t3, t4, t5, t6, t7 = joint_angles
    
    T1 = trvec2tform([0, 0, 0.0383]) @ axang2tform([0, 0, 1, t1])
    T2 = axang2tform([1, 0, 0, np.pi/2]) @ trvec2tform([0, 0.074, -0.06]) @ axang2tform([0, 0, 1, t2])
    T3 = trvec2tform([0, 0.425, 0]) @ axang2tform([0, 0, 1, t3])
    T4 = axang2tform([1, 0, 0, -np.pi/2]) @ trvec2tform([0, -0.06, 0.074]) @ axang2tform([0, 0, 1, t4])
    T5 = axang2tform([1, 0, 0, -np.pi/2]) @ trvec2tform([0, -0.327, 0.04]) @ axang2tform([0, 0, 1, t5])
    T6 = axang2tform([1, 0, 0, np.pi/2]) @ trvec2tform([0, 0.046, 0.04]) @ axang2tform([0, 0, 1, t6])
    T7 = axang2tform([1, 0, 0, -np.pi/2]) @ trvec2tform([0, -0.046, 0.0375]) @ axang2tform([0, 0, 1, t7])
    T8 = trvec2tform([0, 0, 0.0175])
    
    T_final = T1 @ T2 @ T3 @ T4 @ T5 @ T6 @ T7 @ T8
    pos = T_final[:3, 3]
    ori = R.from_matrix(T_final[:3, :3]).as_euler('xyz')
    return np.concatenate((pos, ori))

if __name__ == "__main__":
    # Example usage
    joint_angles = np.zeros(7)  # Initial guess for joint angles
    # joint_angles = np.array([0.37,  -0.354, 0.285, -0.338,  0.37,  -0.45, 0.322])
    joint_angles = np.array([0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3])
    pose = forward_kinematics_dh(joint_angles)
    print("End-effector pose:", pose)