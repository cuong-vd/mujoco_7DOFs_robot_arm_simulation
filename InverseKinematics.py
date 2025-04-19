import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from ForwardKinematics import forward_kinematics_dh
import time

def ik_loss(joint_angles, desired_pose):
    current_pose = forward_kinematics_dh(joint_angles)
    pos_error1 = np.array([np.abs(desired_pose[0] - current_pose[0])])
    pos_error2 = np.array([np.abs(desired_pose[1] - current_pose[1])])
    pos_error3 = np.array([np.abs(desired_pose[2] - current_pose[2])])
    pos_error4 = np.array([np.abs(desired_pose[3] - current_pose[3])])
    pos_error5 = np.array([np.abs(desired_pose[4] - current_pose[4])])
    pos_error6 = np.array([np.abs(desired_pose[5] - current_pose[5])])
    return np.concatenate((pos_error1, pos_error2, pos_error3, pos_error4, pos_error5, pos_error6))

def levenberg_marquardt_ik_dh(q_init, desired_pose):
    result = least_squares(
        ik_loss, 
        q_init, 
        args=(desired_pose,), 
        method='trf',  # Change to 'lm' if you want to use Levenberg-Marquardt
        ftol=1e-6,     # Tolerance for the cost function
        xtol=1e-6,     # Tolerance for the solution
        gtol=1e-6,     # Tolerance for the gradient
        max_nfev=200  # Maximum number of function evaluations
    )
    return result.x, result.cost

if __name__ == "__main__":
    # Example usage
    q_init = np.zeros(7)  # Initial guess for joint angles
    desired_pose = np.array([0.5, 0.5, 0.5, 0, 0, 0])  # Desired end-effector pose
    start_time = time.time()
    q_solution, cost = levenberg_marquardt_ik_dh(q_init, desired_pose)
    end_time = time.time()
    print("Time taken for IK:", (end_time - start_time)*1000, "ms")
    print("Optimized joint angles:", q_solution)
    print("Cost function value:", cost)