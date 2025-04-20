import mujoco
import mujoco.viewer
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import threading
from scipy.spatial.transform import Rotation as R

from InverseKinematics import levenberg_marquardt_ik_dh
from ForwardKinematics import forward_kinematics_dh

# Load MuJoCo model
model = mujoco.MjModel.from_xml_path("robot_7dof.xml")
data = mujoco.MjData(model)

# Create GUI sliders for desired x, y, z, roll, pitch, yaw
fig = plt.figure(figsize=(6, 8))
plt.subplots_adjust(left=0.25, bottom=0.05, top=0.95)
fig.patch.set_visible(False)
plt.axis('off')

sliders = []
slider_labels = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
slider_limits = [(-1, 1), (-1, 1), (0, 1.024), (-3.14, 3.14), (-3.14, 3.14), (-3.14, 3.14)]
num_sliders = len(slider_labels)
slider_height = 0.03
slider_spacing = 0.05
start_y = 0.85

angles_offset = [0.0, -30, -30, 0.0, 30, -90, 0]  # Offset angles
angles_offset = np.radians(angles_offset)  # Convert angles from degrees to radians
initial_values = forward_kinematics_dh(angles_offset)

for i in range(num_sliders):
    ax_slider = fig.add_axes([0.25, start_y - i * slider_spacing, 0.65, slider_height])
    slider = Slider(ax_slider, slider_labels[i], slider_limits[i][0], slider_limits[i][1], 
                    valinit=initial_values[i])
    sliders.append(slider)

# Placeholder Inverse Kinematics function
def InverseKinematics(joint_previous, x, y, z, roll, pitch, yaw):
    q_solution, cost = levenberg_marquardt_ik_dh(joint_previous, np.array([x, y, z, roll, pitch, yaw]))
    return q_solution

# Simulation thread
def run_simulation():
    joint_previous = np.array([0.0] * 7)  # Initial joint positions
    with mujoco.viewer.launch_passive(model, data) as viewer:
        t = 0.0
        while viewer.is_running():
            # if t > 10.0:
            #     print("Simulation end.")
            #     break

            ee_pos = data.body("ee").xpos
            eex, eey, eez = ee_pos
            ee_xmat_flat = data.body("ee").xmat
            ee_R = np.array(ee_xmat_flat).reshape(3, 3)
            ori = R.from_matrix(ee_R).as_euler('xyz')

            print(f"t={t:.3f} | x={eex:.3f} | y={eey:.3f} | z={eez:.3f} | "
                  f"roll={ori[0]:.3f} | pitch={ori[1]:.3f} | yaw={ori[2]:.3f}")

            # Read demanded values from sliders
            xdemand = sliders[0].val
            ydemand = sliders[1].val
            zdemand = sliders[2].val
            roll_demand = sliders[3].val
            pitch_demand = sliders[4].val
            yaw_demand = sliders[5].val

            # Compute joint positions via IK
            joint_demand = InverseKinematics(joint_previous, xdemand, ydemand, zdemand, roll_demand, pitch_demand, yaw_demand)

            # Apply computed joint positions to model
            for i in range(7):
                data.joint(f"joint{i+1}").qpos = joint_demand[i] - angles_offset[i]

            joint_previous = joint_demand

            mujoco.mj_step(model, data)
            viewer.sync()

            t += model.opt.timestep
            time.sleep(model.opt.timestep)

# Start simulation thread
sim_thread = threading.Thread(target=run_simulation)
sim_thread.start()

# Show the matplotlib sliders GUI
plt.show()
