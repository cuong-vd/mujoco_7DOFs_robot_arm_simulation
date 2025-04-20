import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

# Load model từ file XML
model = mujoco.MjModel.from_xml_path("robot_7dof.xml")
data = mujoco.MjData(model)

# Khởi tạo toàn bộ qpos = 0
data.qpos[:] = 0.0

# Mở MuJoCo viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    t = 0.0
    while viewer.is_running():
        mujoco.mj_step(model, data)
        end_effector = data.body("ee").xpos
        eex, eey, eez = end_effector

        ee_xmat_flat = data.body("ee").xmat
        ee_R = np.array(ee_xmat_flat).reshape(3, 3)

        ori = R.from_matrix(ee_R).as_euler('xyz')

        # Print actual end-effector position and orientation
        print(f"t={t:.3f} | x={eex:.3f} | y={eey:.3f} | z={eez:.3f} | "
                f"roll={ori[0]:.3f} | pitch={ori[1]:.3f} | yaw={ori[2]:.3f}")
        
        # Gán giá trị ban đầu cho từng góc (7 góc) là 0
        # angles = [0.0, -30, -30, 0.0, 30, -90, 180]  # All zeros
        angles = [0, 0, 0, 0.0, 0, 0, 0]  # All zeros
        # Convert angles from degrees to radians
        angles = np.radians(angles)

        for i in range(7):
            data.qpos[i] = angles[i]
        viewer.sync()

        t += model.opt.timestep
        time.sleep(model.opt.timestep)
