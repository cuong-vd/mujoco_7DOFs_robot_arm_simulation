# 7-DOF Robot Inverse Kinematics Simulation

This project simulates a 7-DOF robotic arm using MuJoCo and provides a GUI to control the desired end-effector position and orientation using sliders. The inverse kinematics (IK) is solved using the Levenberg-Marquardt algorithm.

## Prerequisites

- Python 3.8 or higher
- MuJoCo installed and configured
- A valid MuJoCo license key (`mjkey.txt`) in your MuJoCo installation directory

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/your-username/7dof-robot-ik.git
cd 7dof-robot-ik
```

### 2. Create a Virtual Environment

It is recommended to use a virtual environment to manage dependencies.

```bash
python3 -m venv venv
source venv/bin/activate  # On Windows, use `venv\Scripts\activate`
```

### 3. Install Required Libraries

Install the required Python libraries using `pip`:

```bash
pip install -r requirements.txt
```

If the `requirements.txt` file is not available, you can manually install the dependencies:

```bash
pip install mujoco matplotlib numpy scipy
```

### 4. Verify MuJoCo Installation

Ensure that MuJoCo is installed and configured correctly. You can follow the official [MuJoCo installation guide](https://mujoco.org/book/install.html).

## Usage

1. Run the `verify_IK.py` script to start the simulation:

   ```bash
   python verify_IK.py
   ```

2. Use the sliders in the GUI to control the desired end-effector position (`x`, `y`, `z`) and orientation (`roll`, `pitch`, `yaw`).

3. The simulation will compute the joint angles using inverse kinematics and update the robot's configuration in real-time.

## File Structure

- `ForwardKinematics.py`: Implements forward kinematics for the 7-DOF robot.
- `InverseKinematics.py`: Implements the inverse kinematics solver using the Levenberg-Marquardt algorithm.
- `verify_IK.py`: Main script to run the simulation and GUI.
- `robot_7dof.xml`: MuJoCo XML model of the 7-DOF robot.
- STL files: Mesh files for the robot's components.

## Troubleshooting

- If you encounter issues with MuJoCo, ensure that the `MUJOCO_PY_MJKEY_PATH` and `MUJOCO_PY_MJPRO_PATH` environment variables are set correctly.
- If the simulation is unstable, check the `MUJOCO_LOG.TXT` file for warnings or errors.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.