import mujoco
import mujoco.viewer
import numpy as np
from tqdm import tqdm
import os
from src.Polynomial_traj import plan_quintic, quintic_coeffs, sample_trajectory

# Load the model from the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(script_dir, 'franka_emika_panda', 'scene.xml')
mj_model = mujoco.MjModel.from_xml_path(xml_path)
mj_data = mujoco.MjData(mj_model)
# 设定仿真时间步长（默认 0.002s，会导致时间翻倍），与轨迹 dt 保持一致
SIM_DT = 0.001
mj_model.opt.timestep = SIM_DT


# 关节空间内进行五次多项式的规划
p0 = np.array([0, 0, 0, -1.57079, 0, 1.57079,  -0.7853])   # 初始位置
pT = np.array([-1, -0.35, -0.84, -2, -1, 2, 0.23])  # 终止位置
T = 5.0  # 终止时间
dt = SIM_DT
num = int(T / dt) + 1  # 20001

coeffs, t, pos, vel, acc = plan_quintic(p0, pT, T=T, num=num)
traj_len = pos.shape[0]

mj_data.qpos[:7] = p0
mj_data.qvel[:7] = 0
mj_data.ctrl[:7] = p0
mujoco.mj_forward(mj_model, mj_data)

measured_torques = []
measured_positions = []


with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer, tqdm(total=0, dynamic_ncols=True) as pbar:
    step_count = 0
    print_interval = 100  # 每100步更新一次 tqdm 显示
    
    while viewer.is_running():
        idx = min(step_count, traj_len - 1)  # 超出轨迹后保持末端姿态
        mj_data.ctrl[:] = pos[idx]  # 设置当前时间步的关节位置作为控制输入
        mujoco.mj_step(mj_model, mj_data)
        measured_torques.append(mj_data.qfrc_actuator[:7])
        measured_positions.append(mj_data.qpos[:7])
        step_count += 1

        # 轨迹执行完毕后自动退出
        if step_count >= traj_len:
            break

        if step_count % print_interval == 0:

            pbar.set_description(f"time: {mj_data.time:.3f}s")
            pbar.update(1)  # 更新一次进度条（动态刷新）

        viewer.sync()
