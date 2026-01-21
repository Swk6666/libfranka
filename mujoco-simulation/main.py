import mujoco
import mujoco.viewer
import numpy as np
from tqdm import tqdm
import os
import matplotlib.pyplot as plt
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
        measured_torques.append(mj_data.qfrc_actuator[:7].copy())
        measured_positions.append(mj_data.qpos[:7].copy())
        step_count += 1

        # 轨迹执行完毕后自动退出
        if step_count >= traj_len:
            break

        if step_count % print_interval == 0:

            pbar.set_description(f"time: {mj_data.time:.3f}s")
            pbar.update(1)  # 更新一次进度条（动态刷新）

        viewer.sync()

# 转换为numpy数组
measured_torques = np.array(measured_torques)
measured_positions = np.array(measured_positions)

# Time axis
time_axis = np.linspace(0, T, len(measured_torques))
joint_names = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7']

# 第一幅图：关节力矩 (2x4布局，显示7个关节力矩)
fig1, axes1 = plt.subplots(2, 4, figsize=(16, 8))
fig1.suptitle('Joint Torques', fontsize=16)

for i in range(7):
    row = i // 4
    col = i % 4
    axes1[row, col].plot(time_axis, measured_torques[:, i], 'b-', linewidth=2)
    axes1[row, col].set_title(f'{joint_names[i]} Torque')
    axes1[row, col].set_xlabel('Time (s)')
    axes1[row, col].set_ylabel('Torque (Nm)')
    axes1[row, col].grid(True, alpha=0.3)

# 隐藏第8个子图（因为只有7个关节）
axes1[1, 3].set_visible(False)

plt.tight_layout()
plt.show()

# 第二幅图：关节角度对比 (期望vs实际，2x4布局)
fig2, axes2 = plt.subplots(2, 4, figsize=(16, 8))
fig2.suptitle('Joint Angles: Desired vs Actual', fontsize=16)

for i in range(7):
    row = i // 4
    col = i % 4
    axes2[row, col].plot(t, pos[:, i], 'r--', linewidth=2, label='Desired')
    axes2[row, col].plot(time_axis, measured_positions[:, i], 'b-', linewidth=2, label='Actual')
    axes2[row, col].set_title(f'{joint_names[i]} Angle')
    axes2[row, col].set_xlabel('Time (s)')
    axes2[row, col].set_ylabel('Angle (rad)')
    axes2[row, col].grid(True, alpha=0.3)
    axes2[row, col].legend()

# 隐藏第8个子图（因为只有7个关节）
axes2[1, 3].set_visible(False)

plt.tight_layout()
plt.show()
