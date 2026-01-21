import mujoco
import mujoco.viewer
import numpy as np
from tqdm import tqdm
import os
import csv
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
pT = np.array([-1, -0.55, -1.2, -2, -2.4, 3.2, 2])  # 终止位置
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

# 创建 data 文件夹（如果不存在）
data_dir = os.path.join(script_dir, 'data')
os.makedirs(data_dir, exist_ok=True)

# 保存关节力矩数据到 CSV
torque_csv_path = os.path.join(data_dir, 'joint_torques.csv')
with open(torque_csv_path, 'w', newline='') as f:
    writer = csv.writer(f)
    # 写入表头
    header = ['time', 'joint1_torque', 'joint2_torque', 'joint3_torque', 
              'joint4_torque', 'joint5_torque', 'joint6_torque', 'joint7_torque']
    writer.writerow(header)
    # 写入数据
    for i in range(len(time_axis)):
        row = [time_axis[i]] + list(measured_torques[i])
        writer.writerow(row)
print(f"关节力矩数据已保存到: {torque_csv_path}")

# 保存关节角度数据到 CSV（包含期望和实际）
position_csv_path = os.path.join(data_dir, 'joint_positions.csv')
with open(position_csv_path, 'w', newline='') as f:
    writer = csv.writer(f)
    # 写入表头
    header = ['time', 
              'joint1_desired', 'joint2_desired', 'joint3_desired', 'joint4_desired',
              'joint5_desired', 'joint6_desired', 'joint7_desired',
              'joint1_actual', 'joint2_actual', 'joint3_actual', 'joint4_actual',
              'joint5_actual', 'joint6_actual', 'joint7_actual']
    writer.writerow(header)
    # 写入数据
    for i in range(len(time_axis)):
        idx = min(i, traj_len - 1)  # 与仿真循环保持一致
        row = [time_axis[i]] + list(pos[idx]) + list(measured_positions[i])
        writer.writerow(row)
print(f"关节角度数据已保存到: {position_csv_path}")
