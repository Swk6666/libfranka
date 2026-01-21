import numpy as np
import matplotlib.pyplot as plt
import os
import csv

# 获取脚本所在目录
script_dir = os.path.dirname(os.path.abspath(__file__))
data_dir = os.path.join(script_dir, 'data')

# 加载关节力矩数据
torque_csv_path = os.path.join(data_dir, 'joint_torques.csv')
torque_data = []
with open(torque_csv_path, 'r') as f:
    reader = csv.reader(f)
    header = next(reader)  # 跳过表头
    for row in reader:
        torque_data.append([float(x) for x in row])
torque_data = np.array(torque_data)

# 加载关节角度数据
position_csv_path = os.path.join(data_dir, 'joint_positions.csv')
position_data = []
with open(position_csv_path, 'r') as f:
    reader = csv.reader(f)
    header = next(reader)  # 跳过表头
    for row in reader:
        position_data.append([float(x) for x in row])
position_data = np.array(position_data)

# 提取数据
time_axis = torque_data[:, 0]
measured_torques = torque_data[:, 1:8]

desired_positions = position_data[:, 1:8]
actual_positions = position_data[:, 8:15]

joint_names = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7']

# 第一幅图：关节力矩 (2x4布局，显示7个关节力矩)
fig1, axes1 = plt.subplots(2, 4, figsize=(16, 8))
fig1.suptitle('Joint Torques', fontsize=16)

for i in range(7):
    row = i // 4
    col = i % 4
    axes1[row, col].plot(time_axis, measured_torques[:, i], 'b-', linewidth=1.5)
    axes1[row, col].set_title(f'{joint_names[i]} Torque')
    axes1[row, col].set_xlabel('Time (s)')
    axes1[row, col].set_ylabel('Torque (Nm)')
    axes1[row, col].grid(True, alpha=0.3)

# 隐藏第8个子图（因为只有7个关节）
axes1[1, 3].set_visible(False)

plt.tight_layout()

# 第二幅图：关节角度对比 (期望vs实际，2x4布局)
fig2, axes2 = plt.subplots(2, 4, figsize=(16, 8))
fig2.suptitle('Joint Angles: Desired vs Actual', fontsize=16)

for i in range(7):
    row = i // 4
    col = i % 4
    axes2[row, col].plot(time_axis, desired_positions[:, i], 'r--', linewidth=1.5, label='Desired')
    axes2[row, col].plot(time_axis, actual_positions[:, i], 'b-', linewidth=1.5, label='Actual')
    axes2[row, col].set_title(f'{joint_names[i]} Angle')
    axes2[row, col].set_xlabel('Time (s)')
    axes2[row, col].set_ylabel('Angle (rad)')
    axes2[row, col].grid(True, alpha=0.3)
    axes2[row, col].legend()

# 隐藏第8个子图（因为只有7个关节）
axes2[1, 3].set_visible(False)

plt.tight_layout()

# 第三幅图：关节角度跟踪误差
fig3, axes3 = plt.subplots(2, 4, figsize=(16, 8))
fig3.suptitle('Joint Angle Tracking Error (Desired - Actual)', fontsize=16)

for i in range(7):
    row = i // 4
    col = i % 4
    error = desired_positions[:, i] - actual_positions[:, i]
    axes3[row, col].plot(time_axis, error, 'g-', linewidth=1.5)
    axes3[row, col].set_title(f'{joint_names[i]} Error')
    axes3[row, col].set_xlabel('Time (s)')
    axes3[row, col].set_ylabel('Error (rad)')
    axes3[row, col].grid(True, alpha=0.3)
    # 添加零线
    axes3[row, col].axhline(y=0, color='k', linestyle='--', linewidth=0.5)

# 隐藏第8个子图
axes3[1, 3].set_visible(False)

plt.tight_layout()

plt.show()

print(f"已加载数据:")
print(f"  - 力矩数据: {torque_csv_path}")
print(f"  - 角度数据: {position_csv_path}")
print(f"  - 数据点数: {len(time_axis)}")
print(f"  - 时间范围: {time_axis[0]:.3f}s ~ {time_axis[-1]:.3f}s")

