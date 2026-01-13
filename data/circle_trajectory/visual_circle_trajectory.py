#!/usr/bin/env python3
"""
圆轨迹3D动画可视化脚本

该脚本可视化由 franka/circle_trajectory.h 生成的轨迹数据。
轨迹包含两个阶段:
1. 线性段: 从圆心沿+X方向移动到圆边缘
2. 圆弧段: 在XY平面内完成一个完整的圆周运动

使用五次多项式时间缩放确保边界处速度和加速度为零。
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import os



def load_trajectory(csv_path: str) -> pd.DataFrame:
    """加载轨迹CSV数据"""
    df = pd.read_csv(csv_path)
    print(f"加载了 {len(df)} 个轨迹点")
    print(f"时间范围: {df['time'].min():.3f}s - {df['time'].max():.3f}s")
    print(f"X范围: [{df['px'].min():.4f}, {df['px'].max():.4f}]")
    print(f"Y范围: [{df['py'].min():.4f}, {df['py'].max():.4f}]")
    print(f"Z范围: [{df['pz'].min():.4f}, {df['pz'].max():.4f}]")
    return df


def create_3d_animation(df: pd.DataFrame, output_path: str = None, skip_frames: int = 10):
    """
    创建3D动画可视化
    
    Args:
        df: 包含轨迹数据的DataFrame
        output_path: 保存动画的路径 (可选)
        skip_frames: 跳过的帧数，用于加速动画 (默认每10帧取1帧)
    """
    # 提取位置数据
    x = df['px'].values
    y = df['py'].values
    z = df['pz'].values
    time = df['time'].values
    
    # 降采样以加快动画
    indices = np.arange(0, len(x), skip_frames)
    x_sampled = x[indices]
    y_sampled = y[indices]
    z_sampled = z[indices]
    time_sampled = time[indices]
    
    # 创建图形 - 使用深色主题
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(14, 10))
    
    # 主3D视图
    ax = fig.add_subplot(111, projection='3d')
    
    # 设置背景色
    fig.patch.set_facecolor('#1a1a2e')
    ax.set_facecolor('#16213e')
    
    # 设置坐标轴范围
    x_range = x.max() - x.min()
    y_range = y.max() - y.min()
    z_range = max(0.01, z.max() - z.min())  # 防止z范围为0
    max_range = max(x_range, y_range, z_range) * 0.6
    
    x_center = (x.max() + x.min()) / 2
    y_center = (y.max() + y.min()) / 2
    z_center = (z.max() + z.min()) / 2
    
    ax.set_xlim(x_center - max_range, x_center + max_range)
    ax.set_ylim(y_center - max_range, y_center + max_range)
    ax.set_zlim(z_center - max_range * 0.5, z_center + max_range * 0.5)
    
    # 设置标签
    ax.set_xlabel('X (m)', fontsize=12, color='#e94560', labelpad=10)
    ax.set_ylabel('Y (m)', fontsize=12, color='#0f3460', labelpad=10)
    ax.set_zlabel('Z (m)', fontsize=12, color='#533483', labelpad=10)
    
    # 设置刻度颜色
    ax.tick_params(colors='#a0a0a0')
    
    # 绘制完整轨迹作为背景 (淡色)
    ax.plot(x, y, z, 'w-', alpha=0.15, linewidth=1, label='Complete trajectory')
    
    # 绘制圆心标记
    circle_center_x = x[0]
    circle_center_y = y[0]
    circle_center_z = z[0]
    ax.scatter([circle_center_x], [circle_center_y], [circle_center_z], 
               c='#00ff88', s=100, marker='o', label='Start (Circle center)', 
               edgecolors='white', linewidths=2, zorder=5)
    
    # 在XY平面绘制投影
    ax.plot(x, y, np.full_like(z, z_center - max_range * 0.5), 
            'c-', alpha=0.2, linewidth=0.5, linestyle='--')
    
    # 初始化动画元素
    trail_line, = ax.plot([], [], [], 'w-', linewidth=2.5, alpha=0.9)
    
    # 渐变色轨迹
    from matplotlib.collections import LineCollection
    from mpl_toolkits.mplot3d.art3d import Line3DCollection
    
    # 当前点标记
    current_point, = ax.plot([], [], [], 'o', markersize=12, 
                             color='#ff6b6b', markeredgecolor='white', 
                             markeredgewidth=2, zorder=10)
    
    # 速度矢量
    velocity_arrow = None
    
    # 标题
    title = ax.set_title('Circle Trajectory Animation\nTime: 0.000s', 
                         fontsize=14, color='#e8e8e8', pad=20)
    
    # 添加信息文本
    info_text = fig.text(0.02, 0.95, '', fontsize=10, color='#a0a0a0',
                         transform=fig.transFigure, verticalalignment='top',
                         fontfamily='monospace')
    
    # 添加阶段指示
    phase_text = fig.text(0.02, 0.02, '', fontsize=12, color='#00ff88',
                          transform=fig.transFigure, verticalalignment='bottom',
                          fontweight='bold')
    
    # 计算线性段结束时间 (根据轨迹特征判断)
    # 线性段: y基本为0, x递增
    # 圆弧段: y开始变化
    y_threshold = 0.0001
    line_end_idx = 0
    for i, yi in enumerate(y):
        if abs(yi) > y_threshold:
            line_end_idx = i
            break
    line_end_time = time[line_end_idx] if line_end_idx > 0 else time[len(time)//3]
    
    def init():
        trail_line.set_data([], [])
        trail_line.set_3d_properties([])
        current_point.set_data([], [])
        current_point.set_3d_properties([])
        return trail_line, current_point
    
    def update(frame):
        nonlocal velocity_arrow
        
        # 获取当前帧索引
        idx = frame
        
        # 获取原始数据中对应的索引
        orig_idx = indices[idx]
        
        # 更新轨迹线 (渐变效果 - 只显示最近的轨迹)
        trail_length = min(idx + 1, 100)  # 最多显示100个点的轨迹
        start_idx = max(0, idx - trail_length + 1)
        
        trail_x = x_sampled[start_idx:idx+1]
        trail_y = y_sampled[start_idx:idx+1]
        trail_z = z_sampled[start_idx:idx+1]
        
        trail_line.set_data(trail_x, trail_y)
        trail_line.set_3d_properties(trail_z)
        
        # 根据时间设置轨迹颜色
        current_time = time_sampled[idx]
        if current_time <= line_end_time:
            trail_line.set_color('#ff6b6b')  # 线性段 - 红色
            phase = "Phase: Linear Segment (Center → Edge)"
            phase_color = '#ff6b6b'
        else:
            trail_line.set_color('#4ecdc4')  # 圆弧段 - 青色
            phase = "Phase: Circular Segment"
            phase_color = '#4ecdc4'
        
        # 更新当前点
        current_point.set_data([x_sampled[idx]], [y_sampled[idx]])
        current_point.set_3d_properties([z_sampled[idx]])
        
        # 更新速度矢量
        if velocity_arrow is not None:
            velocity_arrow.remove()
        
        vx = df['vx'].values[orig_idx]
        vy = df['vy'].values[orig_idx]
        vz = df['vz'].values[orig_idx]
        v_mag = np.sqrt(vx**2 + vy**2 + vz**2)
        
        if v_mag > 1e-6:
            # 缩放速度矢量以便可视化
            scale = 0.3 / max(v_mag, 0.01)
            velocity_arrow = ax.quiver(x_sampled[idx], y_sampled[idx], z_sampled[idx],
                                       vx * scale, vy * scale, vz * scale,
                                       color='#ffd93d', arrow_length_ratio=0.3,
                                       linewidth=2, alpha=0.8)
        else:
            velocity_arrow = None
        
        # 更新标题
        title.set_text(f'Circle Trajectory Animation\nTime: {current_time:.3f}s')
        
        # 更新信息文本
        info = (f'Position: ({x_sampled[idx]:.4f}, {y_sampled[idx]:.4f}, {z_sampled[idx]:.4f}) m\n'
                f'Velocity: ({vx:.4f}, {vy:.4f}, {vz:.4f}) m/s\n'
                f'Speed: {v_mag:.4f} m/s')
        info_text.set_text(info)
        
        # 更新阶段文本
        phase_text.set_text(phase)
        phase_text.set_color(phase_color)
        
        return trail_line, current_point
    
    # 创建动画
    num_frames = len(x_sampled)
    anim = FuncAnimation(fig, update, frames=num_frames, init_func=init,
                         interval=20, blit=False, repeat=True)
    
    # 添加图例
    ax.legend(loc='upper right', fontsize=9, facecolor='#1a1a2e', 
              edgecolor='#a0a0a0', labelcolor='#e8e8e8')
    
    # 设置视角
    ax.view_init(elev=25, azim=-60)
    
    plt.tight_layout()
    
    # 保存或显示
    if output_path:
        print(f"正在保存动画到 {output_path}...")
        anim.save(output_path, writer='pillow', fps=50, dpi=100)
        print("保存完成!")
    
    plt.show()
    
    return anim


def create_static_visualization(df: pd.DataFrame, output_path: str = None):
    """创建静态3D可视化"""
    x = df['px'].values
    y = df['py'].values
    z = df['pz'].values
    time = df['time'].values
    
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(16, 6))
    
    # 3D轨迹图
    ax1 = fig.add_subplot(131, projection='3d')
    
    # 使用时间作为颜色映射
    scatter = ax1.scatter(x, y, z, c=time, cmap='plasma', s=1, alpha=0.8)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Trajectory (color = time)')
    plt.colorbar(scatter, ax=ax1, label='Time (s)', shrink=0.6)
    
    # XY平面投影
    ax2 = fig.add_subplot(132)
    scatter2 = ax2.scatter(x, y, c=time, cmap='plasma', s=1, alpha=0.8)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('XY Plane Projection')
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    plt.colorbar(scatter2, ax=ax2, label='Time (s)')
    
    # 速度曲线
    ax3 = fig.add_subplot(133)
    speed = np.sqrt(df['vx']**2 + df['vy']**2 + df['vz']**2)
    ax3.plot(time, speed, 'c-', linewidth=1, label='Speed')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Speed (m/s)')
    ax3.set_title('Speed Profile')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    plt.tight_layout()
    
    if output_path:
        plt.savefig(output_path, dpi=150, facecolor='#1a1a2e')
        print(f"静态图已保存到 {output_path}")
    
    plt.show()


def main():
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(script_dir, 'circle_trajectory.csv')
    
    # 加载数据
    print("=" * 50)
    print("Circle Trajectory Visualization")
    print("=" * 50)
    df = load_trajectory(csv_path)
    print()
    
    # 创建静态可视化
    print("生成静态可视化...")
    static_output = os.path.join(script_dir, 'circle_trajectory_static.png')
    create_static_visualization(df, static_output)
    
    # 创建3D动画
    print("\n生成3D动画...")
    print("提示: 关闭静态图窗口后将显示动画")
    print("      动画中红色表示线性段,青色表示圆弧段")
    print("      黄色箭头表示速度方向")
    
    # 可选: 保存为GIF
    # gif_output = os.path.join(script_dir, 'circle_trajectory_animation.gif')
    # create_3d_animation(df, output_path=gif_output, skip_frames=20)
    
    # 直接显示动画
    create_3d_animation(df, skip_frames=10)


if __name__ == '__main__':
    main()
