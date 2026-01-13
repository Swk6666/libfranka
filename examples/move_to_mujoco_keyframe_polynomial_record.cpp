// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

// 用于存储每个时间步的数据记录
struct DataRecord {
  double time;                          // 时间戳
  std::array<double, 7> q_desired;      // 期望关节角度
  std::array<double, 7> q_actual;       // 实际关节角度
  std::array<double, 7> dq_actual;      // 实际关节角速度
  std::array<double, 7> tau_J;          // 测量的关节力矩
  std::array<double, 7> tau_J_d;        // 期望的关节力矩（控制器计算）
};

/**
 * @example move_to_mujoco_keyframe_polynomial.cpp
 * Move the robot from q_start to q_end using quintic polynomial trajectory.
 * 使用五次多项式轨迹从q_start移动到q_end
 *
 * @warning Before executing this example, make sure there is enough space around the robot.
 * 警告：执行此示例前，请确保机器人周围有足够的空间
 */

int main(int argc, char** argv) {
  // 检查参数个数
  if (argc > 2) {
    std::cerr << "Usage: " << argv[0] << " [motion-time]" << std::endl
              << "motion-time: optional, motion time in seconds (default: 10.0)" << std::endl;
    return -1;
  }

  try {
    // 固定的机器人IP地址
    const std::string robot_ip = "172.16.1.2";
    
    // 起始和目标关节位置
    std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    std::array<double, 7> q_end = {{0, -M_PI_4*0.5, 0, -2 * M_PI_4, 0, 0, M_PI_4}};
    
    // 运动时间（秒）- 从命令行读取或使用默认值
    double motion_time = 10.0;  // 默认10秒
    if (argc == 2) {
      const std::string motion_time_arg(argv[1]);
      try {
        size_t parsed = 0;
        motion_time = std::stod(motion_time_arg, &parsed);
        if (parsed != motion_time_arg.size()) {
          std::cerr << "Error: motion time is not a valid number: " << motion_time_arg << std::endl;
          return -1;
        }
      } catch (const std::exception&) {
        std::cerr << "Error: motion time is not a valid number: " << motion_time_arg << std::endl;
        return -1;
      }
      if (!std::isfinite(motion_time) || motion_time <= 0.0 || motion_time > 30.0) {
        std::cerr << "Error: motion time should be between 0 and 30 seconds" << std::endl;
        return -1;
      }
      std::cout << "Using custom motion time: " << motion_time << " seconds" << std::endl;
    } else {
      std::cout << "Using default motion time: " << motion_time << " seconds" << std::endl;
    }

    std::cout << "Connecting to robot at " << robot_ip << "..." << std::endl;
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    // 打印起始和目标位置
    std::cout << "\nStart joint angles: [";
    for (size_t i = 0; i < 7; i++) {
      std::cout << q_start[i];
      if (i < 6) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    std::cout << "End joint angles: [";
    for (size_t i = 0; i < 7; i++) {
      std::cout << q_end[i];
      if (i < 6) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // 在控制循环之前设置附加参数，永远不要在控制循环中设置！
    // Set collision behavior.
    // 设置碰撞检测行为
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    // 先移动到起始位置
    std::cout << "\nMoving to start position..." << std::endl;
    MotionGenerator motion_to_start(0.5, q_start);
    robot.control(motion_to_start);
    std::cout << "Reached start position." << std::endl;

    // 五次多项式轨迹规划
    // 边界条件：位置(q_start->q_end)，速度(0->0)，加速度(0->0)
    // 归一化的五次多项式: s(τ) = 10τ³ - 15τ⁴ + 6τ⁵, τ∈[0,1]
    auto quintic_polynomial = [](double tau) -> double {
      if (tau < 0.0) {
        tau = 0.0;
      } else if (tau > 1.0) {
        tau = 1.0;
      }
      double tau2 = tau * tau;
      double tau3 = tau2 * tau;
      double tau4 = tau2 * tau2;
      double tau5 = tau4 * tau;
      return 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
    };
    
    std::cout << "\n=== Quintic Polynomial Trajectory ===" << std::endl;
    std::cout << "Motion time: " << motion_time << " seconds" << std::endl;
    std::cout << "Initial velocity: 0 rad/s (all joints)" << std::endl;
    std::cout << "Final velocity: 0 rad/s (all joints)" << std::endl;
    std::cout << "Initial acceleration: 0 rad/s² (all joints)" << std::endl;
    std::cout << "Final acceleration: 0 rad/s² (all joints)" << std::endl;
    
    std::cout << "\nWARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    // 存储初始化标志和时间
    bool initialized = false;
    double time = 0.0;
    std::array<double, 7> q_start_actual = q_start;
    
    // 数据记录容器 - 预分配固定大小，避免实时循环中的动态内存分配
    // 1kHz采样率，预分配足够空间
    const size_t max_samples = static_cast<size_t>(motion_time * 1000) + 100;
    std::vector<DataRecord> recorded_data(max_samples);  // 预分配并初始化
    size_t sample_index = 0;  // 当前记录索引

    std::cout << "Starting quintic polynomial motion from q_start to q_end..." << std::endl;
    std::cout << "Recording data during motion..." << std::endl;
    
    // 执行五次多项式轨迹运动
    robot.control([&time, &initialized, &q_start_actual, &recorded_data, &sample_index, max_samples, q_end, motion_time, quintic_polynomial](
                      const franka::RobotState& robot_state,
                      franka::Duration period) -> franka::JointPositions {
      
      // 初始化
      if (!initialized) {
        initialized = true;
        q_start_actual = robot_state.q;
      }
      
      // 累加时间
      time += period.toSec();
      
      // 计算归一化时间 τ = t/T
      double tau = time / motion_time;
      
      // 使用五次多项式计算插值系数
      double s = quintic_polynomial(tau);
      
      // 对每个关节进行插值: q(t) = q_start + s * (q_end - q_start)
      franka::JointPositions output(q_start_actual);
      for (size_t i = 0; i < 7; i++) {
        output.q[i] = q_start_actual[i] + s * (q_end[i] - q_start_actual[i]);
      }
      
      // 记录数据 - 直接写入预分配的数组，无动态内存分配
      if (sample_index < max_samples) {
        recorded_data[sample_index].time = time;
        recorded_data[sample_index].q_desired = output.q;
        recorded_data[sample_index].q_actual = robot_state.q;
        recorded_data[sample_index].dq_actual = robot_state.dq;
        recorded_data[sample_index].tau_J = robot_state.tau_J;
        recorded_data[sample_index].tau_J_d = robot_state.tau_J_d;
        sample_index++;
      }
      
      // 运动结束条件
      if (time >= motion_time) {
        return franka::MotionFinished(output);
      }
      
      return output;
    });
    
    // 调整 vector 大小为实际记录的数据量
    recorded_data.resize(sample_index);
    
    std::cout << "Quintic polynomial motion completed successfully!" << std::endl;
    std::cout << "Recorded " << recorded_data.size() << " data points." << std::endl;
    
    // 保存数据到CSV文件
    std::string csv_filename = "trajectory_data.csv";
    std::ofstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
      std::cerr << "Error: Could not open file " << csv_filename << " for writing." << std::endl;
      return -1;
    }
    
    // 写入CSV表头
    csv_file << "time";
    for (int j = 1; j <= 7; j++) csv_file << ",q_desired_" << j;
    for (int j = 1; j <= 7; j++) csv_file << ",q_actual_" << j;
    for (int j = 1; j <= 7; j++) csv_file << ",dq_actual_" << j;
    for (int j = 1; j <= 7; j++) csv_file << ",tau_J_" << j;
    for (int j = 1; j <= 7; j++) csv_file << ",tau_J_d_" << j;
    csv_file << "\n";
    
    // 写入数据
    csv_file << std::fixed << std::setprecision(6);
    for (const auto& record : recorded_data) {
      csv_file << record.time;
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.q_desired[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.q_actual[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.dq_actual[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.tau_J[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.tau_J_d[i];
      csv_file << "\n";
    }
    
    csv_file.close();
    std::cout << "Data saved to " << csv_filename << std::endl;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
