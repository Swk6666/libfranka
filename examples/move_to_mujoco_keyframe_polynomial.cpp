// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <exception>
#include <iostream>
#include <string>

#include <franka/exception.h>
#include <franka/quintic_polynomial.h>
#include <franka/robot.h>

#include "examples_common.h"

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
    std::array<double, 7> q_end = {{0.0, 0.0, 0.0, -M_PI_2, 0.0, M_PI_2, -M_PI_4}};
    
    // 运动时间（秒）- 从命令行读取或使用默认值
    double motion_time = 10.0;  // 默认10秒
    if (argc == 2) 
    {
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
    } 
    else 
    {
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

    // 创建五次多项式轨迹生成器
    // Create quintic polynomial trajectory generator
    // 边界条件：位置(q_start->q_end)，速度(0->0)，加速度(0->0)
    // 归一化的五次多项式: s(τ) = 10τ³ - 15τ⁴ + 6τ⁵, τ∈[0,1]
    franka::QuinticPolynomial trajectory(motion_time);
    
    std::cout << "\n=== Quintic Polynomial Trajectory ===" << std::endl;
    std::cout << "Motion time: " << motion_time << " seconds" << std::endl;
    
    std::cout << "\nWARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    // 存储初始化标志
    bool initialized = false;
    std::array<double, 7> q_start_actual = q_start;

    std::cout << "Starting quintic polynomial motion from q_start to q_end..." << std::endl;
    
    // 执行五次多项式轨迹运动
    // 使用 QuinticPolynomial 类自动处理向量插值
    robot.control([&trajectory, &initialized, &q_start_actual, q_end](
                      const franka::RobotState& robot_state,
                      franka::Duration period) -> franka::JointPositions 
                      {
      
      // 初始化：记录实际起始位置
      if (!initialized) {
        initialized = true;
        q_start_actual = robot_state.q_d;
      }
      
      // 使用 QuinticPolynomial 类进行向量插值
      // The interpolate method automatically handles std::array<double, 7>
      std::array<double, 7> q_current = trajectory.interpolate(q_start_actual, q_end, period.toSec());
      
      franka::JointPositions output(q_current);
      
      // 运动结束条件
      if (trajectory.isFinished()) {
        return franka::MotionFinished(output);
      }
      
      return output;
    });
    
    std::cout << "Quintic polynomial motion completed successfully!" << std::endl;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
