// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <exception>
#include <iostream>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example move_to_mujoco_keyframe.cpp
 * Move the robot to MuJoCo keyframe position.
 * 将机器人移动到 MuJoCo 关键帧位置
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 * 警告：执行此示例前，请确保机器人周围有足够的空间
 */

int main(int argc, char** argv) {
  // 检查参数个数
  if (argc > 2) {
    std::cerr << "Usage: " << argv[0] << " [speed-factor]" << std::endl
              << "speed-factor: optional, between 0 and 1 (default: 0.5)" << std::endl;
    return -1;
  }

  try {
    // 固定的机器人IP地址
    const std::string robot_ip = "172.16.1.2";
    
    // MuJoCo keyframe 位置：(0, 0, 0, -1.57, 0, 1.57, -0.785)
    std::array<double, 7> q_goal = {{0.0, 0.0, 0.0, -1.57, 0.0, 1.57, -0.785}};
    
    // 速度因子（0-1之间）- 从命令行读取或使用默认值
    double speed_factor = 0.5;  // 默认速度因子
    if (argc == 2) {
      const std::string speed_factor_arg(argv[1]);
      try {
        size_t parsed = 0;
        speed_factor = std::stod(speed_factor_arg, &parsed);
        if (parsed != speed_factor_arg.size() || !std::isfinite(speed_factor)) {
          std::cerr << "Error: speed-factor is not a valid number: " << speed_factor_arg << std::endl;
          return -1;
        }
      } catch (const std::exception&) {
        std::cerr << "Error: speed-factor is not a valid number: " << speed_factor_arg << std::endl;
        return -1;
      }
      if (speed_factor <= 0.0 || speed_factor > 1.0) {
        std::cerr << "Error: speed-factor must be between 0 and 1" << std::endl;
        return -1;
      }
      std::cout << "Using custom speed factor: " << speed_factor << std::endl;
    } else {
      std::cout << "Using default speed factor: " << speed_factor << std::endl;
    }

    std::cout << "Connecting to robot at " << robot_ip << "..." << std::endl;
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    std::cout << "Target joint angles: [";
    for (size_t i = 0; i < 7; i++) {
      std::cout << q_goal[i];
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

    // 创建运动生成器
    MotionGenerator motion_generator(speed_factor, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    // 执行运动
    robot.control(motion_generator);
    std::cout << "Motion finished successfully!" << std::endl;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
