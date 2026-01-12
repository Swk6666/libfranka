// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example move_ee_cartesian.cpp
 * Move the end-effector in Cartesian space along x, y, or z axis.
 * 在笛卡尔空间中沿x、y或z轴移动末端执行器
 *
 * @warning Before executing this example, make sure there is enough space around the robot.
 * 警告：执行此示例前，请确保机器人周围有足够的空间
 */

int main(int argc, char** argv) {
  // 检查参数个数
  if (argc != 1 && argc != 3 && argc != 4) {
    std::cerr << "Usage: " << argv[0] << " [direction distance [time]]" << std::endl
              << "direction: x, y, or z (default: x)" << std::endl
              << "distance: distance to move in meters (default: 0.1)" << std::endl
              << "time: motion time in seconds (default: 5.0)" << std::endl
              << "Examples:" << std::endl
              << "  " << argv[0] << "              # x, 0.1m, 5s" << std::endl
              << "  " << argv[0] << " x 0.15       # x, 0.15m, 5s" << std::endl
              << "  " << argv[0] << " y 0.1 3.0    # y, 0.1m, 3s" << std::endl;
    return -1;
  }

  try {
    // 固定的机器人IP地址
    const std::string robot_ip = "172.16.1.2";
    
    // 默认参数
    std::string direction = "x";
    double distance = 0.1;
    double motion_time = 5.0;

    auto parse_double = [](const char* input, const char* name, double* output) -> bool {
      const std::string input_str(input);
      try {
        size_t parsed = 0;
        double value = std::stod(input_str, &parsed);
        if (parsed != input_str.size() || !std::isfinite(value)) {
          std::cerr << "Error: " << name << " is not a valid number: " << input_str << std::endl;
          return false;
        }
        *output = value;
        return true;
      } catch (const std::exception&) {
        std::cerr << "Error: " << name << " is not a valid number: " << input_str << std::endl;
        return false;
      }
    };
    
    // 从命令行读取参数
    if (argc >= 3) {
      direction = argv[1];
      if (!parse_double(argv[2], "distance", &distance)) {
        return -1;
      }
      
      // 检查方向是否有效
      if (direction != "x" && direction != "y" && direction != "z") {
        std::cerr << "Error: direction must be x, y, or z" << std::endl;
        return -1;
      }
      
      // 检查距离是否安全
      if (std::abs(distance) > 0.5) {
        std::cerr << "Error: distance should not exceed 0.5m for safety" << std::endl;
        return -1;
      }
      
      // 读取可选的时间参数
      if (argc == 4) {
        if (!parse_double(argv[3], "motion time", &motion_time)) {
          return -1;
        }
        if (motion_time <= 0.0 || motion_time > 10.0) {
          std::cerr << "Error: motion time should be between 0 and 10 seconds" << std::endl;
          return -1;
        }
      }
      
      std::cout << "Moving " << distance << " meters in " << direction << "-direction over " 
                << motion_time << " seconds" << std::endl;
    } else {
      std::cout << "Using defaults: " << distance << " meters in " << direction 
                << "-direction over " << motion_time << " seconds" << std::endl;
    }

    std::cout << "Connecting to robot at " << robot_ip << "..." << std::endl;
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // 在控制循环之前设置附加参数，永远不要在控制循环中设置！
    // Set collision behavior.
    // 设置碰撞检测行为
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    // 确定要修改的坐标轴索引
    // O_T_EE数组中：[12]=x, [13]=y, [14]=z
    int axis_index = 12;  // 默认x轴
    if (direction == "y") {
      axis_index = 13;
    } else if (direction == "z") {
      axis_index = 14;
    }
    
    // 五次多项式轨迹规划
    // 边界条件：位置(0->distance)，速度(0->0)，加速度(0->0)
    // 归一化的五次多项式: s(τ) = 10τ³ - 15τ⁴ + 6τ⁵, τ∈[0,1]
    // 其中 τ = t/T 是归一化时间
    auto quintic_polynomial = [](double tau) -> double {
      // 限制tau在[0,1]范围内
      tau = std::max(0.0, std::min(1.0, tau));
      // 五次多项式公式
      return 10.0 * std::pow(tau, 3) - 15.0 * std::pow(tau, 4) + 6.0 * std::pow(tau, 5);
    };
    
    std::cout << "\n=== Quintic Polynomial Trajectory ===" << std::endl;
    std::cout << "Motion time: " << motion_time << " seconds" << std::endl;
    std::cout << "Distance: " << distance << " meters in " << direction << "-direction" << std::endl;
    std::cout << "Initial velocity: 0 m/s" << std::endl;
    std::cout << "Final velocity: 0 m/s" << std::endl;
    std::cout << "Initial acceleration: 0 m/s²" << std::endl;
    std::cout << "Final acceleration: 0 m/s²" << std::endl;
    
    std::cout << "\nWARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    std::cout << "Starting Cartesian motion with quintic polynomial trajectory..." << std::endl;
    
    // 存储初始位姿和时间
    bool initialized = false;
    std::array<double, 16> initial_pose;
    double time = 0.0;
    
    // 执行笛卡尔运动控制
    robot.control([&time, &initialized, &initial_pose, distance, motion_time, axis_index, quintic_polynomial](
                      const franka::RobotState& robot_state,
                      franka::Duration period) -> franka::CartesianPose {
      
      // 在第一次迭代时保存初始位姿
      if (!initialized) {
        initial_pose = robot_state.O_T_EE_c;
        initialized = true;
      }

      // 累加时间
      time += period.toSec();

      // 计算归一化时间 τ = t/T
      double tau = time / motion_time;
      
      // 使用五次多项式计算当前位置
      double s = quintic_polynomial(tau);
      double delta = distance * s;

      // 创建新的位姿（根据指定的方向改变相应坐标）
      std::array<double, 16> new_pose = initial_pose;
      new_pose[axis_index] += delta;
      
      // 运动结束条件
      if (time >= motion_time) {
        return franka::MotionFinished(new_pose);
      }
      
      return new_pose;
    });
    
    std::cout << "Cartesian motion completed successfully!" << std::endl;
    
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
