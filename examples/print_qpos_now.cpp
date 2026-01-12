// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>

/**
 * @example print_qpos_now.cpp
 * Print current joint positions of the robot.
 * 打印机器人当前的关节角度位置
 */

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  try {
    // 固定的机器人IP地址
    const std::string robot_ip = "172.16.1.2";
    
    std::cout << "Connecting to robot at " << robot_ip << "..." << std::endl;
    franka::Robot robot(robot_ip);
    
    // 读取一次机器人状态
    franka::RobotState state = robot.readOnce();
    
    // 打印当前关节角度（单位：弧度）
    std::cout << "\n=== Current Joint Positions (rad) ===" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "[";
    for (size_t i = 0; i < 7; i++) {
      std::cout << state.q[i];
      if (i < 6) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    

  
    
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
