// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/circle_trajectory.h>

#include "examples_common.h"

/**
 * @example move_to_mujoco_keyframe_circle_impedance.cpp
 * Move the robot end-effector along a circular trajectory in the XY plane.
 * Uses franka::generateCircleTrajectory to generate the trajectory.
 * Tracks the Cartesian trajectory using joint impedance control with gravity compensation.
 * 
 * The motion consists of two phases:
 * 1. Linear segment: Move from circle center to edge along +X
 * 2. Circular segment: Complete a full circle in the XY plane
 *
 * @warning Before executing this example, make sure there is enough space around the robot.
 */


 /*
 # 使用默认参数 (半径=0.1m, 线性段=4s, 圆弧段=10s)
./examples/move_to_mujoco_keyframe_circle

# 自定义半径 (例如 0.08m)
./examples/move_to_mujoco_keyframe_circle 0.08

# 自定义半径和线性段时间
./examples/move_to_mujoco_keyframe_circle 0.08 3.0

# 自定义所有参数
./examples/move_to_mujoco_keyframe_circle 0.08 3.0 8.0
 */

int main(int argc, char** argv) {
  // Check arguments
  if (argc > 4) {
    std::cerr << "Usage: " << argv[0] << " [radius] [line-time] [circle-time]" << std::endl
              << "  radius: circle radius in meters (default: 0.05)" << std::endl
              << "  line-time: duration of linear segment in seconds (default: 2.0)" << std::endl
              << "  circle-time: duration of circular segment in seconds (default: 4.0)" << std::endl;
    return -1;
  }

  try {
    // Fixed robot IP address
    const std::string robot_ip = "172.16.1.2";
    
    // Initial joint configuration (same as generate_cartesian_pose_motion.cpp)
    std::array<double, 7> q_start = {{0.0, 0.0, 0.0, -1.57, 0.0, 1.57, -0.785}};
    
    // Circle trajectory parameters
    double radius = 0.1;      // Default 5cm radius
    double line_time = 10.0;    // Default 2 seconds for linear segment
    double circle_time = 20.0;  // Default 4 seconds for circular segment
    
    // Parse optional arguments
    if (argc >= 2) {
      try {
        radius = std::stod(argv[1]);
        if (!std::isfinite(radius) || radius <= 0.0 || radius > 0.3) {
          std::cerr << "Error: radius should be between 0 and 0.3 meters" << std::endl;
          return -1;
        }
      } catch (const std::exception&) {
        std::cerr << "Error: invalid radius value" << std::endl;
        return -1;
      }
    }
    
    if (argc >= 3) {
      try {
        line_time = std::stod(argv[2]);
        if (!std::isfinite(line_time) || line_time <= 0.0 || line_time > 30.0) {
          std::cerr << "Error: line-time should be between 0 and 30 seconds" << std::endl;
          return -1;
        }
      } catch (const std::exception&) {
        std::cerr << "Error: invalid line-time value" << std::endl;
        return -1;
      }
    }
    
    if (argc >= 4) {
      try {
        circle_time = std::stod(argv[3]);
        if (!std::isfinite(circle_time) || circle_time <= 0.0 || circle_time > 60.0) {
          std::cerr << "Error: circle-time should be between 0 and 60 seconds" << std::endl;
          return -1;
        }
      } catch (const std::exception&) {
        std::cerr << "Error: invalid circle-time value" << std::endl;
        return -1;
      }
    }

    std::cout << "=== Circle Trajectory Parameters ===" << std::endl;
    std::cout << "Radius: " << radius << " m" << std::endl;
    std::cout << "Linear segment time: " << line_time << " s" << std::endl;
    std::cout << "Circular segment time: " << circle_time << " s" << std::endl;
    std::cout << "Total motion time: " << (line_time + circle_time) << " s" << std::endl;

    // Generate circle trajectory (relative to origin, will be applied as offset)
    // Time step matches robot control frequency (1kHz)
    constexpr double kTimeStep = 0.001;
    franka::CircleTrajectory circle_trajectory = 
        franka::generateCircleTrajectory(radius, line_time, circle_time, kTimeStep);
    
    std::cout << "Generated " << circle_trajectory.points.size() << " trajectory points" << std::endl;

    std::cout << "\nConnecting to robot at " << robot_ip << "..." << std::endl;
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    // Set collision behavior
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    // First move the robot to the initial joint configuration
    std::cout << "\nMoving to initial joint configuration..." << std::endl;
    MotionGenerator motion_generator(0.5, q_start);
    robot.control(motion_generator);
    std::cout << "Reached initial configuration." << std::endl;

    std::cout << "\nWARNING: This example will move the robot end-effector in a circle! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // Load the kinematics and dynamics model for gravity compensation.
    franka::Model model = robot.loadModel();

    // Set gains for the joint impedance control.
    const std::array<double, 7> k_gains = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
  
    // Variables for control loop
    std::array<double, 16> initial_pose;
    std::array<double, 16> last_pose;

    // size_t 在c++中是无符号整型，专门用来表示大小，数量，索引
    size_t trajectory_index = 0;

    bool initialized = false;
    double time = 0.0;


    auto cartesian_pose_callback =
        [&circle_trajectory, &initial_pose, &last_pose, &trajectory_index, &initialized, &time](
            const franka::RobotState& robot_state,
            franka::Duration period) -> franka::CartesianPose 
        {
          // Initialize: record the initial end-effector pose
          if (!initialized) {
            initialized = true;
            initial_pose = robot_state.O_T_EE_c; //改成d试试？
            last_pose = initial_pose;
            time = 0.0;
          }

          time += period.toSec();

          // Advance trajectory index based on elapsed time to avoid large jumps on missed cycles.
          while (trajectory_index + 1 < circle_trajectory.points.size() &&
                 circle_trajectory.points[trajectory_index + 1].time <= time) {
            trajectory_index++;
          }

          // Get current trajectory point
          const auto& point = circle_trajectory.points[trajectory_index];

          // Apply trajectory offset to initial pose
          // The circle trajectory is in the XY plane (robot base frame)
          std::array<double, 16> new_pose = initial_pose;
          new_pose[12] += point.position[0];  // X offset
          new_pose[13] += point.position[1];  // Y offset
          new_pose[14] += point.position[2];  // Z offset (should be 0 for XY plane circle)

          last_pose = new_pose;
          if (time >= circle_trajectory.points.back().time) {
            return franka::MotionFinished(franka::CartesianPose(last_pose));
          }

          return new_pose;
        };

    auto impedance_control_callback = [&model, k_gains, d_gains](
                                          const franka::RobotState& state,
                                          franka::Duration /*period*/) -> franka::Torques 
    {
      // Read current coriolis and gravity terms from model.
      std::array<double, 7> coriolis = model.coriolis(state);
      std::array<double, 7> gravity = model.gravity(state);

      // Compute torque command from joint impedance control law
      // Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d with one
      // time step delay.
      // 不加重力项，因为 libfranka 会自动补偿
      std::array<double, 7> tau_d_calculated;
      for (size_t i = 0; i < 7; i++) {
        tau_d_calculated[i] = k_gains[i] * (state.q_d[i] - state.q[i]) -
                              d_gains[i] * state.dq[i] + coriolis[i];
      }

      // 直接返回计算的力矩，不使用 limitRate
      return tau_d_calculated;
    };

    // Execute joint impedance control following the circle trajectory via internal IK.
    // 等价于我想用力矩控制来驱动机器人，但目标是笛卡尔位姿轨迹描述的。
    // （libfranka）帮着把笛卡尔目标做内部 IK 变成关节目标，然后我用关节阻抗算力矩。”
    robot.control(impedance_control_callback, cartesian_pose_callback);

    std::cout << "\nCircle trajectory motion completed successfully!" << std::endl;
    
  } 
  catch (const franka::Exception& e) 
  {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
