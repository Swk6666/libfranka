// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/circle_trajectory.h>

#include "examples_common.h"

/**
 * @example move_to_mujoco_keyframe_circle.cpp
 * Move the robot end-effector along a circular trajectory in the XY plane.
 * Uses franka::generateCircleTrajectory to generate the trajectory.
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
    std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    
    // Circle trajectory parameters
    double radius = 0.1;      // Default 5cm radius
    double line_time = 4.0;    // Default 2 seconds for linear segment
    double circle_time = 6.0;  // Default 4 seconds for circular segment
    
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

    const double total_time = line_time + circle_time;
    std::cout << "Using analytic quintic trajectory (total time: " << total_time << " s)" << std::endl;

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
    MotionGenerator motion_generator(0.3, q_start);
    robot.control(motion_generator);
    std::cout << "Reached initial configuration." << std::endl;

    std::cout << "\nWARNING: This example will move the robot end-effector in a circle! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // Variables for control loop
    std::array<double, 16> initial_pose;

    bool initialized = false;
    double time = 0.0;

    std::cout << "Starting circle trajectory motion..." << std::endl;
    std::cout << "Phase 1: Linear segment (center -> edge)" << std::endl;

    // Execute Cartesian pose control following the circle trajectory
    robot.control([&initial_pose, &initialized, &time, radius, line_time, circle_time, total_time](
                      const franka::RobotState& robot_state,
                      franka::Duration period) -> franka::CartesianPose {
      
      // Initialize: record the initial end-effector pose
      if (!initialized) {
        initialized = true;
        initial_pose = robot_state.O_T_EE_c;
      }

      time += period.toSec();
      franka::CircleTrajectoryPoint point =
          franka::evaluateCircleTrajectory(time, radius, line_time, circle_time);
      

      // Apply trajectory offset to initial pose
      // The circle trajectory is in the XY plane (robot base frame)
      std::array<double, 16> new_pose = initial_pose;
      new_pose[12] += point.position[0];  // X offset
      new_pose[13] += point.position[1];  // Y offset
      new_pose[14] += point.position[2];  // Z offset (should be 0 for XY plane circle)

      if (time >= total_time) {
        std::cout << std::endl << "Circle trajectory completed!" << std::endl;
        return franka::MotionFinished(new_pose);
      }

      return new_pose;
    });

    std::cout << "\nCircle trajectory motion completed successfully!" << std::endl;
    
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
