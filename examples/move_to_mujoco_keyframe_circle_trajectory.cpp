// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include <franka/circle_trajectory.h>

/**
 * @example move_to_mujoco_keyframe_circle_trajectory.cpp
 * Generate a circular trajectory in the XY plane and save to CSV file.
 * Uses franka::generateCircleTrajectory to generate the trajectory.
 * 
 * The motion consists of two phases:
 * 1. Linear segment: Move from circle center to edge along +X
 * 2. Circular segment: Complete a full circle in the XY plane
 *
 * Outputs: position, velocity, and acceleration data to CSV file.
 */


/*
Usage:
# 使用默认参数 (半径=0.1m, 线性段=10s, 圆弧段=20s)
./examples/move_to_mujoco_keyframe_circle_trajectory

# 自定义半径 (例如 0.08m)
./examples/move_to_mujoco_keyframe_circle_trajectory 0.08

# 自定义半径和线性段时间
./examples/move_to_mujoco_keyframe_circle_trajectory 0.08 8.0

# 自定义所有参数
./examples/move_to_mujoco_keyframe_circle_trajectory 0.08 8.0 16.0
*/

int main(int argc, char** argv) {
  // Check arguments
  if (argc > 4) {
    std::cerr << "Usage: " << argv[0] << " [radius] [line-time] [circle-time]" << std::endl
              << "  radius: circle radius in meters (default: 0.1)" << std::endl
              << "  line-time: duration of linear segment in seconds (default: 10.0)" << std::endl
              << "  circle-time: duration of circular segment in seconds (default: 20.0)" << std::endl;
    return -1;
  }

  try {
    // Circle trajectory parameters
    double radius = 0.1;       // Default 10cm radius
    double line_time = 10.0;   // Default 10 seconds for linear segment
    double circle_time = 20.0; // Default 20 seconds for circular segment
    
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

    std::cout << "=== Circle Trajectory Generation Parameters ===" << std::endl;
    std::cout << "Radius: " << radius << " m" << std::endl;
    std::cout << "Linear segment time: " << line_time << " s" << std::endl;
    std::cout << "Circular segment time: " << circle_time << " s" << std::endl;
    std::cout << "Total motion time: " << (line_time + circle_time) << " s" << std::endl;

    // Generate circle trajectory
    // Time step matches robot control frequency (1kHz)
    constexpr double kTimeStep = 0.001;
    franka::CircleTrajectory circle_trajectory = 
        franka::generateCircleTrajectory(radius, line_time, circle_time, kTimeStep);
    
    std::cout << "Generated " << circle_trajectory.points.size() << " trajectory points" << std::endl;

    // Prepare output directory and file
    std::string source_path = __FILE__;
    std::string source_dir = source_path.substr(0, source_path.find_last_of("/\\"));
    std::string parent_dir = source_dir.substr(0, source_dir.find_last_of("/\\"));
    std::string data_dir = parent_dir + "/data/move_to_mujoco_keyframe_circle_record";
    std::string csv_filename = data_dir + "/trajectory_data.csv";

    std::string mkdir_cmd = "mkdir -p " + data_dir;
    std::system(mkdir_cmd.c_str());

    std::cout << "Saving trajectory to: " << csv_filename << std::endl;

    // Open CSV file for writing
    std::ofstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
      std::cerr << "Error: Could not open file " << csv_filename << " for writing." << std::endl;
      return -1;
    }

    // Write CSV header
    csv_file << "time,x,y,z,vx,vy,vz,ax,ay,az\n";

    // Write trajectory data
    csv_file << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < circle_trajectory.points.size(); i++) {
      const auto& point = circle_trajectory.points[i];
      double time = i * kTimeStep;
      
      csv_file << time << ","
               << point.position[0] << "," << point.position[1] << "," << point.position[2] << ","
               << point.velocity[0] << "," << point.velocity[1] << "," << point.velocity[2] << ","
               << point.acceleration[0] << "," << point.acceleration[1] << "," << point.acceleration[2]
               << "\n";
    }

    csv_file.close();
    std::cout << "Trajectory data saved successfully!" << std::endl;
    std::cout << "Total points: " << circle_trajectory.points.size() << std::endl;
    std::cout << "File: " << csv_filename << std::endl;
    
  } 
  catch (const std::exception& e) 
  {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}
