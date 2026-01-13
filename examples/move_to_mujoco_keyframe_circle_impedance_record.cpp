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
#include <vector>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/circle_trajectory.h>

#include "examples_common.h"

/**
 * @example move_to_mujoco_keyframe_circle_impedance_record.cpp
 * Move the robot end-effector along a circular trajectory in the XY plane.
 * Uses franka::generateCircleTrajectory to generate the trajectory.
 * Tracks the Cartesian trajectory using joint impedance control with gravity compensation.
 * Records trajectory data to CSV file.
 * 
 * The motion consists of two phases:
 * 1. Linear segment: Move from circle center to edge along +X
 * 2. Circular segment: Complete a full circle in the XY plane
 *
 * @warning Before executing this example, make sure there is enough space around the robot.
 */


 /*
# 使用默认参数 (半径=0.1m, 线性段=4s, 圆弧段=10s)
./examples/move_to_mujoco_keyframe_circle_impedance_record

# 自定义半径 (例如 0.08m)
./examples/move_to_mujoco_keyframe_circle_impedance_record 0.08

# 自定义半径和线性段时间
./examples/move_to_mujoco_keyframe_circle_impedance_record 0.08 3.0

# 自定义所有参数
./examples/move_to_mujoco_keyframe_circle_impedance_record 0.08 3.0 8.0
 */

// Data record structure for each time step
struct DataRecord {
  double time;                              // Timestamp
  std::array<double, 3> ee_desired;         // Desired end-effector position (x, y, z)
  std::array<double, 3> ee_actual;          // Actual end-effector position (x, y, z)
  std::array<double, 7> q_desired;          // Desired joint angles (from IK)
  std::array<double, 7> q_actual;           // Actual joint angles
  std::array<double, 7> dq_actual;          // Actual joint velocities
  std::array<double, 7> tau_J;              // Measured joint torques (actual)
  std::array<double, 7> tau_cmd;            // Commanded joint torques (input to robot)
  std::array<double, 7> tau_impedance;      // Impedance control torques (k*(q_d-q) - d*dq)
  std::array<double, 7> tau_coriolis;       // Coriolis torques
  std::array<double, 7> tau_gravity;        // Gravity compensation torques
};

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
    double radius = 0.1;      // Default 10cm radius
    double line_time = 4.0;    // Default 4 seconds for linear segment
    double circle_time = 10.0;  // Default 10 seconds for circular segment
    
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

    double total_time = line_time + circle_time;

    std::cout << "=== Circle Trajectory Parameters ===" << std::endl;
    std::cout << "Radius: " << radius << " m" << std::endl;
    std::cout << "Linear segment time: " << line_time << " s" << std::endl;
    std::cout << "Circular segment time: " << circle_time << " s" << std::endl;
    std::cout << "Total motion time: " << total_time << " s" << std::endl;

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
    std::array<double, 3> current_ee_desired = {{0.0, 0.0, 0.0}};  // Current desired EE position

    // size_t 在c++中是无符号整型，专门用来表示大小，数量，索引
    size_t trajectory_index = 0;

    bool initialized = false;
    double time = 0.0;

    // Data recording container - pre-allocate to avoid dynamic memory allocation in real-time loop
    // 1kHz sampling rate, pre-allocate enough space
    const size_t max_samples = static_cast<size_t>(total_time * 1000) + 100;
    std::vector<DataRecord> recorded_data(max_samples);
    size_t sample_index = 0;

    std::cout << "Starting circle trajectory motion..." << std::endl;
    std::cout << "Phase 1: Linear segment (center -> edge)" << std::endl;
    std::cout << "Recording data during motion..." << std::endl;

    auto cartesian_pose_callback =
        [&circle_trajectory, &initial_pose, &last_pose, &current_ee_desired, &trajectory_index, &initialized, &time](
            const franka::RobotState& robot_state,
            franka::Duration period) -> franka::CartesianPose 
        {
          // Accumulate time
          time += period.toSec();

          // Initialize: record the initial end-effector pose
          if (!initialized) {
            initialized = true;
            initial_pose = robot_state.O_T_EE_c;
            last_pose = initial_pose;
            current_ee_desired = {{initial_pose[12], initial_pose[13], initial_pose[14]}};
            std::cout << "Initial EE position: [" << initial_pose[12] << ", "
                      << initial_pose[13] << ", " << initial_pose[14] << "]" << std::endl;
          }

          // Check if trajectory is complete
          if (trajectory_index >= circle_trajectory.points.size()) {
            std::cout << std::endl << "Circle trajectory completed!" << std::endl;
            return franka::MotionFinished(franka::CartesianPose(last_pose));
          }

          // Get current trajectory point
          const auto& point = circle_trajectory.points[trajectory_index];

          // Apply trajectory offset to initial pose
          // The circle trajectory is in the XY plane (robot base frame)
          std::array<double, 16> new_pose = initial_pose;
          new_pose[12] += point.position[0];  // X offset
          new_pose[13] += point.position[1];  // Y offset
          new_pose[14] += point.position[2];  // Z offset (should be 0 for XY plane circle)

          // Update current desired EE position for recording
          current_ee_desired = {{new_pose[12], new_pose[13], new_pose[14]}};

          last_pose = new_pose;
          trajectory_index++;

          return new_pose;
        };

    auto impedance_control_callback = [&model, &k_gains, &d_gains, &recorded_data, &sample_index, 
                                       max_samples, &time, &current_ee_desired](
                                          const franka::RobotState& state,
                                          franka::Duration /*period*/) -> franka::Torques 
    {
      // Read current coriolis and gravity terms from model.
      std::array<double, 7> coriolis = model.coriolis(state);
      std::array<double, 7> gravity = model.gravity(state);

      // Compute impedance control torque: tau_impedance = k*(q_d - q) - d*dq
      std::array<double, 7> tau_impedance;
      for (size_t i = 0; i < 7; i++) {
        tau_impedance[i] = k_gains[i] * (state.q_d[i] - state.q[i]) - d_gains[i] * state.dq[i];
      }

      // Compute torque command from joint impedance control law with gravity compensation.
      // tau_cmd = tau_impedance + coriolis + gravity
      // Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d with one
      // time step delay.
      std::array<double, 7> tau_d_calculated;
      for (size_t i = 0; i < 7; i++) {
        tau_d_calculated[i] = tau_impedance[i] + coriolis[i] + gravity[i];
      }

      std::array<double, 7> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

      // Record data - write directly to pre-allocated array, no dynamic memory allocation
      if (sample_index < max_samples) {
        recorded_data[sample_index].time = time;
        // End-effector positions
        recorded_data[sample_index].ee_desired = current_ee_desired;  // Desired EE position
        recorded_data[sample_index].ee_actual = {{state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14]}};  // Actual EE position
        // Joint data
        recorded_data[sample_index].q_desired = state.q_d;       // Desired joint angles (from IK)
        recorded_data[sample_index].q_actual = state.q;          // Actual joint angles
        recorded_data[sample_index].dq_actual = state.dq;        // Actual joint velocities
        recorded_data[sample_index].tau_J = state.tau_J;         // Measured joint torques
        recorded_data[sample_index].tau_cmd = tau_d_rate_limited; // Commanded torques (after rate limiting)
        recorded_data[sample_index].tau_impedance = tau_impedance; // Impedance control torques
        recorded_data[sample_index].tau_coriolis = coriolis;     // Coriolis torques
        recorded_data[sample_index].tau_gravity = gravity;       // Gravity compensation torques
        sample_index++;
      }

      return tau_d_rate_limited;
    };


    /* 
    等价于我想用力矩控制来驱动机器人，但目标是笛卡尔位姿轨迹描述的。
    libfranka帮着把笛卡尔目标做内部 IK 变成关节目标，然后我用关节阻抗算力矩。
    */ 

    /* 
    robot.control() 会一直运行，直到：回调函数返回 franka::MotionFinished(...)或者发生异常，
    只有当控制循环完全结束后，robot.control() 才会返回，程序才会继续执行后面的代码：
    */ 

    robot.control(impedance_control_callback, cartesian_pose_callback);

    // Resize vector to actual recorded data size
    recorded_data.resize(sample_index);

    std::cout << "\nCircle trajectory motion completed successfully!" << std::endl;
    std::cout << "Recorded " << recorded_data.size() << " data points." << std::endl;

    // Save data to CSV file
    std::string source_path = __FILE__;  // e.g.: /home/swk/libfranka/examples/xxx.cpp
    std::string source_dir = source_path.substr(0, source_path.find_last_of("/\\"));  // examples directory
    std::string parent_dir = source_dir.substr(0, source_dir.find_last_of("/\\"));    // libfranka directory
    std::string data_dir = parent_dir + "/data/move_to_mujoco_keyframe_circle_record";
    std::string csv_filename = data_dir + "/trajectory_data.csv";

    // Create data directory if it doesn't exist
    std::string mkdir_cmd = "mkdir -p " + data_dir;
    std::system(mkdir_cmd.c_str());

    std::cout << "Saving data to: " << csv_filename << std::endl;

    std::ofstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
      std::cerr << "Error: Could not open file " << csv_filename << " for writing." << std::endl;
      return -1;
    }

    // Write CSV header
    csv_file << "time";
    // End-effector positions (x, y, z)
    csv_file << ",ee_desired_x,ee_desired_y,ee_desired_z";
    csv_file << ",ee_actual_x,ee_actual_y,ee_actual_z";
    // Joint data
    for (int j = 1; j <= 7; j++) csv_file << ",q_desired_" << j;
    for (int j = 1; j <= 7; j++) csv_file << ",q_actual_" << j;
    for (int j = 1; j <= 7; j++) csv_file << ",dq_actual_" << j;
    for (int j = 1; j <= 7; j++) csv_file << ",tau_J_" << j;          // Measured torques
    for (int j = 1; j <= 7; j++) csv_file << ",tau_cmd_" << j;        // Commanded torques
    for (int j = 1; j <= 7; j++) csv_file << ",tau_impedance_" << j;  // Impedance torques
    for (int j = 1; j <= 7; j++) csv_file << ",tau_coriolis_" << j;   // Coriolis torques
    for (int j = 1; j <= 7; j++) csv_file << ",tau_gravity_" << j;    // Gravity torques
    csv_file << "\n";

    // Write data
    csv_file << std::fixed << std::setprecision(6);
    for (const auto& record : recorded_data) {
      csv_file << record.time;
      // End-effector positions
      for (size_t i = 0; i < 3; i++) csv_file << "," << record.ee_desired[i];
      for (size_t i = 0; i < 3; i++) csv_file << "," << record.ee_actual[i];
      // Joint data
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.q_desired[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.q_actual[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.dq_actual[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.tau_J[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.tau_cmd[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.tau_impedance[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.tau_coriolis[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.tau_gravity[i];
      csv_file << "\n";
    }

    csv_file.close();
    std::cout << "Data saved to " << csv_filename << std::endl;
    
  } 
  catch (const franka::Exception& e) 
  {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
