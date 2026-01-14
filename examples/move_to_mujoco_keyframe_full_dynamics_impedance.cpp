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
#include <franka/quintic_polynomial.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example move_to_mujoco_keyframe_full_dynamics_impedance.cpp
 * Move the robot from q_start to q_end using quintic polynomial planning in joint space.
 * Uses full dynamics feedforward + joint impedance, and records trajectory data.
 *
 * @warning Before executing this example, make sure there is enough space around the robot.
 */

// Data record structure for each time step
struct DataRecord {
  double time;
  std::array<double, 7> q_desired;
  std::array<double, 7> q_actual;
  std::array<double, 7> dq_desired;
  std::array<double, 7> dq_actual;
  std::array<double, 7> tau_J;            // Measured joint torques
  std::array<double, 7> tau_cmd;          // Commanded joint torques
  std::array<double, 7> tau_impedance;    // Impedance control torques
  std::array<double, 7> tau_feedforward;  // Feedforward torques (M*ddq + C + g)
};

std::array<double, 7> multiplyMassMatrix(const std::array<double, 49>& mass,
                                         const std::array<double, 7>& vector) {
  std::array<double, 7> result{};
  for (size_t i = 0; i < 7; i++) {
    result[i] = 0.0;
    for (size_t j = 0; j < 7; j++) {
      result[i] += mass[i + 7 * j] * vector[j];
    }
  }
  return result;
}

int main(int argc, char** argv) {
  if (argc > 2) {
    std::cerr << "Usage: " << argv[0] << " [motion-time]" << std::endl
              << "  motion-time: duration in seconds (default: 10.0)" << std::endl;
    return -1;
  }

  try {
    const std::string robot_ip = "172.16.1.2";

    // Start and end joint configurations
    std::array<double, 7> q_start = {{0.0, 0.0, 0.0, -1.57, 0.0, 1.57, -0.785}};
    std::array<double, 7> q_end = {{-1, -0.35, -0.84, -2, -1, 2, 0.23}};

    double motion_time = 5.0;
    if (argc == 2) {
      const std::string motion_time_arg(argv[1]);
      try {
        size_t parsed = 0;
        motion_time = std::stod(motion_time_arg, &parsed);
        if (parsed != motion_time_arg.size()) {
          std::cerr << "Error: motion time is not a valid number: " << motion_time_arg
                    << std::endl;
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

    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    std::cout << "\nMoving to start position..." << std::endl;
    MotionGenerator motion_to_start(0.5, q_start);
    robot.control(motion_to_start);
    std::cout << "Reached start position." << std::endl;

    std::cout << "\nWARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    franka::Model model = robot.loadModel();

    const std::array<double, 7> k_gains = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};

    franka::QuinticPolynomial quintic(motion_time);
    quintic.reset();

    const size_t max_samples = static_cast<size_t>(motion_time * 1000) + 100;
    std::vector<DataRecord> recorded_data(max_samples);
    size_t sample_index = 0;

    std::cout << "Starting quintic polynomial motion with full dynamics impedance..." << std::endl;
    std::cout << "Recording data during motion..." << std::endl;

    robot.control([&](const franka::RobotState& state,
      franka::Duration period) -> franka::Torques {
      double dt = period.toSec();
      double s = quintic.step(dt);
      double time = quintic.getTime();
      double tau = time / motion_time;

      double s_dot = franka::QuinticPolynomial::calculateVelocity(tau) / motion_time;
      double s_ddot = franka::QuinticPolynomial::calculateAcceleration(tau) /
            (motion_time * motion_time);

      std::array<double, 7> q_desired;
      std::array<double, 7> dq_desired;
      std::array<double, 7> ddq_desired;
      for (size_t i = 0; i < 7; i++) {
      double delta = q_end[i] - q_start[i];
      q_desired[i] = q_start[i] + s * delta;
      dq_desired[i] = s_dot * delta;
      ddq_desired[i] = s_ddot * delta;
      }

      std::array<double, 49> mass = model.mass(state);
      std::array<double, 7> coriolis = model.coriolis(state);
      std::array<double, 7> gravity = model.gravity(state);

      std::array<double, 7> tau_ff = multiplyMassMatrix(mass, ddq_desired);
      for (size_t i = 0; i < 7; i++) {
        tau_ff[i] += coriolis[i] + gravity[i];
      }

      std::array<double, 7> tau_impedance;
      for (size_t i = 0; i < 7; i++) {
        tau_impedance[i] = k_gains[i] * (q_desired[i] - state.q[i]) +
                           d_gains[i] * (dq_desired[i] - state.dq[i]);
      }

      std::array<double, 7> tau_cmd;
      for (size_t i = 0; i < 7; i++) {
        tau_cmd[i] = tau_ff[i] + tau_impedance[i];
      }

      // --------------------------------------------------------------------------
      // FIX: Handle Rate Limiting Initialization Correctly
      // --------------------------------------------------------------------------
      std::array<double, 7> tau_d_rate_limited;
      
      if (sample_index == 0) {
          // 第一帧：使用当前测量到的力矩 (state.tau_J) 作为参考基准。
          // 这样 franka::limitRate 会计算从“实际测量值”到“目标计算值(tau_cmd)”的合法变化量。
          // 这既避免了从0开始的掉臂，也避免了如果模型与实际有偏差时产生的力矩突变。
          tau_d_rate_limited = franka::limitRate(franka::kMaxTorqueRate, tau_cmd, state.tau_J);
      } else {
          // 后续帧：使用上一帧的指令力矩 (state.tau_J_d) 作为参考基准。
          tau_d_rate_limited = franka::limitRate(franka::kMaxTorqueRate, tau_cmd, state.tau_J_d);
      }
      
      franka::Torques command(tau_d_rate_limited);

      if (sample_index < max_samples) {
        recorded_data[sample_index].time = time;
        recorded_data[sample_index].q_desired = q_desired;
        recorded_data[sample_index].q_actual = state.q;
        recorded_data[sample_index].dq_desired = dq_desired;
        recorded_data[sample_index].dq_actual = state.dq;
        recorded_data[sample_index].tau_J = state.tau_J;
        recorded_data[sample_index].tau_cmd = tau_d_rate_limited;
        recorded_data[sample_index].tau_impedance = tau_impedance;
        recorded_data[sample_index].tau_feedforward = tau_ff;
        sample_index++;
      }

      if (time >= motion_time) {
        return franka::MotionFinished(command);
      }

      return command;
    });

    recorded_data.resize(sample_index);

    std::cout << "Motion completed successfully!" << std::endl;
    std::cout << "Recorded " << recorded_data.size() << " data points." << std::endl;

    std::string source_path = __FILE__;
    std::string source_dir = source_path.substr(0, source_path.find_last_of("/\\"));
    std::string parent_dir = source_dir.substr(0, source_dir.find_last_of("/\\"));
    std::string data_dir = parent_dir + "/data/move_to_mujoco_keyframe_full_dynamics_impedance_record";
    std::string csv_filename = data_dir + "/trajectory_data.csv";

    std::string mkdir_cmd = "mkdir -p " + data_dir;
    std::system(mkdir_cmd.c_str());

    std::cout << "Saving data to: " << csv_filename << std::endl;

    std::ofstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
      std::cerr << "Error: Could not open file " << csv_filename << " for writing." << std::endl;
      return -1;
    }

    csv_file << "time";
    for (int j = 1; j <= 7; j++) csv_file << ",q_desired_" << j;
    for (int j = 1; j <= 7; j++) csv_file << ",q_actual_" << j;
    for (int j = 1; j <= 7; j++) csv_file << ",dq_desired_" << j;
    for (int j = 1; j <= 7; j++) csv_file << ",dq_actual_" << j;
    for (int j = 1; j <= 7; j++) csv_file << ",tau_J_" << j;
    for (int j = 1; j <= 7; j++) csv_file << ",tau_cmd_" << j;
    for (int j = 1; j <= 7; j++) csv_file << ",tau_impedance_" << j;
    for (int j = 1; j <= 7; j++) csv_file << ",tau_feedforward_" << j;
    csv_file << "\n";

    csv_file << std::fixed << std::setprecision(6);
    for (const auto& record : recorded_data) {
      csv_file << record.time;
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.q_desired[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.q_actual[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.dq_desired[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.dq_actual[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.tau_J[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.tau_cmd[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.tau_impedance[i];
      for (size_t i = 0; i < 7; i++) csv_file << "," << record.tau_feedforward[i];
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
