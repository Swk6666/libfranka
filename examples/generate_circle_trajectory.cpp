// Copyright (c) 2024 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include <franka/circle_trajectory.h>

namespace {

bool parseDouble(const char* arg, double* output) {
  try {
    const std::string text(arg);
    size_t parsed = 0;
    const double value = std::stod(text, &parsed);
    if (parsed != text.size() || !std::isfinite(value)) {
      return false;
    }
    *output = value;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

void printUsage(const char* name) {
  std::cerr << "Usage: " << name
            << " [radius] [line-time] [circle-time] [time-step]\n"
            << "Defaults: radius=0.2, line-time=2.0, circle-time=4.0, time-step=0.001\n";
}

}  // namespace

/**
 * @example generate_circle_trajectory.cpp
 * Generates a line-then-circle trajectory and saves it to a CSV file.
 */
int main(int argc, char** argv) {
  if (argc > 5) {
    printUsage(argv[0]);
    return -1;
  }

  double radius = 0.2;
  double line_time = 2.0;
  double circle_time = 4.0;
  double time_step = 0.001;

  if (argc >= 2 && !parseDouble(argv[1], &radius)) {
    std::cerr << "Error: radius is not a valid number.\n";
    return -1;
  }
  if (argc >= 3 && !parseDouble(argv[2], &line_time)) {
    std::cerr << "Error: line-time is not a valid number.\n";
    return -1;
  }
  if (argc >= 4 && !parseDouble(argv[3], &circle_time)) {
    std::cerr << "Error: circle-time is not a valid number.\n";
    return -1;
  }
  if (argc >= 5 && !parseDouble(argv[4], &time_step)) {
    std::cerr << "Error: time-step is not a valid number.\n";
    return -1;
  }

  if (radius < 0.0 || line_time <= 0.0 || circle_time <= 0.0 || time_step <= 0.0) {
    std::cerr << "Error: radius must be non-negative and times must be positive.\n";
    return -1;
  }

  franka::CircleTrajectory trajectory;
  try {
    trajectory = franka::generateCircleTrajectory(radius, line_time, circle_time, time_step);
  } catch (const std::exception& e) {
    std::cerr << "Error generating trajectory: " << e.what() << "\n";
    return -1;
  }

  // 保存数据到CSV文件 - 保存到源文件上级目录的 data 文件夹下
  // __FILE__ 是当前源文件的路径，从中提取目录
  std::string source_path = __FILE__;  // 例如: /home/swk/libfranka/examples/xxx.cpp
  std::string source_dir = source_path.substr(0, source_path.find_last_of("/\\"));  // examples 目录
  std::string parent_dir = source_dir.substr(0, source_dir.find_last_of("/\\"));    // libfranka 目录
  std::string data_dir = parent_dir + "/data/circle_trajectory";
  std::string csv_filename = data_dir + "/circle_trajectory.csv";
  
  // 尝试创建 data 目录（如果不存在）
  std::string mkdir_cmd = "mkdir -p " + data_dir;
  system(mkdir_cmd.c_str());
  
  std::cout << "Saving data to: " << csv_filename << std::endl;
  
  std::ofstream csv_file(csv_filename);
  if (!csv_file.is_open()) {
    std::cerr << "Error: could not open file " << csv_filename << " for writing.\n";
    return -1;
  }

  csv_file << "time,px,py,pz,vx,vy,vz,ax,ay,az\n";
  csv_file << std::fixed << std::setprecision(6);
  for (const auto& sample : trajectory.points) {
    csv_file << sample.time << "," << sample.position[0] << "," << sample.position[1] << ","
             << sample.position[2] << "," << sample.velocity[0] << "," << sample.velocity[1]
             << "," << sample.velocity[2] << "," << sample.acceleration[0] << ","
             << sample.acceleration[1] << "," << sample.acceleration[2] << "\n";
  }

  csv_file.close();
  std::cout << "Saved " << trajectory.points.size() << " samples to " << csv_filename << "\n";
  return 0;
}
