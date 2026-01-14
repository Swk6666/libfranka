cmake_minimum_required(VERSION 3.4)

project(libfranka-examples CXX)

# ==============================================================================
# 新增部分：默认开启 Release 模式 (-O3) 和本地架构优化
# ==============================================================================

# 如果没有指定构建类型，默认设置为 Release (包含 -O3 优化)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "没有指定构建类型，默认为 'Release' 以开启 -O3 优化。")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
  # 设置 cmake-gui 的可选值
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# 针对 Release 模式，额外增加 -march=native 以利用 CPU 指令集优化 (AVX等)
# 这对 1kHz 控制循环非常有用
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -march=native)
endif()

# ==============================================================================

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT FRANKA_IS_FOUND)
    find_package(Franka REQUIRED)
endif()
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  examples_common.cpp
)

target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3)

# 添加本地 include 路径，以便使用尚未安装的新头文件
target_include_directories(examples_common PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../include)

set(EXAMPLES
  cartesian_impedance_control
  communication_test
  echo_robot_state
  force_control
  generate_cartesian_pose_motion
  generate_cartesian_velocity_motion
  generate_circle_trajectory
  generate_consecutive_motions
  generate_elbow_motion
  generate_joint_position_motion
  generate_joint_velocity_motion
  grasp_object
  joint_impedance_control
  joint_point_to_point_motion
  motion_with_control
  move_ee_cartesian
  move_to_mujoco_keyframe_p2p
  move_to_mujoco_keyframe_polynomial
  move_to_mujoco_keyframe_polynomial_record
  move_to_mujoco_keyframe_circle
  move_to_mujoco_keyframe_circle_trajectory
  move_to_mujoco_keyframe_circle_impedance
  move_to_mujoco_keyframe_full_dynamics_impedance
  print_joint_poses
  print_qpos_now
  vacuum_object
)

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} Franka::Franka examples_common Eigen3::Eigen3)
endforeach()

target_link_libraries(joint_impedance_control Threads::Threads)
target_link_libraries(motion_with_control Poco::Foundation)

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)