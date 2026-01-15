// Copyright (c) 2024 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <franka/quintic_polynomial.h>

/**
 * @file circle_trajectory.h
 * Generates a trajectory with a linear segment followed by a circular segment in the XY plane.
 */

namespace franka {

/**
 * A single trajectory sample.
 */
struct CircleTrajectoryPoint {
  double time;
  std::array<double, 3> position;
  std::array<double, 3> velocity;
  std::array<double, 3> acceleration;
};

/**
 * A container for the generated trajectory samples.
 */
struct CircleTrajectory {
  std::vector<CircleTrajectoryPoint> points;
};

namespace detail {

inline std::vector<double> makeTimeSamples(double duration, double time_step) {
  std::vector<double> samples;
  if (duration <= 0.0 || time_step <= 0.0) {
    return samples;
  }
  const size_t steps = static_cast<size_t>(std::floor(duration / time_step));
  samples.reserve(steps + 2);
  for (size_t i = 0; i <= steps; ++i) {
    samples.push_back(static_cast<double>(i) * time_step);
  }
  if (samples.empty() || samples.back() < duration) {
    samples.push_back(duration);
  }
  return samples;
}

inline void validateParameters(double radius, double line_time, double circle_time) {
  if (!std::isfinite(radius) || !std::isfinite(line_time) || !std::isfinite(circle_time)) {
    throw std::invalid_argument("Inputs must be finite numbers.");
  }
  if (radius < 0.0) {
    throw std::invalid_argument("Radius must be non-negative.");
  }
  if (line_time <= 0.0 || circle_time <= 0.0) {
    throw std::invalid_argument("Times must be positive.");
  }
}

inline void validateParameters(double radius, double line_time, double circle_time, double time_step) {
  validateParameters(radius, line_time, circle_time);
  if (!std::isfinite(time_step)) {
    throw std::invalid_argument("Inputs must be finite numbers.");
  }
  if (time_step <= 0.0) {
    throw std::invalid_argument("Time step must be positive.");
  }
}

}  // namespace detail

/**
 * Evaluates a linear-to-circular trajectory in the XY plane at a given time.
 *
 * The trajectory matches generateCircleTrajectory but is computed analytically. Time is
 * clamped to [0, line_time + circle_time].
 *
 * @param[in] time Time in seconds.
 * @param[in] radius Circle radius in meters.
 * @param[in] circle_center Center of the circle.
 * @param[in] line_time Duration of the linear segment in seconds.
 * @param[in] circle_time Duration of the circular segment in seconds.
 *
 * @return Trajectory sample with time, position, velocity, and acceleration.
 *
 * @throw std::invalid_argument if inputs are non-finite or non-positive where required.
 */
inline CircleTrajectoryPoint evaluateCircleTrajectory(double time,
                                                      double radius,
                                                      const std::array<double, 3>& circle_center,
                                                      double line_time,
                                                      double circle_time) {
  if (!std::isfinite(time)) {
    throw std::invalid_argument("Time must be a finite number.");
  }
  detail::validateParameters(radius, line_time, circle_time);

  const double total_time = line_time + circle_time;
  double clamped_time = time;
  if (clamped_time < 0.0) {
    clamped_time = 0.0;
  } else if (clamped_time > total_time) {
    clamped_time = total_time;
  }

  CircleTrajectoryPoint sample;
  sample.time = clamped_time;

  const std::array<double, 3> line_delta = {{radius, 0.0, 0.0}};

  if (clamped_time <= line_time) 
  {
    const double tau = clamped_time / line_time;
    const double s = QuinticPolynomial::calculate(tau);
    const double s_dot = QuinticPolynomial::calculateVelocity(tau) / line_time;
    const double s_ddot = QuinticPolynomial::calculateAcceleration(tau) / (line_time * line_time);

    for (size_t i = 0; i < 3; ++i) {
      sample.position[i] = circle_center[i] + s * line_delta[i];
      sample.velocity[i] = s_dot * line_delta[i];
      sample.acceleration[i] = s_ddot * line_delta[i];
    }
    return sample;
  }

  const double t = clamped_time - line_time;
  const double tau = t / circle_time;
  const double s = QuinticPolynomial::calculate(tau);
  const double s_dot = QuinticPolynomial::calculateVelocity(tau) / circle_time;
  const double s_ddot = QuinticPolynomial::calculateAcceleration(tau) / (circle_time * circle_time);

  constexpr double kPi = 3.14159265358979323846;
  const double theta_range = 2.0 * kPi;
  const double theta = s * theta_range;
  const double theta_dot = s_dot * theta_range;
  const double theta_ddot = s_ddot * theta_range;

  sample.position = {{circle_center[0] + radius * std::cos(theta),
                      circle_center[1] + radius * std::sin(theta), circle_center[2]}};
  sample.velocity = {{-radius * std::sin(theta) * theta_dot,
                      radius * std::cos(theta) * theta_dot, 0.0}};
  sample.acceleration = {{-radius * std::cos(theta) * theta_dot * theta_dot -
                              radius * std::sin(theta) * theta_ddot,
                          -radius * std::sin(theta) * theta_dot * theta_dot +
                              radius * std::cos(theta) * theta_ddot,
                          0.0}};

  return sample;
}

/**
 * Evaluates a linear-to-circular trajectory using the origin as the circle center.
 *
 * @param[in] time Time in seconds.
 * @param[in] radius Circle radius in meters.
 * @param[in] line_time Duration of the linear segment in seconds.
 * @param[in] circle_time Duration of the circular segment in seconds.
 *
 * @return Trajectory sample with time, position, velocity, and acceleration.
 */
inline CircleTrajectoryPoint evaluateCircleTrajectory(double time,
                                                      double radius,
                                                      double line_time,
                                                      double circle_time) {
  return evaluateCircleTrajectory(time, radius, {{0.0, 0.0, 0.0}}, line_time, circle_time);
}

/**
 * Generates a linear-to-circular trajectory in the XY plane.
 *
 * The motion starts at the circle center, moves along +X to the circle edge, then completes a
 * full circle back to the start point. Quintic time-scaling enforces zero velocity and
 * acceleration at the segment boundaries.
 *
 * @param[in] radius Circle radius in meters.
 * @param[in] circle_center Center of the circle.
 * @param[in] line_time Duration of the linear segment in seconds.
 * @param[in] circle_time Duration of the circular segment in seconds.
 * @param[in] time_step Sampling time step in seconds.
 *
 * @return Generated trajectory samples with time, position, velocity, and acceleration.
 *
 * @throw std::invalid_argument if inputs are non-finite or non-positive where required.
 */
inline CircleTrajectory generateCircleTrajectory(double radius,
                                                 const std::array<double, 3>& circle_center,
                                                 double line_time,
                                                 double circle_time,
                                                 double time_step = 0.001) {
  detail::validateParameters(radius, line_time, circle_time, time_step);

  const std::array<double, 3> start_point = circle_center;
  const std::array<double, 3> line_end = {
      {circle_center[0] + radius, circle_center[1], circle_center[2]}};
  const std::array<double, 3> line_delta = {
      {line_end[0] - start_point[0], line_end[1] - start_point[1], line_end[2] - start_point[2]}};

  const auto line_times = detail::makeTimeSamples(line_time, time_step);
  const auto circle_times = detail::makeTimeSamples(circle_time, time_step);

  CircleTrajectory trajectory;
  if (line_times.empty() || circle_times.empty()) {
    return trajectory;
  }
  trajectory.points.reserve(line_times.size() + circle_times.size() - 1);

  for (double t : line_times) {
    const double tau = t / line_time;
    const double s = QuinticPolynomial::calculate(tau);
    const double s_dot = QuinticPolynomial::calculateVelocity(tau) / line_time;
    const double s_ddot = QuinticPolynomial::calculateAcceleration(tau) / (line_time * line_time);

    CircleTrajectoryPoint sample;
    sample.time = t;
    for (size_t i = 0; i < 3; ++i) {
      sample.position[i] = start_point[i] + s * line_delta[i];
      sample.velocity[i] = s_dot * line_delta[i];
      sample.acceleration[i] = s_ddot * line_delta[i];
    }
    trajectory.points.push_back(sample);
  }

  constexpr double kPi = 3.14159265358979323846;
  const double theta_range = 2.0 * kPi;

  for (size_t idx = 1; idx < circle_times.size(); ++idx) {
    const double t = circle_times[idx];
    const double tau = t / circle_time;
    const double s = QuinticPolynomial::calculate(tau);
    const double s_dot = QuinticPolynomial::calculateVelocity(tau) / circle_time;
    const double s_ddot =
        QuinticPolynomial::calculateAcceleration(tau) / (circle_time * circle_time);

    const double theta = s * theta_range;
    const double theta_dot = s_dot * theta_range;
    const double theta_ddot = s_ddot * theta_range;

    CircleTrajectoryPoint sample;
    sample.time = line_time + t;
    sample.position = {{circle_center[0] + radius * std::cos(theta),
                        circle_center[1] + radius * std::sin(theta), circle_center[2]}};
    sample.velocity = {{-radius * std::sin(theta) * theta_dot,
                        radius * std::cos(theta) * theta_dot, 0.0}};
    sample.acceleration = {{-radius * std::cos(theta) * theta_dot * theta_dot -
                                radius * std::sin(theta) * theta_ddot,
                            -radius * std::sin(theta) * theta_dot * theta_dot +
                                radius * std::cos(theta) * theta_ddot,
                            0.0}};

    trajectory.points.push_back(sample);
  }

  return trajectory;
}

/**
 * Generates a linear-to-circular trajectory using the origin as the circle center.
 *
 * @param[in] radius Circle radius in meters.
 * @param[in] line_time Duration of the linear segment in seconds.
 * @param[in] circle_time Duration of the circular segment in seconds.
 * @param[in] time_step Sampling time step in seconds.
 *
 * @return Generated trajectory samples with time, position, velocity, and acceleration.
 */
inline CircleTrajectory generateCircleTrajectory(double radius,
                                                 double line_time,
                                                 double circle_time,
                                                 double time_step = 0.001) {
  return generateCircleTrajectory(radius, {{0.0, 0.0, 0.0}}, line_time, circle_time, time_step);
}

}  // namespace franka
