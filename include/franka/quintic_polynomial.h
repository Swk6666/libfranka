// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <cmath>
#include <type_traits>

/**
 * @file quintic_polynomial.h
 * Quintic polynomial trajectory generator for smooth motion planning.
 * 五次多项式轨迹生成器，用于平滑运动规划。
 */

namespace franka {

/**
 * Quintic polynomial trajectory generator.
 * 五次多项式轨迹生成器
 *
 * The normalized quintic polynomial: s(τ) = 10τ³ - 15τ⁴ + 6τ⁵, τ∈[0,1]
 * 归一化五次多项式: s(τ) = 10τ³ - 15τ⁴ + 6τ⁵, τ∈[0,1]
 *
 * Boundary conditions:
 * 边界条件:
 *   - Position: s(0) = 0, s(1) = 1
 *   - Velocity: s'(0) = 0, s'(1) = 0
 *   - Acceleration: s''(0) = 0, s''(1) = 0
 *
 * This class supports both scalar and vector (std::array) inputs through
 * function overloading and templates.
 * 此类通过函数重载和模板支持标量和向量（std::array）输入。
 */
class QuinticPolynomial {
 public:
  /**
   * Creates a new QuinticPolynomial instance.
   * 创建新的 QuinticPolynomial 实例。
   *
   * @param[in] motion_time Total motion time in seconds.
   *            运动总时间（秒）
   */
  explicit QuinticPolynomial(double motion_time) : motion_time_(motion_time), time_(0.0) {}

  /**
   * Resets the trajectory to the beginning.
   * 将轨迹重置为初始状态。
   */
  void reset() { time_ = 0.0; }

  /**
   * Gets the current elapsed time.
   * 获取当前已经过的时间。
   *
   * @return Current time in seconds.
   */
  double getTime() const { return time_; }

  /**
   * Gets the total motion time.
   * 获取运动总时间。
   *
   * @return Motion time in seconds.
   */
  double getMotionTime() const { return motion_time_; }

  /**
   * Checks if the motion is finished.
   * 检查运动是否完成。
   *
   * @return True if the motion is finished, false otherwise.
   */
  bool isFinished() const { return time_ >= motion_time_; }

  /**
   * Calculates the normalized polynomial value s(τ).
   * 计算归一化的多项式值 s(τ)。
   *
   * @param[in] tau Normalized time τ∈[0,1].
   *            归一化时间 τ∈[0,1]
   * @return Polynomial value s∈[0,1].
   */
  static double calculate(double tau) {
    // Clamp tau to [0, 1]
    if (tau < 0.0) {
      tau = 0.0;
    } else if (tau > 1.0) {
      tau = 1.0;
    }
    double tau2 = tau * tau;
    double tau3 = tau2 * tau;
    double tau4 = tau2 * tau2;
    double tau5 = tau4 * tau;
    return 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
  }

  /**
   * Calculates the normalized polynomial velocity s'(τ).
   * 计算归一化的多项式速度 s'(τ)。
   *
   * @param[in] tau Normalized time τ∈[0,1].
   * @return Polynomial velocity value.
   */
  static double calculateVelocity(double tau) {
    if (tau < 0.0) {
      tau = 0.0;
    } else if (tau > 1.0) {
      tau = 1.0;
    }
    double tau2 = tau * tau;
    double tau3 = tau2 * tau;
    double tau4 = tau2 * tau2;
    // s'(τ) = 30τ² - 60τ³ + 30τ⁴
    return 30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4;
  }

  /**
   * Calculates the normalized polynomial acceleration s''(τ).
   * 计算归一化的多项式加速度 s''(τ)。
   *
   * @param[in] tau Normalized time τ∈[0,1].
   * @return Polynomial acceleration value.
   */
  static double calculateAcceleration(double tau) {
    if (tau < 0.0) {
      tau = 0.0;
    } else if (tau > 1.0) {
      tau = 1.0;
    }
    double tau2 = tau * tau;
    double tau3 = tau2 * tau;
    // s''(τ) = 60τ - 180τ² + 120τ³
    return 60.0 * tau - 180.0 * tau2 + 120.0 * tau3;
  }

  /**
   * Updates the internal time and returns the current interpolation coefficient.
   * 更新内部时间并返回当前插值系数。
   *
   * @param[in] dt Time increment in seconds.
   *            时间增量（秒）
   * @return Interpolation coefficient s∈[0,1].
   */
  double step(double dt) {
    time_ += dt;
    double tau = time_ / motion_time_;
    return calculate(tau);
  }

  // ============================================================
  // Interpolation methods - Scalar version
  // 插值方法 - 标量版本
  // ============================================================

  /**
   * Interpolates a scalar value from start to end.
   * 将标量值从起点插值到终点。
   *
   * @param[in] start Start value.
   *            起始值
   * @param[in] end End value.
   *            结束值
   * @param[in] dt Time increment in seconds.
   *            时间增量（秒）
   * @return Interpolated value.
   */
  double interpolate(double start, double end, double dt) {
    double s = step(dt);
    return start + s * (end - start);
  }

  /**
   * Gets the interpolated scalar value without updating time.
   * 获取插值后的标量值，不更新时间。
   *
   * @param[in] start Start value.
   * @param[in] end End value.
   * @return Interpolated value at current time.
   */
  double getInterpolatedValue(double start, double end) const {
    double tau = time_ / motion_time_;
    double s = calculate(tau);
    return start + s * (end - start);
  }

  // ============================================================
  // Interpolation methods - Vector (std::array) version
  // 插值方法 - 向量（std::array）版本
  // ============================================================

  /**
   * Interpolates an array of values from start to end.
   * 将值数组从起点插值到终点。
   *
   * @tparam N Size of the array.
   *         数组大小
   * @param[in] start Start values.
   *            起始值数组
   * @param[in] end End values.
   *            结束值数组
   * @param[in] dt Time increment in seconds.
   *            时间增量（秒）
   * @return Interpolated values.
   */
  template <size_t N>
  std::array<double, N> interpolate(const std::array<double, N>& start,
                                    const std::array<double, N>& end,
                                    double dt) {
    double s = step(dt);
    std::array<double, N> result;
    for (size_t i = 0; i < N; i++) {
      result[i] = start[i] + s * (end[i] - start[i]);
    }
    return result;
  }

  /**
   * Gets the interpolated array values without updating time.
   * 获取插值后的数组值，不更新时间。
   *
   * @tparam N Size of the array.
   * @param[in] start Start values.
   * @param[in] end End values.
   * @return Interpolated values at current time.
   */
  template <size_t N>
  std::array<double, N> getInterpolatedValue(const std::array<double, N>& start,
                                             const std::array<double, N>& end) const {
    double tau = time_ / motion_time_;
    double s = calculate(tau);
    std::array<double, N> result;
    for (size_t i = 0; i < N; i++) {
      result[i] = start[i] + s * (end[i] - start[i]);
    }
    return result;
  }

  /**
   * Interpolates values and stores the result in an output array.
   * 插值并将结果存储在输出数组中。
   *
   * @tparam N Size of the array.
   * @param[in] start Start values.
   * @param[in] end End values.
   * @param[in] dt Time increment in seconds.
   * @param[out] output Output array to store the result.
   */
  template <size_t N>
  void interpolate(const std::array<double, N>& start,
                   const std::array<double, N>& end,
                   double dt,
                   std::array<double, N>& output) {
    double s = step(dt);
    for (size_t i = 0; i < N; i++) {
      output[i] = start[i] + s * (end[i] - start[i]);
    }
  }

 private:
  double motion_time_;  ///< Total motion time in seconds. 运动总时间（秒）
  double time_;         ///< Current elapsed time in seconds. 当前已经过时间（秒）
};

}  // namespace franka

