#ifndef COORDINATE_TRANSFORM_H
#define COORDINATE_TRANSFORM_H

#include <array>
#include <cmath>

/**
 * 高性能坐标转换类
 * 支持四元数(w,x,y,z)、旋转矩阵(3x3)、欧拉角ZYX(弧度制)之间的转换
 * 使用固定维度数组，在实例化时预分配所有内存，确保最快速度
 */
class CoordinateTransform {
public:
    // 固定维度类型定义
    using Quaternion = std::array<double, 4>;      // [w, x, y, z]
    using RotationMatrix = std::array<std::array<double, 3>, 3>; // 3x3矩阵
    using EulerAngles = std::array<double, 3>;     // [z, y, x] (ZYX顺序，弧度制)
    
    /**
     * 构造函数 - 预分配所有临时变量内存
     */
    CoordinateTransform();
    
    /**
     * 析构函数
     */
    ~CoordinateTransform() = default;
    
    // 禁用拷贝构造和赋值操作符以避免不必要的内存操作
    CoordinateTransform(const CoordinateTransform&) = delete;
    CoordinateTransform& operator=(const CoordinateTransform&) = delete;
    
    /**
     * 四元数转旋转矩阵
     * @param quat 输入四元数 [w, x, y, z] (自动归一化)
     * @param rot 输出旋转矩阵 3x3
     */
    void quat2rot(const Quaternion& quat, RotationMatrix& rot);
    
    /**
     * 旋转矩阵转四元数
     * @param rot 输入旋转矩阵 3x3
     * @param quat 输出四元数 [w, x, y, z]
     */
    void rot2quat(const RotationMatrix& rot, Quaternion& quat);
    
    /**
     * 四元数转欧拉角(ZYX)
     * @param quat 输入四元数 [w, x, y, z] (自动归一化)
     * @param eul 输出欧拉角 [z, y, x] (弧度制)
     */
    void quat2Eul(const Quaternion& quat, EulerAngles& eul);
    
    /**
     * 欧拉角(ZYX)转四元数
     * @param eul 输入欧拉角 [z, y, x] (弧度制)
     * @param quat 输出四元数 [w, x, y, z]
     */
    void Eul2quat(const EulerAngles& eul, Quaternion& quat);
    
    /**
     * 旋转矩阵转欧拉角(ZYX)
     * @param rot 输入旋转矩阵 3x3
     * @param eul 输出欧拉角 [z, y, x] (弧度制)
     */
    void rot2Eul(const RotationMatrix& rot, EulerAngles& eul);
    
    /**
     * 欧拉角(ZYX)转旋转矩阵
     * @param eul 输入欧拉角 [z, y, x] (弧度制)
     * @param rot 输出旋转矩阵 3x3
     */
    void Eul2rot(const EulerAngles& eul, RotationMatrix& rot);
    
private:
    // 预分配的临时变量，避免运行时内存分配
    Quaternion temp_quat_;
    RotationMatrix temp_rot_;
    EulerAngles temp_eul_;
    
    // 常用的数学常量
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double HALF_PI = PI * 0.5;
    static constexpr double TWO_PI = PI * 2.0;
    static constexpr double EPSILON = 1e-15;
    
    /**
     * 四元数归一化（内联函数提高性能）
     * @param quat 需要归一化的四元数
     */
    inline void normalizeQuaternion(Quaternion& quat);
    
    /**
     * 角度规范化到[-π, π]范围
     * @param angle 输入角度
     * @return 规范化后的角度
     */
    inline double normalizeAngle(double angle);
};

#endif // COORDINATE_TRANSFORM_H
