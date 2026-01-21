#include "CoordinateTransform.h"
#include <cmath>
#include <algorithm>

CoordinateTransform::CoordinateTransform() {
    // 初始化临时变量为零
    temp_quat_.fill(0.0);
    temp_quat_[0] = 1.0; // w分量初始化为1
    
    temp_eul_.fill(0.0);
    
    for (auto& row : temp_rot_) {
        row.fill(0.0);
    }
    // 初始化为单位矩阵
    temp_rot_[0][0] = temp_rot_[1][1] = temp_rot_[2][2] = 1.0;
}

void CoordinateTransform::quat2rot(const Quaternion& quat, RotationMatrix& rot) {
    // 自动归一化输入四元数
    temp_quat_ = quat;
    normalizeQuaternion(temp_quat_);
    
    const double w = temp_quat_[0];
    const double x = temp_quat_[1];
    const double y = temp_quat_[2];
    const double z = temp_quat_[3];
    
    // 预计算常用项
    const double x2 = x * x;
    const double y2 = y * y;
    const double z2 = z * z;
    const double xy = x * y;
    const double xz = x * z;
    const double yz = y * z;
    const double wx = w * x;
    const double wy = w * y;
    const double wz = w * z;
    
    // 计算旋转矩阵元素
    rot[0][0] = 1.0 - 2.0 * (y2 + z2);
    rot[0][1] = 2.0 * (xy - wz);
    rot[0][2] = 2.0 * (xz + wy);
    
    rot[1][0] = 2.0 * (xy + wz);
    rot[1][1] = 1.0 - 2.0 * (x2 + z2);
    rot[1][2] = 2.0 * (yz - wx);
    
    rot[2][0] = 2.0 * (xz - wy);
    rot[2][1] = 2.0 * (yz + wx);
    rot[2][2] = 1.0 - 2.0 * (x2 + y2);
}

void CoordinateTransform::rot2quat(const RotationMatrix& rot, Quaternion& quat) {
    const double trace = rot[0][0] + rot[1][1] + rot[2][2];
    
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0; // s = 4 * qw
        quat[0] = 0.25 * s;                            // w
        quat[1] = (rot[2][1] - rot[1][2]) / s;         // x
        quat[2] = (rot[0][2] - rot[2][0]) / s;         // y
        quat[3] = (rot[1][0] - rot[0][1]) / s;         // z
    } else if (rot[0][0] > rot[1][1] && rot[0][0] > rot[2][2]) {
        const double s = std::sqrt(1.0 + rot[0][0] - rot[1][1] - rot[2][2]) * 2.0; // s = 4 * qx
        quat[0] = (rot[2][1] - rot[1][2]) / s;         // w
        quat[1] = 0.25 * s;                            // x
        quat[2] = (rot[0][1] + rot[1][0]) / s;         // y
        quat[3] = (rot[0][2] + rot[2][0]) / s;         // z
    } else if (rot[1][1] > rot[2][2]) {
        const double s = std::sqrt(1.0 + rot[1][1] - rot[0][0] - rot[2][2]) * 2.0; // s = 4 * qy
        quat[0] = (rot[0][2] - rot[2][0]) / s;         // w
        quat[1] = (rot[0][1] + rot[1][0]) / s;         // x
        quat[2] = 0.25 * s;                            // y
        quat[3] = (rot[1][2] + rot[2][1]) / s;         // z
    } else {
        const double s = std::sqrt(1.0 + rot[2][2] - rot[0][0] - rot[1][1]) * 2.0; // s = 4 * qz
        quat[0] = (rot[1][0] - rot[0][1]) / s;         // w
        quat[1] = (rot[0][2] + rot[2][0]) / s;         // x
        quat[2] = (rot[1][2] + rot[2][1]) / s;         // y
        quat[3] = 0.25 * s;                            // z
    }
}

void CoordinateTransform::quat2Eul(const Quaternion& quat, EulerAngles& eul) {
    // 自动归一化输入四元数
    temp_quat_ = quat;
    normalizeQuaternion(temp_quat_);
    
    const double w = temp_quat_[0];
    const double x = temp_quat_[1];
    const double y = temp_quat_[2];
    const double z = temp_quat_[3];
    
    // 预计算常用项
    const double w2 = w * w;
    const double x2 = x * x;
    const double y2 = y * y;
    const double z2 = z * z;
    
    // ZYX欧拉角转换
    // Roll (x-axis rotation)
    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x2 + y2);
    eul[2] = std::atan2(sinr_cosp, cosr_cosp); // x角度
    
    // Pitch (y-axis rotation)
    const double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
        eul[1] = std::copysign(HALF_PI, sinp); // 万向节锁情况
    } else {
        eul[1] = std::asin(sinp); // y角度
    }
    
    // Yaw (z-axis rotation)
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y2 + z2);
    eul[0] = std::atan2(siny_cosp, cosy_cosp); // z角度
}

void CoordinateTransform::Eul2quat(const EulerAngles& eul, Quaternion& quat) {
    const double z = eul[0] * 0.5; // yaw / 2
    const double y = eul[1] * 0.5; // pitch / 2
    const double x = eul[2] * 0.5; // roll / 2
    
    const double cz = std::cos(z);
    const double sz = std::sin(z);
    const double cy = std::cos(y);
    const double sy = std::sin(y);
    const double cx = std::cos(x);
    const double sx = std::sin(x);
    
    // ZYX顺序的四元数乘积
    quat[0] = cz * cy * cx + sz * sy * sx; // w
    quat[1] = cz * cy * sx - sz * sy * cx; // x
    quat[2] = cz * sy * cx + sz * cy * sx; // y
    quat[3] = sz * cy * cx - cz * sy * sx; // z
}

void CoordinateTransform::rot2Eul(const RotationMatrix& rot, EulerAngles& eul) {
    // ZYX欧拉角从旋转矩阵提取
    // Roll (x-axis rotation)
    eul[2] = std::atan2(rot[2][1], rot[2][2]);
    
    // Pitch (y-axis rotation)  
    const double sinp = -rot[2][0];
    if (std::abs(sinp) >= 1.0) {
        eul[1] = std::copysign(HALF_PI, sinp); // 万向节锁情况
    } else {
        eul[1] = std::asin(sinp);
    }
    
    // Yaw (z-axis rotation)
    eul[0] = std::atan2(rot[1][0], rot[0][0]);
}

void CoordinateTransform::Eul2rot(const EulerAngles& eul, RotationMatrix& rot) {
    const double z = eul[0]; // yaw
    const double y = eul[1]; // pitch
    const double x = eul[2]; // roll
    
    const double cz = std::cos(z);
    const double sz = std::sin(z);
    const double cy = std::cos(y);
    const double sy = std::sin(y);
    const double cx = std::cos(x);
    const double sx = std::sin(x);
    
    // ZYX欧拉角旋转矩阵 = Rz * Ry * Rx
    rot[0][0] = cz * cy;
    rot[0][1] = cz * sy * sx - sz * cx;
    rot[0][2] = cz * sy * cx + sz * sx;
    
    rot[1][0] = sz * cy;
    rot[1][1] = sz * sy * sx + cz * cx;
    rot[1][2] = sz * sy * cx - cz * sx;
    
    rot[2][0] = -sy;
    rot[2][1] = cy * sx;
    rot[2][2] = cy * cx;
}

inline void CoordinateTransform::normalizeQuaternion(Quaternion& quat) {
    const double norm = std::sqrt(quat[0] * quat[0] + quat[1] * quat[1] + 
                                  quat[2] * quat[2] + quat[3] * quat[3]);
    
    if (norm > EPSILON) {
        const double inv_norm = 1.0 / norm;
        quat[0] *= inv_norm;
        quat[1] *= inv_norm;
        quat[2] *= inv_norm;
        quat[3] *= inv_norm;
    } else {
        // 如果范数太小，设置为单位四元数
        quat[0] = 1.0;
        quat[1] = quat[2] = quat[3] = 0.0;
    }
}

inline double CoordinateTransform::normalizeAngle(double angle) {
    while (angle > PI) {
        angle -= TWO_PI;
    }
    while (angle < -PI) {
        angle += TWO_PI;
    }
    return angle;
}
