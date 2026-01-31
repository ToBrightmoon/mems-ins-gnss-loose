#ifndef MEMS_INS_GPS_ROTATION_H
#define MEMS_INS_GPS_ROTATION_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>
#include "utils/logger.h"

inline Eigen::Vector3d matrix2euler(const Eigen::Matrix3d &dcm)
{
    Eigen::Vector3d euler;

    euler[1] = atan(-dcm(2, 0) / sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));

    if (dcm(2, 0) <= -0.999)
    {
        euler[0] = 0;
        euler[2] = atan2((dcm(1, 2) - dcm(0, 1)), (dcm(0, 2) + dcm(1, 1)));
        LOG_WARNING("Rotation::matrix2euler: Singular Euler Angle! Set the roll angle to 0!");
    }
    else if (dcm(2, 0) >= 0.999)
    {
        euler[0] = 0;
        euler[2] = M_PI + atan2((dcm(1, 2) + dcm(0, 1)), (dcm(0, 2) - dcm(1, 1)));
        LOG_WARNING("Rotation::matrix2euler: Singular Euler Angle! Set the roll angle to 0!");
    }
    else
    {
        euler[0] = atan2(dcm(2, 1), dcm(2, 2));
        euler[2] = atan2(dcm(1, 0), dcm(0, 0));
    }

    // heading 0~2PI
    if (euler[2] < 0)
    {
        euler[2] = M_PI * 2 + euler[2];
    }

    return euler;
}

inline Eigen::Vector3d quaternion2euler(const Eigen::Quaterniond &quaternion)
{
    return matrix2euler(quaternion.toRotationMatrix());
}

inline  Eigen::Quaterniond euler2quaternion(const double roll,const double pitch,const double yaw)
{
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

inline Eigen::Vector3d euler2vector(const double roll,const double pitch,const double yaw)
{
    Eigen::AngleAxisd rollX(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchY(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawZ(yaw, Eigen::Vector3d::UnitZ());
    return Sophus::SO3d(yawZ * pitchY * rollX).log();
}

#endif //MEMS_INS_GPS_ROTATION_H
