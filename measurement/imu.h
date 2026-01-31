#ifndef MEMS_INS_GPS_IMU_H
#define MEMS_INS_GPS_IMU_H
#include <Eigen/Core>

struct ImuData
{
    Eigen::Vector3d gyro; //角速度增量,rad
    Eigen::Vector3d accel; // 加速度计增量 ,m/s
    double time;           // 时间点
};

#endif //MEMS_INS_GPS_IMU_H