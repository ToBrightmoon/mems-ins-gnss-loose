#ifndef MEMS_INS_GPS_NAV_RESULT_H
#define MEMS_INS_GPS_NAV_RESULT_H
#include <Eigen/Core>

struct NavResultData
{
    double time;
    Eigen::Vector3d pos;
    Eigen::Vector3d att;
};
#endif