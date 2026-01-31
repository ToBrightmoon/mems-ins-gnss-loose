#ifndef MEMS_INS_GPS_GNSS_H
#define MEMS_INS_GPS_GNSS_H
#include <Eigen/Core>

struct GnssData
{
    Eigen::Vector3d pos; // 纬度，经度，高度
    Eigen::Vector3d vel;
    double time;
    bool valid = true;
};

#endif //MEMS_INS_GPS_GNSS_H