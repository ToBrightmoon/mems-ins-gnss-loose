#ifndef MEMS_INS_GPS_REF_H
#define MEMS_INS_GPS_REF_H

#include <Eigen/Core>

struct RefData
{
    double week;
    double time;
    Eigen::Vector3d pos; // lat (deg), lon(deg) alt
    Eigen::Vector3d vel; // vn,ve,vd
    Eigen::Vector3d att; // roll,pitch,yaw (deg)
};
#endif //MEMS_INS_GPS_REF_H