#ifndef MEMS_INS_GPS_IMU_DATA_PARSER_H
#define MEMS_INS_GPS_IMU_DATA_PARSER_H

#include <sstream>
#include <string>
#include "measurement/imu.h"

inline bool parseImuLine(const std::string& line, ImuData& imu)
{
    std::stringstream ss(line);

    if (!(ss >> imu.time
              >> imu.gyro.x()
              >> imu.gyro.y()
              >> imu.gyro.z()
              >> imu.accel.x()
              >> imu.accel.y()
              >> imu.accel.z()))
    {
        return false;
    }

    return true;
}

#endif //MEMS_INS_GPS_IMU_DATA_PARSER_H