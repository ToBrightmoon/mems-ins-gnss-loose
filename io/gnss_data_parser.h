#ifndef MEMS_INS_GPS_GNSS_DATA_PARSER_H
#define MEMS_INS_GPS_GNSS_DATA_PARSER_H

#include <sstream>
#include <string>
#include "measurement/gnss.h"
#include "utils/constants.h"   // Deg2Rad

inline bool parseGnssLine(const std::string& line, GnssData& gnss)
{
    std::stringstream ss(line);

    double lat_deg = 0.0;
    double lon_deg = 0.0;

    if (!(ss >> gnss.time
              >> lat_deg
              >> lon_deg
              >> gnss.pos.z()
              >> gnss.vel.x()
              >> gnss.vel.y()
              >> gnss.vel.z()))
    {
        return false;
    }

    gnss.pos.x() = lat_deg * Deg2Rad;
    gnss.pos.y() = lon_deg * Deg2Rad;
    gnss.valid = true;

    return true;
}

#endif //MEMS_INS_GPS_GNSS_DATA_PARSER_H