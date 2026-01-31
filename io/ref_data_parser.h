#ifndef MEMS_INS_GPS_REF_DATA_PARSER_H
#define MEMS_INS_GPS_REF_DATA_PARSER_H

#include <sstream>
#include <string>
#include "measurement/ref.h"

inline bool parseRefDataLine(const std::string& line, RefData& ref)
{
    std::stringstream ss(line);

    if (!(ss >> ref.week
              >> ref.time
              >> ref.pos.x()
              >> ref.pos.y()
              >> ref.pos.z()
              >> ref.vel.x()
              >> ref.vel.y()
              >> ref.vel.z()
              >> ref.att.x()
              >> ref.att.y()
              >> ref.att.z()))
    {
        return false;
    }

    return true;
}

#endif //MEMS_INS_GPS_REF_DATA_PARSER_H