#ifndef MEMS_INS_GPS_NAV_RESULT_PARSER_H
#define MEMS_INS_GPS_NAV_RESULT_PARSER_H

#include <sstream>
#include <string>
#include "measurement/nav_result.h"

inline bool parseNavResultDataLine(const std::string& line, NavResultData& navResult)
{
    std::stringstream ss(line);

    if (!(ss >> navResult.time
              >> navResult.pos.x()
              >> navResult.pos.y()
              >> navResult.pos.z()
              >> navResult.att.x()
              >> navResult.att.y()
              >> navResult.att.z()))
    {
        return false;
    }

    return true;
}

#endif //MEMS_INS_GPS_NAV_RESULT_PARSER_H