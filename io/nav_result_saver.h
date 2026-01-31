#ifndef MEMS_INS_GPS_NAV_RESULT_SAVER_H
#define MEMS_INS_GPS_NAV_RESULT_SAVER_H

#include <string>
#include <fstream>

#include "state/state.h"

class NavResultSaver
{
public:
    explicit NavResultSaver(const std::string&);

    ~NavResultSaver();

    NavResultSaver(const NavResultSaver&) = delete;

    NavResultSaver& operator=(const NavResultSaver&) = delete;

    NavResultSaver(NavResultSaver&&) = default;

    NavResultSaver& operator=(NavResultSaver&&) = default;

    void save(const double time,const InsState&);

private:
    std::ofstream saveFile_;
};
#endif //MEMS_INS_GPS_NAV_RESULT_SAVER_H