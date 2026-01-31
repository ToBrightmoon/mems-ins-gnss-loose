#include "nav_result_saver.h"

#include <iomanip>

#include "utils/rotation.h"
#include "utils/constants.h"
#include "utils/logger.h"

NavResultSaver::NavResultSaver(const std::string & path):saveFile_(path)
{
    if (!saveFile_.is_open())
    {
        LOG_ERROR_STREAM() << "无法打开输出路径: " << path;
        throw std::runtime_error("无法打开路径");
    }
    saveFile_ << std::fixed << std::setprecision(10);
}

NavResultSaver::~NavResultSaver()
{
    saveFile_.close();
}

void NavResultSaver::save(const double time,const InsState & state)
{
    const double lat = state.pos[0]*Rad2Deg;
    const double lon = state.pos[1]*Rad2Deg;
    const double alt = state.pos[2];
    const Eigen::Vector3d euler = quaternion2euler(state.quat)*Rad2Deg;
    const double roll = euler[0];
    const double pitch = euler[1];
    const double yaw = euler[2];

    saveFile_ << time << " "
                    << lat << " "
                    << lon << " "
                    << alt << " "
                    << roll << " "
                    << pitch << " "
                    << yaw << "\n";
}
