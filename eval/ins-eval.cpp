#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>

#include "filter/gins_config.h"
#include "ins/pins.h"
#include "state/state.h"
#include "io/sensor_data_provider.hpp"
#include "io/imu_data_parser.h"
#include "io/nav_result_saver.h"
#include "utils/logger.h"
#include "utils/exceptions.h"


int main(int argc, const char **argv)
{
    Logger::Logger::getInstance().init(Logger::Level::INFO);
    
    if (argc < 4)
    {
        LOG_ERROR_STREAM() << "Usage: " << argv[0] << " <imu_file> <output_file> <config_file>";
        return -1;
    }

    std::string imuPath = argv[1];
    std::string outPath = argv[2];
    std::string configPath = argv[3];

    try
    {
        SensorDataProvider<ImuData> imuDataProvider(imuPath, parseImuLine);
        imuDataProvider.load();

        if (imuDataProvider.size() < 2)
        {
            LOG_ERROR("IMU 数据量太少,不支持解算");
            return -1;
        }

        GinsConfig config{configPath};
        auto nomialState = config.buildNominalState();

        LOG_INFO_STREAM() << "IMU samples: " << imuDataProvider.size();

        Pins ins;
        ins.init(NominalState2InsState(nomialState), imuDataProvider.next());

        NavResultSaver saver(outPath);
        while (!imuDataProvider.empty())
        {
            auto currImuData = imuDataProvider.next();
            ins.insUpdate(currImuData);

            const InsState &s = ins.getState();
            saver.save(currImuData.time, s);
        }

        LOG_INFO_STREAM() << "INS test finished. Output saved to: " << outPath;
        return 0;
    }
    catch (const mems_ins_gps::FileException& e)
    {
        LOG_ERROR_STREAM() << "File error: " << e.what();
        return -1;
    }
    catch (const mems_ins_gps::ConfigException& e)
    {
        LOG_ERROR_STREAM() << "Config error: " << e.what();
        return -1;
    }
    catch (const mems_ins_gps::DataException& e)
    {
        LOG_ERROR_STREAM() << "Data error: " << e.what();
        return -1;
    }
    catch (const mems_ins_gps::TimeException& e)
    {
        LOG_ERROR_STREAM() << "Time error: " << e.what();
        return -1;
    }
    catch (const mems_ins_gps::ValidationException& e)
    {
        LOG_ERROR_STREAM() << "Validation error: " << e.what();
        return -1;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR_STREAM() << "Unexpected error: " << e.what();
        return -1;
    }
    catch (...)
    {
        LOG_ERROR("Unknown error occurred");
        return -1;
    }
}
