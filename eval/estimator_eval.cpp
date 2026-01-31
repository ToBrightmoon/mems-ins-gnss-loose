#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <memory>
#include <fstream>
#include "estimator/pose_estimator.h"
#include "filter/gins_config.h"
#include "io/sensor_data_provider.hpp"
#include "io/gnss_data_parser.h"
#include "io/imu_data_parser.h"
#include "io/nav_result_saver.h"
#include "utils/logger.h"

int main(int argc, char **argv)
{
    Logger::Logger::getInstance().init(Logger::Level::INFO);
    
    if (argc < 4)
    {
        LOG_ERROR("Usage: ./estimator_eval [imu_path] [gnss_path] [config_path] [output_path]");
        return -1;
    }

    std::string imuPath = argv[1];
    std::string gnssPath = argv[2];
    std::string configPath = argv[3];
    std::string outPath = (argc > 4) ? argv[4] : "res.txt";
    try
    {
        auto imuDataProvider = SensorDataProvider<ImuData>(imuPath,parseImuLine);
        imuDataProvider.load();

        auto gnssDataProvider = SensorDataProvider<GnssData>(gnssPath,parseGnssLine);
        gnssDataProvider.load();

        GinsConfig config{configPath};

        if (imuDataProvider.size() < 2 || gnssDataProvider.empty())
        {
            LOG_ERROR("Data source is empty!");
            return -1;
        }
        auto gnssData = gnssDataProvider.next();
        NavResultSaver saver(outPath);

        PoseEstimator estimator(config);

        estimator.addImuData(imuDataProvider.next());
        estimator.addGnssData(gnssData);

        LOG_INFO("Starting Pose Estimation...");

        while (!imuDataProvider.empty())
        {
            auto imuData = imuDataProvider.next();
            estimator.addImuData(imuData);

            while (!gnssDataProvider.empty() && gnssData.time <= imuData.time)
            {
                estimator.addGnssData(gnssData);
                gnssData = gnssDataProvider.next();
            }

            estimator.updatePose();

            NominalState cur = estimator.getPose();

            saver.save(imuData.time,NominalState2InsState(cur));

        }

        LOG_INFO_STREAM() << "Estimation Done. Results saved to: " << outPath;

        return 0;
    }
    catch (std::exception& e)
    {
        LOG_ERROR_STREAM() << "Exception: " << e.what();
        return -1;
    }

}
