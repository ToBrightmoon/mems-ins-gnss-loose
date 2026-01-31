#include <iostream>
#include <vector>

#include "measurement/ref.h"
#include "io/ref_data_parser.h"
#include "io/nav_result_parser.h"
#include "measurement/nav_result.h"
#include "io/sensor_data_provider.hpp"
#include "eval/nav_evaluator.hpp"
#include "utils/logger.h"


int main(int argc, char **argv)
{
    Logger::Logger::getInstance().init(Logger::Level::INFO);
    
    if (argc < 4)
    {
        LOG_ERROR_STREAM() << "Usage: " << argv[0] << " <ref_path> <res_path> <error_out_path>";
        return -1;
    }

    // 1. 加载参考数据
    SensorDataProvider<RefData> refProvider(argv[1], parseRefDataLine);
    if (!refProvider.load()) return -1;

    std::vector<RefData> refs;
    while (!refProvider.empty()) refs.push_back(refProvider.next());

    SensorDataProvider<NavResultData> resProvider(argv[2], parseNavResultDataLine);
    if (!resProvider.load()) return -1;

    NavEvaluator evaluator;
    evaluator.compute(refs, resProvider);

    evaluator.printStatistics();
    if (evaluator.saveErrorFile(argv[3]))
    {
        LOG_INFO_STREAM() << "Error curve data saved to: " << argv[3];
    }

    return 0;
}
