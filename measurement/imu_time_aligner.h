#ifndef MEMS_INS_GPS_IMU_TIME_ALIGNER_H
#define MEMS_INS_GPS_IMU_TIME_ALIGNER_H

#include "measurement/imu.h"
#include "measurement/gnss.h"

class ImuTimeAligner
{
public:
    enum class Mode
    {
        No,
        Curr,
        Next,
        Internal
    };

    struct InterpResult
    {
        ImuData mid;
        ImuData remain;
    };

    static Mode getMode(const ImuData &prev,
                        const ImuData &curr,
                        const GnssData &gnss,
                        double eps);

    static InterpResult getInterpResult(const ImuData &prev,
                                        const ImuData &curr,
                                        double t);
};
#endif //MEMS_INS_GPS_IMU_TIME_ALIGNER_H
