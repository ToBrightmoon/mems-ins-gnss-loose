#include "imu_time_aligner.h"

ImuTimeAligner::Mode ImuTimeAligner::getMode(const ImuData &prev, const ImuData &curr, const GnssData &gnss, double eps)
{
    if (!gnss.valid) return Mode::No;

    double t = gnss.time;
    if (std::abs(prev.time - t) < eps) return Mode::Curr;
    if (std::abs(curr.time - t) < eps) return Mode::Next;
    if (prev.time < t && curr.time > t) return Mode::Internal;
    return Mode::No;
}

ImuTimeAligner::InterpResult ImuTimeAligner::getInterpResult(const ImuData &prev, const ImuData &curr, double t)
{
    if (t <= prev.time || t >= curr.time) {
        throw std::runtime_error("IMU 插值时间非法");
    }

    double lambda = (curr.time - t) / (curr.time - prev.time);

    InterpResult res;
    res.mid.time = t;
    res.mid.accel = lambda * curr.accel;
    res.mid.gyro  = lambda * curr.gyro;

    res.remain.time = curr.time;
    res.remain.accel = curr.accel - res.mid.accel;
    res.remain.gyro  = curr.gyro  - res.mid.gyro;

    return res;
}
