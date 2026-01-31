#ifndef MEMS_INS_GPS_POSE_ESTIMATOR_H
#define MEMS_INS_GPS_POSE_ESTIMATOR_H

#include "ins/pins.h"
#include "state/state.h"
#include "measurement/imu.h"
#include "measurement/gnss.h"
#include "filter/gins_config.h"
#include "filter/filter_model.h"
#include "filter/gi_loose_filter.h"

class PoseEstimator
{
public:
    explicit PoseEstimator(const GinsConfig& config);

    ~PoseEstimator() = default;

    PoseEstimator(const PoseEstimator&) = default;

    PoseEstimator& operator=(const PoseEstimator&) = delete;

    PoseEstimator(PoseEstimator&&) = default;

    PoseEstimator& operator=(PoseEstimator&&) = delete;

    void addImuData(const ImuData&);

    void addGnssData(const GnssData&);

    void updatePose();

    [[nodiscard]] NominalState getPose() const;

private:
    [[nodiscard]] ImuData comPenImu(const ImuData &) const;

    void propagate(const ImuData&);

    void gnssUpdate();

    void correctPose();

    void syncFromINS();

private:

    NominalState state_;

    ImuData currImuData_{};

    ImuData prevImuData_{};

    GnssData gnssData_{};

    Pins pins_;

    GILooseFilter filter_;

    Eigen::Vector3d antlever_{};

    bool isInit_ = false;
};
#endif //MEMS_INS_GPS_POSE_ESTIMATOR_H