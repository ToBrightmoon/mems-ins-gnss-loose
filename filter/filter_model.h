#ifndef MEMS_INS_GPS_FILTER_MODEL_H
#define MEMS_INS_GPS_FILTER_MODEL_H

#include <Eigen/Core>

struct FilterModel
{
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd P;
};

struct ImuNoise
{
    Eigen::Vector3d gyrArw;
    Eigen::Vector3d accVrw;
    Eigen::Vector3d gyrBiasStd;
    Eigen::Vector3d accBiasStd;
};

struct GnssNoise
{
    Eigen::Vector3d posStd;
};

struct InitialStateNoise
{
    Eigen::Vector3d posNoise;
    Eigen::Vector3d velNoise;
    Eigen::Vector3d attNoise;
    Eigen::Vector3d bgNoise;
    Eigen::Vector3d baNoise;
};



#endif //MEMS_INS_GPS_FILTER_MODEL_H