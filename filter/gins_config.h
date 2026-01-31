#ifndef MEMS_INS_GPS_GINS_CONFIG_H
#define MEMS_INS_GPS_GINS_CONFIG_H

#include <string>
#include <yaml-cpp/yaml.h>

#include "state/state.h"
#include "filter_model.h"

class GinsConfig
{
public:
    explicit GinsConfig(const std::string&);

    ~GinsConfig() = default;

    GinsConfig(const GinsConfig&) = default;

    GinsConfig& operator=(const GinsConfig&) = default;

    GinsConfig(GinsConfig&&) = default;

    GinsConfig& operator=(GinsConfig&&) = default;

    [[nodiscard]] Eigen::MatrixXd buildImuNoise() const;

    [[nodiscard]] Eigen::MatrixXd buildGnssNoise() const;

    [[nodiscard]] Eigen::MatrixXd buildInitialStateNoise() const;

    [[nodiscard]] NominalState buildNominalState() const;

    [[nodiscard]] Eigen::Vector3d buildAntlever() const;

private:
    void loadImuNoise(const YAML::Node& node);

    void loadGnssNoise(const YAML::Node& node);

    void loadNominalState(const YAML::Node& node);

    void loadInitialCovariance(const YAML::Node& node);

    void loadAntlever(const YAML::Node& node);

private:
    ImuNoise imuNoise_;
    GnssNoise gnssNoise_;
    InitialStateNoise initialStateNoise_;
    NominalState nominalState_;
    Eigen::Vector3d antlever_;
};
#endif //MEMS_INS_GPS_GINS_CONFIG_H