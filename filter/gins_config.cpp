#include "gins_config.h"

#include <cmath>
#include <fstream>
#include <filesystem>

#include "utils/constants.h"
#include "utils/rotation.h"
#include "utils/exceptions.h"


GinsConfig::GinsConfig(const std::string& path)
{
    // 检查文件是否存在
    if (!std::filesystem::exists(path)) {
        throw mems_ins_gps::FileException("load config", "File does not exist: " + path);
    }

    // 检查文件是否可读
    std::ifstream testFile(path);
    if (!testFile.good()) {
        throw mems_ins_gps::FileException("load config", "Cannot read file: " + path);
    }
    testFile.close();

    try {
        YAML::Node config = YAML::LoadFile(path);

        if (config.IsNull()) {
            throw mems_ins_gps::ConfigException("Invalid YAML file: " + path);
        }

        loadImuNoise(config["imu"]);
        loadGnssNoise(config["gnss"]);
        loadNominalState(config["initial_state"]);
        loadInitialCovariance(config["initial_covariance"]);
        loadAntlever(config["antlever"]);
    } catch (const YAML::BadFile& e) {
        throw mems_ins_gps::FileException("load config", "YAML parse error: " + std::string(e.what()));
    } catch (const YAML::ParserException& e) {
        throw mems_ins_gps::ConfigException("YAML syntax error: " + std::string(e.what()));
    } catch (const mems_ins_gps::BaseException&) {
        throw;  // 重新抛出我们的异常
    } catch (const std::exception& e) {
        throw mems_ins_gps::ConfigException("Unexpected error: " + std::string(e.what()));
    }
}

Eigen::MatrixXd GinsConfig::buildImuNoise() const
{
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(12, 12);

    Q.block<3,3>(0,0) = imuNoise_.accVrw.array().square().matrix().asDiagonal();
    Q.block<3,3>(3,3) = imuNoise_.gyrArw.array().square().matrix().asDiagonal();
    Q.block<3,3>(6,6) = imuNoise_.accBiasStd.array().square().matrix().asDiagonal();
    Q.block<3,3>(9,9) = imuNoise_.gyrBiasStd.array().square().matrix().asDiagonal();

    return Q;
}

Eigen::MatrixXd GinsConfig::buildGnssNoise() const
{
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3,3);
    R.diagonal() = gnssNoise_.posStd.array().square();
    return R;
}

Eigen::MatrixXd GinsConfig::buildInitialStateNoise() const
{
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(15,15);

    P.block<3,3>(0,0)   = initialStateNoise_.posNoise.array().square().matrix().asDiagonal();
    P.block<3,3>(3,3)   = initialStateNoise_.velNoise.array().square().matrix().asDiagonal();
    P.block<3,3>(6,6)   = initialStateNoise_.attNoise.array().square().matrix().asDiagonal();
    P.block<3,3>(9,9)   = initialStateNoise_.bgNoise.array().square().matrix().asDiagonal();
    P.block<3,3>(12,12) = initialStateNoise_.baNoise.array().square().matrix().asDiagonal();

    return P;
}

NominalState GinsConfig::buildNominalState() const
{
    return nominalState_;
}

Eigen::Vector3d GinsConfig::buildAntlever() const
{
    return antlever_;
}

void GinsConfig::loadImuNoise(const YAML::Node& node)
{
    if (!node || node.IsNull()) {
        throw mems_ins_gps::ConfigException("imu", "Missing or null node");
    }

    try {
        auto arw = node["arw"].as<std::vector<double>>();
        auto vrw = node["vrw"].as<std::vector<double>>();
        auto gb  = node["gbstd"].as<std::vector<double>>();
        auto ab  = node["abstd"].as<std::vector<double>>();

        if (arw.size() != 3 || vrw.size() != 3 || gb.size() != 3 || ab.size() != 3) {
            throw mems_ins_gps::ValidationException("imu", "All noise parameters must have 3 elements");
        }

        for (int i = 0; i < 3; ++i)
        {
            if (arw[i] < 0 || vrw[i] < 0 || gb[i] < 0 || ab[i] < 0) {
                throw mems_ins_gps::ValidationException("imu", "Noise parameters must be non-negative");
            }
            imuNoise_.gyrArw[i]     = arw[i] * Deg2Rad / std::sqrt(Hour2Sec);
            imuNoise_.accVrw[i]     = vrw[i] / std::sqrt(Hour2Sec);
            imuNoise_.gyrBiasStd[i] = gb[i]  * Deg2Rad / Hour2Sec;
            imuNoise_.accBiasStd[i] = ab[i]  * 1e-5;   // mGal -> m/s^2
        }
    } catch (const YAML::BadConversion& e) {
        throw mems_ins_gps::ConfigException("imu", "Type conversion error: " + std::string(e.what()));
    }
}

void GinsConfig::loadGnssNoise(const YAML::Node& node)
{
    if (!node || node.IsNull()) {
        throw mems_ins_gps::ConfigException("gnss", "Missing or null node");
    }

    try {
        auto p = node["position"].as<std::vector<double>>();
        if (p.size() != 3) {
            throw mems_ins_gps::ValidationException("gnss.position", "Must have 3 elements");
        }
        if (p[0] < 0 || p[1] < 0 || p[2] < 0) {
            throw mems_ins_gps::ValidationException("gnss.position", "Position std must be non-negative");
        }
        gnssNoise_.posStd = Eigen::Vector3d(p[0], p[1], p[2]);
    } catch (const YAML::BadConversion& e) {
        throw mems_ins_gps::ConfigException("gnss", "Type conversion error: " + std::string(e.what()));
    }
}

void GinsConfig::loadNominalState(const YAML::Node& node)
{
    if (!node || node.IsNull()) {
        throw mems_ins_gps::ConfigException("initial_state", "Missing or null node");
    }

    try {
        auto pos = node["position"].as<std::vector<double>>();
        auto vel = node["velocity"].as<std::vector<double>>();
        auto att = node["attitude"].as<std::vector<double>>();

        if (pos.size() != 3) {
            throw mems_ins_gps::ValidationException("initial_state.position", "Must have 3 elements [lat, lon, h]");
        }
        if (vel.size() != 3) {
            throw mems_ins_gps::ValidationException("initial_state.velocity", "Must have 3 elements");
        }
        if (att.size() != 3) {
            throw mems_ins_gps::ValidationException("initial_state.attitude", "Must have 3 elements [roll, pitch, yaw]");
        }

        // 验证位置范围
        if (pos[0] < -90 || pos[0] > 90) {
            throw mems_ins_gps::ValidationException("initial_state.position[0]", "Latitude must be in [-90, 90]");
        }
        if (pos[1] < -180 || pos[1] > 180) {
            throw mems_ins_gps::ValidationException("initial_state.position[1]", "Longitude must be in [-180, 180]");
        }

        nominalState_.pos << pos[0] * Deg2Rad,
                              pos[1] * Deg2Rad,
                              pos[2];

        nominalState_.vel << vel[0], vel[1], vel[2];

        nominalState_.quat = euler2quaternion(att[0]*Deg2Rad,att[1]*Deg2Rad,att[2]*Deg2Rad);
        nominalState_.quat.normalize();

        nominalState_.bg.setZero();
        nominalState_.ba.setZero();
    } catch (const YAML::BadConversion& e) {
        throw mems_ins_gps::ConfigException("initial_state", "Type conversion error: " + std::string(e.what()));
    }
}

void GinsConfig::loadInitialCovariance(const YAML::Node& node)
{
    if (!node || node.IsNull()) {
        throw mems_ins_gps::ConfigException("initial_covariance", "Missing or null node");
    }

    try {
        auto pos = node["pos"].as<std::vector<double>>();
        auto vel = node["vel"].as<std::vector<double>>();
        auto att = node["att"].as<std::vector<double>>();
        auto bg  = node["gyro_bias"].as<std::vector<double>>();
        auto ba  = node["accel_bias"].as<std::vector<double>>();

        if (pos.size() != 3 || vel.size() != 3 || att.size() != 3 || 
            bg.size() != 3 || ba.size() != 3) {
            throw mems_ins_gps::ValidationException("initial_covariance", "All covariance parameters must have 3 elements");
        }

        // 验证协方差必须为正
        for (size_t i = 0; i < 3; ++i) {
            if (pos[i] <= 0 || vel[i] <= 0 || att[i] <= 0 || 
                bg[i] <= 0 || ba[i] <= 0) {
                throw mems_ins_gps::ValidationException("initial_covariance", "All covariance values must be positive");
            }
        }

        initialStateNoise_.posNoise = Eigen::Vector3d(pos[0], pos[1], pos[2]);
        initialStateNoise_.velNoise = Eigen::Vector3d(vel[0], vel[1], vel[2]);
        initialStateNoise_.attNoise = Eigen::Vector3d(att[0], att[1], att[2]) * Deg2Rad;
        initialStateNoise_.bgNoise  = Eigen::Vector3d(bg[0], bg[1], bg[2]) * Deg2Rad / Hour2Sec;
        initialStateNoise_.baNoise  = Eigen::Vector3d(ba[0], ba[1], ba[2]) * 1e-5;
    } catch (const YAML::BadConversion& e) {
        throw mems_ins_gps::ConfigException("initial_covariance", "Type conversion error: " + std::string(e.what()));
    }
}

void GinsConfig::loadAntlever(const YAML::Node& node)
{
    if (!node || node.IsNull()) {
        throw mems_ins_gps::ConfigException("antlever", "Missing or null node");
    }

    try {
        auto v = node.as<std::vector<double>>();
        if (v.size() != 3) {
            throw mems_ins_gps::ValidationException("antlever", "Must have 3 elements [x, y, z]");
        }
        antlever_ << v[0], v[1], v[2];
    } catch (const YAML::BadConversion& e) {
        throw mems_ins_gps::ConfigException("antlever", "Type conversion error: " + std::string(e.what()));
    }
}

