#ifndef STATE_H
#define STATE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

struct InsState
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Quaterniond quat;
};

struct NominalState
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Quaterniond quat;
    Eigen::Vector3d bg;
    Eigen::Vector3d ba;
};

struct GIState
{
    Eigen::Vector3d posDelta;
    Eigen::Vector3d velDelta;
    Eigen::Vector3d attDelta;
    Eigen::Vector3d bg;
    Eigen::Vector3d ba;
};

inline InsState NominalState2InsState(const NominalState& state)
{
    InsState result;
    result.pos = state.pos;
    result.vel = state.vel;
    result.quat = state.quat;
    return result;
}
#endif //STATE_H
