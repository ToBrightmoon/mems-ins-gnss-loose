#ifndef MEMS_INS_GPS_NAV_EVALUATOR_HPP
#define MEMS_INS_GPS_NAV_EVALUATOR_HPP

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>
#include <Eigen/Dense>

#include "utils/earth_model.h"
#include "utils/constants.h"
#include "utils/rotation.h"
#include "utils/logger.h"

class NavEvaluator
{
public:
    struct ErrorFrame
    {
        double time;
        Eigen::Vector3d posErr;
        Eigen::Vector3d attErr;
    };

    void compute(const std::vector<RefData> &refs, SensorDataProvider<NavResultData> &resProvider)
    {
        errors_.clear();
        size_t refPtr = 0;

        while (!resProvider.empty())
        {
            auto res = resProvider.next();

            // 1. 寻找最近的左邻参考点
            while (refPtr + 1 < refs.size() && refs[refPtr + 1].time < res.time)
            {
                refPtr++;
            }

            // 范围越界检查
            if (refPtr + 1 >= refs.size()) break;

            RefData matchedRef;
            bool needInterpolate = true;

            if (std::abs(res.time - refs[refPtr].time) < TIME_ALIGN_ERR)
            {
                matchedRef = refs[refPtr];
                needInterpolate = false;
            }
            else if (std::abs(res.time - refs[refPtr + 1].time) < TIME_ALIGN_ERR)
            {
                matchedRef = refs[refPtr + 1];
                needInterpolate = false;
            }

            // 3. 获取参考位置与姿态
            Eigen::Vector3d refPos;
            Sophus::SO3d SO3_ref;

            if (!needInterpolate)
            {
                // 直接对齐
                refPos = matchedRef.pos;
                SO3_ref = Sophus::SO3d::exp(euler2vector(matchedRef.att[0] * Deg2Rad,
                                                         matchedRef.att[1] * Deg2Rad,
                                                         matchedRef.att[2] * Deg2Rad));
            }
            else
            {
                // 实在对不齐，执行插值
                LOG_DEBUG("插值对齐");
                double ratio = (res.time - refs[refPtr].time) / (refs[refPtr + 1].time - refs[refPtr].time);

                refPos = refs[refPtr].pos + ratio * (refs[refPtr + 1].pos - refs[refPtr].pos);

                Sophus::SO3d SO3Ref1 = Sophus::SO3d::exp(euler2vector(refs[refPtr].att[0] * Deg2Rad,
                                                                      refs[refPtr].att[1] * Deg2Rad,
                                                                      refs[refPtr].att[2] * Deg2Rad));
                Sophus::SO3d SO3Ref2 = Sophus::SO3d::exp(euler2vector(refs[refPtr + 1].att[0] * Deg2Rad,
                                                                      refs[refPtr + 1].att[1] * Deg2Rad,
                                                                      refs[refPtr + 1].att[2] * Deg2Rad));
                SO3_ref = interpolateAttitude(SO3Ref1, SO3Ref2, ratio);
            }

            // 4. 计算位置误差 (LLH -> NED)
            Eigen::Vector3d dLLHRad;
            dLLHRad << (res.pos[0] - refPos[0]) * Deg2Rad,
                    (res.pos[1] - refPos[1]) * Deg2Rad,
                    (res.pos[2] - refPos[2]);

            Eigen::Matrix3d dr = EarthModel::LLh2NEDMatrix(refPos[0] * Deg2Rad, refPos[2]);

            // 5. 计算姿态误差
            Sophus::SO3d SO3Est = Sophus::SO3d::exp(euler2vector(res.att[0] * Deg2Rad,
                                                                 res.att[1] * Deg2Rad,
                                                                 res.att[2] * Deg2Rad));
            Sophus::SO3d SO3Err = SO3_ref.inverse() * SO3Est;

            ErrorFrame frame;
            frame.time = res.time;
            frame.posErr = dr * dLLHRad;
            frame.attErr = SO3Err.log() * Rad2Deg; // 旋转向量表示的误差角度

            errors_.push_back(frame);
        }
    }

    bool saveErrorFile(const std::string &path) const
    {
        std::ofstream ofs(path);
        if (!ofs.is_open()) return false;

        ofs << "# time(s) dN(m) dE(m) dD(m) dRoll(deg) dPitch(deg) dYaw(deg)\n";
        ofs << std::fixed << std::setprecision(6);
        for (const auto &e: errors_)
        {
            ofs << e.time << " "
                    << e.posErr.x() << " " << e.posErr.y() << " " << e.posErr.z() << " "
                    << e.attErr.x() << " " << e.attErr.y() << " " << e.attErr.z() << "\n";
        }
        return true;
    }

    void printStatistics() const
    {
        if (errors_.empty()) return;

        size_t n = errors_.size();
        Eigen::Vector3d posMse = Eigen::Vector3d::Zero();
        Eigen::Vector3d attMse = Eigen::Vector3d::Zero();

        // 用于统计最大值
        Eigen::Vector3d posMax = Eigen::Vector3d::Zero();
        Eigen::Vector3d attMax = Eigen::Vector3d::Zero();

        // 用于统计分位值 (CDF)
        std::vector<double> posNorms;
        posNorms.reserve(n);

        for (const auto &e: errors_)
        {
            // 累加平方项用于 RMSE
            posMse += e.posErr.array().square().matrix();
            attMse += e.attErr.array().square().matrix();

            // 更新最大值 (取绝对值比较)
            posMax = posMax.cwiseMax(e.posErr.cwiseAbs());
            attMax = attMax.cwiseMax(e.attErr.cwiseAbs());

            // 记录 3D 位置误差模长
            posNorms.push_back(e.posErr.norm());
        }

        // 计算 RMSE
        Eigen::Vector3d posRmse = (posMse / (double) n).array().sqrt();
        Eigen::Vector3d attRmse = (attMse / (double) n).array().sqrt();

        // 计算 95% 误差分位 (CEP95 风格)
        std::sort(posNorms.begin(), posNorms.end());
        double pos95 = posNorms[static_cast<size_t>(n * 0.95)];

        std::cout << "\n==================== EVALUATION REPORT ====================\n";
        std::cout << "  Matched Samples : " << n << "\n";
        std::cout << std::fixed << std::setprecision(4);

        std::cout << "-----------------------------------------------------------\n";
        std::cout << "  Metric        |  North (X)  |   East (Y)  |   Down (Z)  |\n";
        std::cout << "----------------|-------------|-------------|-------------|\n";
        std::cout << "  POS RMSE (m)  |  " << std::setw(10) << posRmse.x() << " |  " << std::setw(10) << posRmse.y() <<
                " |  " << std::setw(10) << posRmse.z() << " |\n";
        std::cout << "  POS MAX  (m)  |  " << std::setw(10) << posMax.x() << " |  " << std::setw(10) << posMax.y() <<
                " |  " << std::setw(10) << posMax.z() << " |\n";

        std::cout << "----------------|-------------|-------------|-------------|\n";
        std::cout << "  Metric        |   Roll (R)  |  Pitch (P)  |   Yaw (Y)   |\n";
        std::cout << "----------------|-------------|-------------|-------------|\n";
        std::cout << "  ATT RMSE (deg)|  " << std::setw(10) << attRmse.x() << " |  " << std::setw(10) << attRmse.y() <<
                " |  " << std::setw(10) << attRmse.z() << " |\n";
        std::cout << "  ATT MAX  (deg)|  " << std::setw(10) << attMax.x() << " |  " << std::setw(10) << attMax.y() <<
                " |  " << std::setw(10) << attMax.z() << " |\n";

        std::cout << "-----------------------------------------------------------\n";
        std::cout << "  3D Position Error (95th percentile): " << pos95 << " m\n";
        std::cout << "===========================================================\n" << std::endl;
    }

private:
    std::vector<ErrorFrame> errors_;

    static Sophus::SO3d interpolateAttitude(const Sophus::SO3d &R1, const Sophus::SO3d &R2, double ratio)
    {
        Sophus::Vector3d delta_omega = (R2 * R1.inverse()).log();

        return Sophus::SO3d::exp(ratio * delta_omega) * R1;
    }
};

#endif //MEMS_INS_GPS_NAV_EVALUATOR_HPP
