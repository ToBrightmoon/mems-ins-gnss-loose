#include "gi_loose_filter.h"

#include <sophus/so3.hpp>

#include "utils/earth_model.h"
#include "utils/exceptions.h"

/**
 * @brief 构造函数
 * 
 * @param Q 过程噪声协方差矩阵（12x12）
 * @param R 观测噪声协方差矩阵（3x3）
 * @param P 初始误差状态协方差矩阵（15x15）
 * @throw ValidationException 如果矩阵维度不匹配
 */
GILooseFilter::GILooseFilter(const Eigen::MatrixXd& Q,const Eigen::MatrixXd& R , const Eigen::MatrixXd& P):Q_(Q),R_(R),P_(P)
{
    // 验证矩阵维度
    if (Q_.rows() != 12 || Q_.cols() != 12) {
        throw mems_ins_gps::ValidationException("Q", 
            "Process noise matrix must be 12x12, got " + 
            std::to_string(Q_.rows()) + "x" + std::to_string(Q_.cols()));
    }
    if (R_.rows() != 3 || R_.cols() != 3) {
        throw mems_ins_gps::ValidationException("R", 
            "Measurement noise matrix must be 3x3, got " + 
            std::to_string(R_.rows()) + "x" + std::to_string(R_.cols()));
    }
    if (P_.rows() != 15 || P_.cols() != 15) {
        throw mems_ins_gps::ValidationException("P", 
            "Initial covariance matrix must be 15x15, got " + 
            std::to_string(P_.rows()) + "x" + std::to_string(P_.cols()));
    }

    // 验证矩阵对称性（协方差矩阵应该是对称的）
    if (!Q_.isApprox(Q_.transpose(), 1e-6)) {
        throw mems_ins_gps::ValidationException("Q", 
            "Process noise matrix must be symmetric");
    }
    if (!R_.isApprox(R_.transpose(), 1e-6)) {
        throw mems_ins_gps::ValidationException("R", 
            "Measurement noise matrix must be symmetric");
    }
    if (!P_.isApprox(P_.transpose(), 1e-6)) {
        throw mems_ins_gps::ValidationException("P", 
            "Initial covariance matrix must be symmetric");
    }

    // 验证矩阵正定性（协方差矩阵应该是正定的）
    Eigen::LDLT<Eigen::MatrixXd> Q_ldlt(Q_);
    if (Q_ldlt.info() != Eigen::Success || (Q_ldlt.vectorD().array() <= 0).any()) {
        throw mems_ins_gps::ValidationException("Q", 
            "Process noise matrix must be positive definite");
    }
    Eigen::LDLT<Eigen::MatrixXd> R_ldlt(R_);
    if (R_ldlt.info() != Eigen::Success || (R_ldlt.vectorD().array() <= 0).any()) {
        throw mems_ins_gps::ValidationException("R", 
            "Measurement noise matrix must be positive definite");
    }
    Eigen::LDLT<Eigen::MatrixXd> P_ldlt(P_);
    if (P_ldlt.info() != Eigen::Success || (P_ldlt.vectorD().array() <= 0).any()) {
        throw mems_ins_gps::ValidationException("P", 
            "Initial covariance matrix must be positive definite");
    }

    reset();
}

/**
 * @brief 误差状态卡尔曼滤波的时间更新（预测步骤）
 * 
 * 预测步骤包括：
 * 1. 误差状态传播：δx_k|k-1 = Phi * δx_{k-1|k-1}
 * 2. 误差协方差传播：P_k|k-1 = Phi * P_{k-1|k-1} * Phi^T + G * Q * G^T
 * 
 * 其中：
 *   - Phi: 状态转移矩阵（15x15）
 *   - G: 过程噪声输入矩阵（15x12）
 *   - Q: 过程噪声协方差矩阵（12x12）
 *   - P: 误差状态协方差矩阵（15x15）
 * 
 * @param insState 当前INS名义状态
 * @param imuData IMU测量数据
 * @param dt 时间间隔 (s)
 */
void GILooseFilter::predict(const InsState & insState, const ImuData & imuData, const double dt)
{
    // 验证时间间隔
    if (dt <= 0.0) {
        throw mems_ins_gps::ValidationException("dt", 
            "Time interval must be positive");
    }
    if (dt > 1.0) {
        throw mems_ins_gps::ValidationException("dt", 
            "Time interval too large (> 1.0s)");
    }

    // 验证INS状态
    if (std::abs(insState.quat.norm() - 1.0) > 1e-6) {
        throw mems_ins_gps::ValidationException("insState.quat", 
            "Quaternion is not normalized");
    }

    // 构建状态转移矩阵和过程噪声输入矩阵
    auto Phi = buildPhi(insState,imuData,dt);
    auto G = buildG(insState);

    // 更新误差协方差矩阵
    // P_k|k-1 = Phi * P_{k-1|k-1} * Phi^T + G * Q * G^T
    // 第一项：状态传播引起的协方差变化
    // 第二项：过程噪声引起的协方差增加
    P_ = Phi * P_ * Phi.transpose() + G * Q_ * G.transpose();

    // 更新误差状态（如果误差状态非零）
    // 在标准ESKF中，预测步骤通常假设误差状态为零，但这里保留传播逻辑
    Eigen::VectorXd state = Eigen::VectorXd::Zero(15,1);
    state.block(0,0,3,1) = state_.posDelta;
    state.block(3,0,3,1) = state_.velDelta;
    state.block(6,0,3,1) = state_.attDelta;
    state.block(9,0,3,1) = state_.bg;
    state.block(12,0,3,1) = state_.ba;
    state = Phi * state;  // 误差状态传播
    
    // 更新误差状态
    state_.posDelta = state.segment(0,3);
    state_.velDelta = state.segment(3,3);
    state_.attDelta = state.segment(6,3);
    state_.bg = state.segment(9,3);
    state_.ba = state.segment(12,3);
}

/**
 * @brief 误差状态卡尔曼滤波的测量更新（校正步骤）
 * 
 * 测量更新使用GNSS位置观测，观测方程为：
 *   z = H * δx + v
 * 其中：
 *   - z: 观测残差（GNSS位置与INS位置的差值，NED坐标系，单位：m）
 *   - H: 观测矩阵（3x15），H = [I 0 0 0 0]，只观测位置误差
 *   - v: 观测噪声（GNSS位置误差）
 * 
 * 更新步骤：
 * 1. 计算新息协方差：S = H * P * H^T + R
 * 2. 计算卡尔曼增益：K = P * H^T * S^(-1)
 * 3. 更新误差状态：δx_k|k = δx_k|k-1 + K * (z - H * δx_k|k-1)
 * 4. 更新误差协方差：P_k|k = (I - K*H) * P_k|k-1 * (I - K*H)^T + K * R * K^T
 *    （使用Joseph形式保证数值稳定性）
 * 
 * @param dz 观测残差（GNSS位置与INS位置的差值，NED坐标系，单位：m）
 */
void GILooseFilter::update(const Eigen::Vector3d & dz)
{
    // 验证观测残差的有效性
    if (!std::isfinite(dz.x()) || !std::isfinite(dz.y()) || !std::isfinite(dz.z())) {
        throw mems_ins_gps::ValidationException("dz", 
            "Observation residual contains NaN or Inf");
    }

    // 验证观测残差的大小（假设位置误差不应超过1000km）
    if (dz.norm() > 1e6) {
        throw mems_ins_gps::ValidationException("dz", 
            "Observation residual too large (> 1000 km)");
    }

    // 构建观测矩阵 H（3x15）
    // 只观测位置误差，所以H = [I_{3x3} 0 0 0 0]
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,15);
    H.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3);

    // 计算新息协方差矩阵 S = H * P * H^T + R
    // S表示观测预测的不确定性
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;

    // 验证S矩阵的条件数（避免数值不稳定）
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(S);
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
    if (cond > 1e12) {
        throw mems_ins_gps::ValidationException("S", 
            "Innovation covariance matrix is ill-conditioned (cond=" + 
            std::to_string(cond) + ")");
    }

    // 计算卡尔曼增益 K = P * H^T * S^(-1)
    // 卡尔曼增益决定了观测信息对状态估计的修正程度
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // 更新误差协方差矩阵（使用Joseph形式保证对称正定）
    // P_k|k = (I - K*H) * P_k|k-1 * (I - K*H)^T + K * R * K^T
    Eigen::MatrixXd Ks = (Eigen::MatrixXd::Identity(15,15) - K * H);
    P_ = Ks * P_ * Ks.transpose() + K * R_ * K.transpose();

    // 计算误差状态修正量
    Eigen::VectorXd dx = K * dz;

    // 更新误差状态
    state_.posDelta += dx.segment<3>(0);   // 位置误差修正
    state_.velDelta += dx.segment<3>(3);  // 速度误差修正
    state_.attDelta += dx.segment<3>(6);  // 姿态误差修正
    state_.bg  += dx.segment<3>(9);       // 陀螺零偏误差修正
    state_.ba  += dx.segment<3>(12);      // 加速度计零偏误差修正
}

GIState GILooseFilter::getState() const
{
    return state_;
}

void GILooseFilter::reset()
{
    state_.posDelta = Eigen::Vector3d::Zero();
    state_.velDelta = Eigen::Vector3d::Zero();
    state_.attDelta = Eigen::Vector3d::Zero();
    state_.bg = Eigen::Vector3d::Zero();
    state_.ba = Eigen::Vector3d::Zero();
}

/**
 * @brief 构建误差状态转移矩阵 Phi
 * 
 * 误差状态向量：δx = [δr, δv, δφ, δbg, δba]^T (15维)
 * 其中：
 *   - δr: 位置误差 (3维，NED坐标系，单位：m)
 *   - δv: 速度误差 (3维，NED坐标系，单位：m/s)
 *   - δφ: 姿态误差 (3维，载体坐标系，单位：rad)
 *   - δbg: 陀螺零偏误差 (3维，单位：rad/s)
 *   - δba: 加速度计零偏误差 (3维，单位：m/s²)
 * 
 * 状态转移方程：δx_k = Phi * δx_{k-1}
 * 
 * 矩阵结构：
 *   Phi = [Frr  Frv  0    0    0  ]
 *         [Fvr  Fvv  Fvt  0    Fvba]
 *         [Ftr  Ftv  Ftt  Ftbg 0  ]
 *         [0    0    0    Fbg  0  ]
 *         [0    0    0    0    Fba]
 * 
 * @param insState 当前INS状态
 * @param imuData IMU测量数据
 * @param dt 时间间隔 (s)
 * @return 15x15的状态转移矩阵
 */
Eigen::MatrixXd GILooseFilter::buildPhi(const InsState & insState,const ImuData& imuData,const double dt)
{
    // 计算当前时刻的地球模型参数
    auto navParam = EarthModel::computeNavParam(insState.pos[0], insState.pos[2], insState.vel[0],insState.vel[1]);
    double vn = insState.vel[0];  // 北向速度
    double ve = insState.vel[1];  // 东向速度
    double vd = insState.vel[2];  // 地向速度
    double lat = insState.pos[0]; // 纬度（弧度）
    double wie = EarthModel::getWie();  // 地球自转角速度

    // ========== 1. 位置误差传播矩阵 ==========
    // Frr: 位置误差对位置误差的传播（由速度引起的曲率效应）
    // 考虑地球曲率半径对位置误差的影响
    Eigen::Matrix3d Frr;
    Frr.setZero(3,3);
    // 纬度误差传播：考虑地向速度对纬度的影响
    Frr(0,0) = -vd/navParam.Rmh;  // 纬度误差对纬度误差的影响
    Frr(0,2) = vn/navParam.Rmh;   // 高度误差对纬度误差的影响
    // 经度误差传播：考虑速度对经度的影响（与纬度相关）
    Frr(1,0) = (ve*tan(lat))/navParam.Rnh;  // 纬度误差对经度误差的影响
    Frr(1,1) = -(vd + vn * tan(lat))/navParam.Rnh;  // 经度误差对经度误差的影响
    Frr(1,2) = ve/navParam.Rnh;   // 高度误差对经度误差的影响

    // Frv: 速度误差对位置误差的传播（直接积分关系）
    Eigen::Matrix3d Frv = Eigen::Matrix3d::Identity();

    // ========== 2. 速度误差传播矩阵 ==========
    // Fvr: 位置误差对速度误差的传播（由地球曲率和哥式力引起）
    Eigen::Matrix3d Fvr;
    Fvr.setZero(3,3);
    // 北向速度误差传播：考虑地球自转、曲率半径和速度耦合项
    Fvr(0,0) = -2*ve*wie*cos(lat)/navParam.Rmh - std::pow(ve,2)/(navParam.Rmh*navParam.Rnh*std::pow(std::cos(lat),2));
    Fvr(0,2) = vn*vd/std::pow(navParam.Rmh,2) - std::pow(ve,2)*std::tan(lat)/std::pow(navParam.Rnh,2);
    // 东向速度误差传播
    Fvr(1,0) = 2*EarthModel::getWie()*(vn*std::cos(lat) - vd*std::sin(lat))/navParam.Rmh + vn*ve/(navParam.Rmh*navParam.Rnh*std::pow(std::cos(lat),2));
    Fvr(1,2) = (ve*vd + vn*ve*std::tan(lat))/std::pow(navParam.Rnh,2);
    // 地向速度误差传播：考虑地球自转和重力梯度
    Fvr(2,0) = 2*wie*ve*std::sin(lat)/navParam.Rmh;
    Fvr(2,2) = -std::pow(ve,2)/std::pow(navParam.Rnh,2) - std::pow(vn,2)/std::pow(navParam.Rmh,2) + 2*EarthModel::getGravity(lat,insState.pos[2])/(std::pow(navParam.Rm*navParam.Rn,0.5) + insState.pos[2]);

    // Fvv: 速度误差对速度误差的传播（由哥式力和曲率引起）
    Eigen::Matrix3d Fvv;
    Fvv.setZero(3,3);
    // 北向速度误差的自耦合和交叉耦合
    Fvv(0,0) = vd/navParam.Rmh;  // 地向速度对北向速度误差的影响
    Fvv(0,1) = -2*(wie*std::sin(lat) + ve*std::tan(lat)/navParam.Rnh);  // 东向速度对北向速度误差的影响（哥式力）
    Fvv(0,2) = vn/navParam.Rmh;  // 北向速度对北向速度误差的影响
    // 东向速度误差的自耦合和交叉耦合
    Fvv(1,0) = 2*wie*std::sin(lat) + ve*std::tan(lat)/navParam.Rnh;  // 北向速度对东向速度误差的影响（哥式力）
    Fvv(1,1) = (vd + vn*std::tan(lat))/navParam.Rnh;  // 东向速度误差的自耦合
    Fvv(1,2) = 2*wie*std::cos(lat) + ve/navParam.Rnh;  // 地向速度对东向速度误差的影响（哥式力）
    // 地向速度误差的自耦合和交叉耦合
    Fvv(2,0) = -2*vn/navParam.Rmh;  // 北向速度对地向速度误差的影响
    Fvv(2,1) = -2*(wie*std::cos(lat) + ve/navParam.Rnh);  // 东向速度对地向速度误差的影响（哥式力）

    // Fvt: 姿态误差对速度误差的传播（由比力投影误差引起）
    // 当姿态有误差时，比力从载体坐标系到导航坐标系的投影会产生误差
    // Fvt = -[f_n ×]，其中f_n是比力在导航系的投影
    Eigen::Vector3d f_n = insState.quat.toRotationMatrix() * imuData.accel / dt;
    Eigen::Matrix3d Fvt = Sophus::SO3d::hat(f_n);

    // Fvba: 加速度计零偏误差对速度误差的传播（直接投影）
    // 加速度计零偏直接投影到导航坐标系
    Eigen::Matrix3d Fvba = insState.quat.toRotationMatrix();

    // ========== 3. 姿态误差传播矩阵 ==========
    // Ftr: 位置误差对姿态误差的传播（由地球曲率引起）
    Eigen::Matrix3d Ftr;
    Ftr.setZero(3,3);
    // 姿态误差对位置误差的敏感性（考虑地球曲率半径）
    Ftr(0,0) = -wie*sin(lat)/navParam.Rmh;  // 纬度误差对横滚误差的影响
    Ftr(0,2) = ve/std::pow(navParam.Rnh,2);  // 高度误差对横滚误差的影响
    Ftr(1,2) = -vn/std::pow(navParam.Rmh,2);  // 高度误差对俯仰误差的影响
    // 航向误差对位置误差的敏感性
    Ftr(2,0) = -wie*cos(lat)/navParam.Rmh - ve/(navParam.Rmh*navParam.Rnh*std::pow(std::cos(lat),2));
    Ftr(2,2) = -ve*std::tan(lat)/std::pow(navParam.Rnh,2);

    // Ftv: 速度误差对姿态误差的传播（由运移角速度引起）
    Eigen::Matrix3d Ftv;
    Ftv.setZero(3,3);
    // 速度误差通过运移角速度影响姿态误差
    Ftv(0,1) = 1/navParam.Rnh;   // 东向速度对横滚误差的影响
    Ftv(1,0) = -1/navParam.Rmh;  // 北向速度对俯仰误差的影响
    Ftv(2,1) = -std::tan(lat)/navParam.Rnh;  // 东向速度对航向误差的影响

    // Ftt: 姿态误差对姿态误差的传播（由地球自转和运移角速度引起）
    // 姿态误差会随着导航系的旋转而传播
    // Ftt = -[ω_ie + ω_en ×]，表示姿态误差的旋转耦合
    Eigen::Matrix3d Ftt = -Sophus::SO3d::hat(navParam.wie_n + navParam.wen_n);

    // Ftbg: 陀螺零偏误差对姿态误差的传播（直接投影）
    // 陀螺零偏直接投影到载体坐标系
    Eigen::Matrix3d Ftbg = -insState.quat.toRotationMatrix();

    // ========== 4. 零偏误差传播矩阵 ==========
    // 零偏建模为一阶马尔可夫过程：db/dt = -β*db + w
    // 其中β = 1/(2*τ)，τ为相关时间（这里假设为2小时）
    // 离散化后：F = -β*I = -I/(2*τ) = -I/2 (假设τ=1，单位归一化)
    // 注意：这里使用-1/2作为衰减系数，实际应根据零偏相关时间调整

    // ========== 5. 组装完整的状态转移矩阵 ==========
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(15,15);
    // 位置误差相关
    F.block<3,3>(0,0) = Frr;  // 位置对位置
    F.block<3,3>(0,3) = Frv;  // 速度对位置
    // 速度误差相关
    F.block<3,3>(3,0) = Fvr;  // 位置对速度
    F.block<3,3>(3,3) = Fvv;  // 速度对速度
    F.block<3,3>(3,6) = Fvt;  // 姿态对速度
    F.block<3,3>(3,12) = Fvba;  // 加速度计零偏对速度
    // 姿态误差相关
    F.block<3,3>(6,0) = Ftr;  // 位置对姿态
    F.block<3,3>(6,3) = Ftv;  // 速度对姿态
    F.block<3,3>(6,6) = Ftt;  // 姿态对姿态
    F.block<3,3>(6,9) = Ftbg;  // 陀螺零偏对姿态
    // 陀螺零偏相关（一阶马尔可夫过程）
    F.block<3,3>(9,9) = -1*Eigen::Matrix3d::Identity()/2;
    // 加速度计零偏相关（一阶马尔可夫过程）
    F.block<3,3>(12,12) = -1*Eigen::Matrix3d::Identity()/2;

    // ========== 6. 计算状态转移矩阵 Phi ==========
    // 使用一阶近似：Phi ≈ I + F*dt
    // 对于小时间步长，这是状态转移矩阵的线性化近似
    Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(15,15);
    Phi = Phi + F * dt;

    return Phi;
}

/**
 * @brief 构建过程噪声输入矩阵 G
 * 
 * 过程噪声向量：w = [w_a, w_g, w_bg, w_ba]^T (12维)
 * 其中：
 *   - w_a: 加速度计白噪声 (3维)
 *   - w_g: 陀螺白噪声 (3维)
 *   - w_bg: 陀螺零偏驱动噪声 (3维)
 *   - w_ba: 加速度计零偏驱动噪声 (3维)
 * 
 * 过程噪声传播方程：Q = G * Q_noise * G^T
 * 其中Q_noise是12x12的噪声协方差矩阵
 * 
 * 矩阵结构：
 *   G = [0    0    0    0  ]
 *       [C_bn 0    0    0  ]  <- 加速度计噪声投影到导航系
 *       [0    C_bn 0    0  ]  <- 陀螺噪声投影到载体系（姿态误差定义在载体系）
 *       [0    0    I    0  ]  <- 陀螺零偏驱动噪声
 *       [0    0    0    I  ]  <- 加速度计零偏驱动噪声
 * 
 * @param insState 当前INS状态
 * @return 15x12的过程噪声输入矩阵
 */
Eigen::MatrixXd GILooseFilter::buildG(const InsState& insState)
{
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(15,12);
    // 加速度计白噪声投影到导航坐标系（影响速度误差）
    G.block<3,3>(3,0) = insState.quat.toRotationMatrix();
    // 陀螺白噪声投影到载体坐标系（影响姿态误差，姿态误差定义在载体系）
    G.block<3,3>(6,3) = insState.quat.toRotationMatrix();
    // 陀螺零偏驱动噪声（直接影响陀螺零偏误差）
    G.block<3,3>(9,6) = Eigen::MatrixXd::Identity(3,3);
    // 加速度计零偏驱动噪声（直接影响加速度计零偏误差）
    G.block<3,3>(12,9) = Eigen::MatrixXd::Identity(3,3);
    return G;
}
