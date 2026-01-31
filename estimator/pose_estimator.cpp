#include "pose_estimator.h"

#include <sophus/so3.hpp>

#include "utils/constants.h"
#include "utils/earth_model.h"
#include "utils/logger.h"
#include "utils/exceptions.h"
#include "measurement/imu_time_aligner.h"

/**
 * @brief 构造函数
 * 
 * @param config GINS配置
 * @throw ValidationException 如果配置无效
 */
PoseEstimator::PoseEstimator(const GinsConfig &config) : state_(config.buildNominalState()),
                                                         filter_(config.buildImuNoise(), config.buildGnssNoise(),
                                                                 config.buildInitialStateNoise()),antlever_(config.buildAntlever())
{
    // 验证初始状态
    if (std::abs(state_.quat.norm() - 1.0) > 1e-6) {
        throw mems_ins_gps::ValidationException("initial_state.attitude", 
            "Quaternion is not normalized");
    }

    // 验证位置范围
    if (state_.pos[0] < -M_PI/2 || state_.pos[0] > M_PI/2) {
        throw mems_ins_gps::ValidationException("initial_state.position[0]", 
            "Latitude must be in [-90°, 90°]");
    }
    if (state_.pos[1] < -M_PI || state_.pos[1] > M_PI) {
        throw mems_ins_gps::ValidationException("initial_state.position[1]", 
            "Longitude must be in [-180°, 180°]");
    }

    // 验证杆臂向量的大小（假设不应超过10米）
    if (antlever_.norm() > 10.0) {
        throw mems_ins_gps::ValidationException("antlever", 
            "Antenna lever arm too large (> 10 m)");
    }
}

void PoseEstimator::addGnssData(const GnssData &data)
{
    gnssData_ = data;
}

void PoseEstimator::addImuData(const ImuData &data)
{
    if (!isInit_)
    {
        currImuData_ = data;
        prevImuData_ = currImuData_;
        isInit_ = true;
        pins_.init(NominalState2InsState(state_), prevImuData_);
        return;
    }

    prevImuData_ = currImuData_;
    currImuData_ = data;
}

/**
 * @brief 更新姿态估计（组合导航主循环）
 * 
 * 根据IMU和GNSS数据的时间对齐情况，执行不同的更新策略：
 * 
 * 1. Curr模式：GNSS时间在当前IMU时间之前
 *    - 先进行GNSS更新和状态校正
 *    - 再进行IMU传播
 * 
 * 2. Next模式：GNSS时间在当前IMU时间之后
 *    - 先进行IMU传播到当前时刻
 *    - 再进行GNSS更新和状态校正
 * 
 * 3. Internal模式：GNSS时间在当前IMU时间区间内
 *    - 先传播到GNSS时间点
 *    - 进行GNSS更新和状态校正
 *    - 再传播剩余时间到当前IMU时间
 * 
 * 4. No模式：没有GNSS数据或时间差太大
 *    - 只进行IMU传播
 * 
 * 这种时间对齐策略确保GNSS更新在正确的时刻进行，提高组合导航精度。
 */
void PoseEstimator::updatePose()
{
    switch (ImuTimeAligner::getMode(prevImuData_, currImuData_, gnssData_, TIME_ALIGN_ERR))
    {
        case ImuTimeAligner::Mode::Curr:
        {
            // GNSS时间在当前IMU时间之前，先更新再传播
            gnssUpdate();
            correctPose();
            propagate(currImuData_);
            break;
        }
        case ImuTimeAligner::Mode::Next:
        {
            // GNSS时间在当前IMU时间之后，先传播再更新
            propagate(currImuData_);
            gnssUpdate();
            correctPose();
            break;
        }
        case ImuTimeAligner::Mode::Internal:
        {
            // GNSS时间在IMU时间区间内，需要插值
            auto interpImuRes = ImuTimeAligner::getInterpResult(prevImuData_, currImuData_, gnssData_.time);
            auto midImu = interpImuRes.mid;

            // 传播到GNSS时间点
            propagate(midImu);
            // 在GNSS时间点进行更新
            gnssUpdate();
            correctPose();
            // 继续传播到当前IMU时间
            propagate(interpImuRes.remain);
            break;
        }
        case ImuTimeAligner::Mode::No:
        {
            // 没有GNSS数据或时间差太大，只进行IMU传播
            propagate(currImuData_);
            break;
        }
    }
}

NominalState PoseEstimator::getPose() const
{
    return state_;
}

/**
 * @brief 补偿IMU零偏
 * 
 * 使用当前估计的零偏值对IMU测量值进行补偿：
 *   a_compensated = a_raw - ba * dt
 *   ω_compensated = ω_raw - bg * dt
 * 
 * 注意：这里使用零偏的变化率（ba*dt, bg*dt）进行补偿，
 * 假设零偏在时间间隔dt内线性变化。
 * 
 * @param imuData 原始IMU数据
 * @return 补偿后的IMU数据
 */
ImuData PoseEstimator::comPenImu(const ImuData &imuData) const
{
    double dt = imuData.time - prevImuData_.time;
    ImuData data;
    data.time = imuData.time;
    // 补偿加速度计零偏
    data.accel = imuData.accel - state_.ba * dt;
    // 补偿陀螺零偏
    data.gyro = imuData.gyro - state_.bg * dt;
    return data;
}

/**
 * @brief 状态传播（时间更新）
 * 
 * 使用IMU数据进行状态传播，包括：
 * 1. INS解算：更新名义状态（位置、速度、姿态）
 * 2. 误差状态传播：更新误差状态和协方差矩阵
 * 3. 同步：将INS解算结果同步到名义状态
 * 
 * 这是误差状态卡尔曼滤波（ESKF）的核心：名义状态由INS解算得到，
 * 误差状态由卡尔曼滤波估计得到。
 * 
 * @param imuData IMU测量数据
 */
void PoseEstimator::propagate(const ImuData &imuData)
{
    double dt = imuData.time - prevImuData_.time;
    
    // 验证时间间隔
    if (dt <= 0.0) {
        throw mems_ins_gps::TimeException("Invalid time interval: dt=" + std::to_string(dt) + "s");
    }
    if (dt > 1.0) {
        throw mems_ins_gps::TimeException("Time interval too large: dt=" + std::to_string(dt) + "s");
    }

    // 验证IMU数据有效性
    if (!std::isfinite(imuData.accel.norm()) || !std::isfinite(imuData.gyro.norm())) {
        throw mems_ins_gps::DataException("IMU data contains NaN or Inf");
    }
    
    // 1. 补偿IMU零偏
    auto comPenImuData = comPenImu(imuData);

    // 2. INS解算：更新名义状态（位置、速度、姿态）
    pins_.insUpdate(comPenImuData);

    // 3. 误差状态传播：更新误差状态和协方差矩阵
    filter_.predict(pins_.getState(), comPenImuData, dt);

    // 4. 同步：将INS解算结果同步到名义状态
    // 在ESKF中，名义状态由INS解算得到，误差状态由滤波器估计
    syncFromINS();

    prevImuData_ = imuData;  // 更新时间戳
}

void PoseEstimator::gnssUpdate()
{
    bool valid = gnssData_.valid;
    if (!valid) return;

    // 验证GNSS数据有效性
    if (!std::isfinite(gnssData_.pos.x()) || !std::isfinite(gnssData_.pos.y()) || 
        !std::isfinite(gnssData_.pos.z())) {
        throw mems_ins_gps::DataException("GNSS position contains NaN or Inf");
    }

    // 验证GNSS位置范围
    if (gnssData_.pos[0] < -90 || gnssData_.pos[0] > 90) {
        throw mems_ins_gps::ValidationException("gnss.pos[0]", 
            "GNSS latitude must be in [-90°, 90°]");
    }
    if (gnssData_.pos[1] < -180 || gnssData_.pos[1] > 180) {
        throw mems_ins_gps::ValidationException("gnss.pos[1]", 
            "GNSS longitude must be in [-180°, 180°]");
    }

    Eigen::Vector3d fixCurrPos = state_.pos + EarthModel::NDE2LLhMatrix(state_.pos[0], state_.pos[2]) * state_.quat.
                                 toRotationMatrix() * antlever_;

    Eigen::Vector3d posErr = fixCurrPos - gnssData_.pos;
    LOG_DEBUG_STREAM() << "curr pos:" << state_.pos.transpose();
    LOG_DEBUG_STREAM() << "gnss pos:" << gnssData_.pos.transpose();

    Eigen::Vector3d dz = EarthModel::LLh2NEDMatrix(state_.pos[0], state_.pos[2]) * posErr;

    LOG_DEBUG_STREAM() << "dz:" << dz.transpose();

    filter_.update(dz);
    gnssData_.valid = false;
}

/**
 * @brief 校正名义状态（误差注入）
 * 
 * 将误差状态的估计值注入到名义状态，完成状态校正：
 * 
 * 1. 位置校正：pos = pos - T * δr
 *    - δr是NED坐标系下的位置误差，需要转换到LLH坐标系
 * 
 * 2. 速度校正：v = v - δv
 *    - δv是NED坐标系下的速度误差，直接相减
 * 
 * 3. 姿态校正：q = exp(δφ) * q
 *    - δφ是导航坐标系下的姿态误差（左扰动）
 *    - 使用指数映射将旋转向量转换为四元数
 * 
 * 4. 零偏校正：bg = bg + δbg, ba = ba + δba
 *    - 零偏误差直接相加
 * 
 * 校正后需要：
 * - 重置误差状态（ESKF特性：误差状态在注入后应重置为零）
 * - 重置INS状态（使INS状态与校正后的名义状态一致）
 */
void PoseEstimator::correctPose()
{
    auto dx = filter_.getState();
    
    // 验证误差状态的有效性
    if (!std::isfinite(dx.posDelta.norm()) || !std::isfinite(dx.velDelta.norm()) ||
        !std::isfinite(dx.attDelta.norm()) || !std::isfinite(dx.bg.norm()) ||
        !std::isfinite(dx.ba.norm())) {
        throw mems_ins_gps::ValidationException("error_state", 
            "Error state contains NaN or Inf");
    }

    // 验证误差状态的大小（防止异常大的修正）
    if (dx.posDelta.norm() > 1000) {  // 位置误差不应超过1km
        throw mems_ins_gps::ValidationException("error_state.posDelta", 
            "Position error correction too large (> 1000 m)");
    }
    if (dx.velDelta.norm() > 100) {  // 速度误差不应超过100 m/s
        throw mems_ins_gps::ValidationException("error_state.velDelta", 
            "Velocity error correction too large (> 100 m/s)");
    }
    if (dx.attDelta.norm() > M_PI) {  // 姿态误差不应超过180度
        throw mems_ins_gps::ValidationException("error_state.attDelta", 
            "Attitude error correction too large (> 180°)");
    }
    
    // 1. 位置校正：将NED坐标系下的位置误差转换到LLH坐标系后校正
    state_.pos -= EarthModel::NDE2LLhMatrix(state_.pos[0], state_.pos[2]) * dx.posDelta;
    
    // 2. 速度校正：直接相减（都在NED坐标系）
    state_.vel -= dx.velDelta;
    
    // 3. 零偏校正：直接相加
    state_.bg += dx.bg;
    state_.ba += dx.ba;

    // 4. 姿态校正：使用左扰动模式
    // 姿态误差定义在导航，使用左乘：q_new = exp(δφ) * q_old
    state_.quat = Sophus::SO3d::exp(dx.attDelta).unit_quaternion() * state_.quat;
    state_.quat.normalize();
    
    // 验证校正后的状态有效性
    if (std::abs(state_.quat.norm() - 1.0) > 1e-6) {
        throw mems_ins_gps::ValidationException("state.quat", 
            "Quaternion normalization failed after correction");
    }

    // 5. 重置误差状态和INS状态
    // ESKF特性：误差状态在注入后应重置为零，避免重复注入
    filter_.reset();
    // 重置INS状态，使其与校正后的名义状态一致
    pins_.resetState(NominalState2InsState(state_));
}

/**
 * @brief 从INS同步名义状态
 * 
 * 在ESKF中，名义状态由INS解算得到，误差状态由滤波器估计。
 * 在每次传播后，需要将INS解算的结果同步到名义状态。
 * 
 * 注意：这里只同步位置、速度、姿态，不同步零偏。
 * 零偏由滤波器估计，在GNSS更新时通过correctPose()注入。
 */
void PoseEstimator::syncFromINS()
{
    auto s = pins_.getState();
    state_.pos = s.pos;
    state_.vel = s.vel;
    state_.quat = s.quat;
}
