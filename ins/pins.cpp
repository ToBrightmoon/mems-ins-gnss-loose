#include "pins.h"

#include <sophus/so3.hpp>

#include "utils/earth_model.h"
#include "utils/constants.h"
#include "utils/logger.h"
#include "utils/exceptions.h"

/**
 * @brief 初始化INS
 * 
 * @param state 初始INS状态
 * @param pre 初始IMU数据
 * @throw ValidationException 如果初始状态无效
 */
void Pins::init(const InsState & state,const ImuData & pre)
{
    // 验证初始位置范围
    if (state.pos[0] < -M_PI/2 || state.pos[0] > M_PI/2) {
        throw mems_ins_gps::ValidationException("initial_state.pos[0]", 
            "Latitude must be in [-90°, 90°]");
    }
    if (state.pos[1] < -M_PI || state.pos[1] > M_PI) {
        throw mems_ins_gps::ValidationException("initial_state.pos[1]", 
            "Longitude must be in [-180°, 180°]");
    }
    if (state.pos[2] < -1000 || state.pos[2] > 100000) {
        throw mems_ins_gps::ValidationException("initial_state.pos[2]", 
            "Height must be in [-1000m, 100000m]");
    }

    // 验证速度范围（假设最大速度不超过1000 m/s）
    if (state.vel.norm() > 1000.0) {
        throw mems_ins_gps::ValidationException("initial_state.velocity", 
            "Velocity magnitude too large (> 1000 m/s)");
    }

    // 验证四元数归一化
    if (std::abs(state.quat.norm() - 1.0) > 1e-6) {
        throw mems_ins_gps::ValidationException("initial_state.attitude", 
            "Quaternion is not normalized");
    }

    // 验证IMU数据时间戳
    if (pre.time < 0) {
        throw mems_ins_gps::ValidationException("initial_imu.time", 
            "IMU time must be non-negative");
    }

    currState_ = state;
    prevState_ = currState_;

    currImuData_ = pre;
    preImuData_ = pre;
}

void Pins::insUpdate(const ImuData & curr)
{
    if (!checkTime(curr))
    {
        double dt = curr.time - preImuData_.time;
        std::ostringstream oss;
        oss << "Invalid IMU time interval: dt=" << dt 
            << "s (expected: 0 < dt <= " << MinTimeDelta << "s)";
        throw mems_ins_gps::TimeException(oss.str());
    }
    
    currImuData_ = curr;
    const double dt = currImuData_.time - preImuData_.time;

    try {
        propagate(dt);
    } catch (const std::exception& e) {
        LOG_ERROR_STREAM() << "Error in INS propagation: " << e.what();
        throw;
    }

    prevState_ = currState_;
    preImuData_ = currImuData_;
}

InsState Pins::getState() const
{
    return currState_;
}

void Pins::resetState(const InsState & state)
{
    prevState_ = state;
    currState_ = state;
}

/**
 * @brief 状态传播
 * 
 * @param dt 时间间隔 (s)
 * @throw ValidationException 如果时间间隔无效
 */
void Pins::propagate(double dt)
{
    // 验证时间间隔
    if (dt <= 0.0) {
        throw mems_ins_gps::ValidationException("dt", 
            "Time interval must be positive");
    }
    if (dt > MinTimeDelta) {
        throw mems_ins_gps::ValidationException("dt", 
            "Time interval too large (> " + std::to_string(MinTimeDelta) + "s)");
    }

#ifdef USE_EASY_PINS
    updateVelocityByMid(dt);

    updatePositionByMid(dt);

    updateAttitudeByMid(dt);
#else
    updateVelocity(dt);

    updatePosition(dt);

    updateAttitude(dt);
#endif

}

/**
 * @brief 更新速度（使用中点法进行二阶积分）
 * 
 * 速度更新方程：v = v0 + ∫(f_n + g_n - (2*ω_ie + ω_en) × v) dt
 * 其中：
 *   - f_n: 比力在导航系的投影
 *   - g_n: 重力加速度
 *   - ω_ie: 地球自转角速度
 *   - ω_en: 运移角速度
 * 
 * 算法采用中点法（midpoint method）进行二阶积分，提高精度：
 * 1. 使用前一时刻的参数计算初步速度增量
 * 2. 计算中间时刻的位置和导航参数
 * 3. 使用中间时刻的参数重新计算速度增量
 * 4. 更新速度
 * 
 * @param dt 时间间隔 (s)
 */
void Pins::updateVelocity(double dt)
{
    // 1. 计算之前时刻的地球模型参数
    // 包括：子午圈半径Rm、卯酉圈半径Rn、地球自转角速度投影wie_n、运移角速度投影wen_n
    auto prevNavParam = EarthModel::computeNavParam(prevState_.pos[0],prevState_.pos[2],prevState_.vel[0],prevState_.vel[1]);
    auto prevGravity = EarthModel::getGravity(prevState_.pos[0],prevState_.pos[2]);

    // 2. 计算Sculling补偿项（考虑角速度和加速度的耦合效应）
    // Sculling效应：当载体同时存在角运动和线运动时，需要补偿的交叉项
    // 一阶补偿：0.5 * (ω × a)
    // 二阶补偿：1/12 * (ω × a) 和 1/12 * (a × ω)
    Eigen::Vector3d comPenOne = currImuData_.gyro.cross(currImuData_.accel) / 2;
    Eigen::Vector3d comPenTwo = currImuData_.gyro.cross(currImuData_.accel)/12;
    Eigen::Vector3d comPenThree = currImuData_.accel.cross(currImuData_.gyro)/12;

    // 3. 计算载体坐标系下的比力（包含Sculling补偿）
    // 比力 = 加速度计测量值 + Sculling补偿项
    Eigen::Vector3d vfB = currImuData_.accel + comPenOne + comPenTwo + comPenThree;
    
    // 计算导航系旋转角速度（地球自转 + 运移角速度）在中间时刻的投影
    Eigen::Vector3d midThetaQnn = -(prevNavParam.wie_n + prevNavParam.wen_n)*dt*0.5;
    
    // 将比力从载体坐标系转换到导航坐标系
    // 考虑导航系本身的旋转（地球自转和运移）
    Eigen::Vector3d vfN = Sophus::SO3d::exp(midThetaQnn).matrix() * prevState_.quat.toRotationMatrix() * vfB;

    // 4. 计算重力加速度和哥式力（Coriolis force）
    // 哥式力：-2*(ω_ie + ω_en) × v，由地球自转和载体运动引起
    // 重力方向：向下（NED坐标系中为[0,0,g]）
    Eigen::Vector3d gVgN = (Eigen::Vector3d{0,0,prevGravity} - (2 * prevNavParam.wie_n + prevNavParam.wen_n).cross(prevState_.vel))*dt;

    // 5. 计算中间时刻的速度（用于后续计算中间时刻的位置）
    Eigen::Vector3d midVel = prevState_.vel + (vfN + gVgN)*0.5;

    // 6. 计算中间时刻的位置（LLH坐标系）
    // 需要计算：导航系旋转四元数qnn、地心地固系旋转四元数qee、导航系到地心系四元数qne
    Eigen::Quaterniond midQnn = Sophus::SO3d::exp(-midThetaQnn).unit_quaternion().normalized();
    auto midThetaQee = Eigen::Vector3d{0,0,-0.5*EarthModel::getWie()*dt};  // 地心系旋转（地球自转）
    Eigen::Quaterniond midQee = Sophus::SO3d::exp(midThetaQee).unit_quaternion();
    auto preQne = EarthModel::getQne(prevState_.pos[0],prevState_.pos[1]);
    auto midQne = midQnn * preQne * midQee;  // 组合旋转：n系->e系
    auto midH = prevState_.pos[2] - 0.5*midVel[2]*dt;  // 中间时刻高度
    Eigen::Vector3d midPos = EarthModel::getLLhByQne(midQne,midH);

    // 7. 使用中间时刻的位置和速度重新计算地球模型参数
    auto midNavParam = EarthModel::computeNavParam(midPos[0],midPos[2],midVel[0],midVel[1]);

    // 8. 使用中间时刻的导航参数重新计算比力在导航系的投影
    midThetaQnn = -(midNavParam.wie_n + midNavParam.wen_n)*dt*0.5;
    vfN = Sophus::SO3d::exp(midThetaQnn).matrix() * prevState_.quat.toRotationMatrix() * vfB;

    // 9. 使用中间时刻的参数重新计算重力和哥式力
    auto midGravity = EarthModel::getGravity(midPos[0],midPos[2]);
    gVgN = (Eigen::Vector3d{0,0,midGravity} - (2 * midNavParam.wie_n + midNavParam.wen_n).cross(midVel))*dt;

    // 10. 更新速度：v_k = v_{k-1} + Δv（比力增量 + 重力哥式力增量）
    currState_.vel = prevState_.vel + vfN + gVgN;
}

/**
 * @brief 更新位置（使用中点法）
 * 
 * 位置更新需要考虑：
 * 1. 导航坐标系（n系）随载体位置变化而旋转（运移角速度）
 * 2. 地心地固坐标系（e系）随地球自转而旋转
 * 3. 位置在LLH坐标系下的表示（纬度、经度、高度）
 * 
 * 算法流程：
 * 1. 计算中间时刻的速度和位置
 * 2. 计算导航系旋转四元数qnn（由运移角速度引起）
 * 3. 计算地心系旋转四元数qee（由地球自转引起）
 * 4. 更新导航系到地心系的四元数qne
 * 5. 从qne反算得到新的LLH位置
 * 
 * @param dt 时间间隔 (s)
 */
void Pins::updatePosition(double dt)
{
    // 1. 获取中间时刻的速度和位置（用于计算中间时刻的地球模型参数）
    Eigen::Vector3d midVel = (prevState_.vel + currState_.vel) / 2.0;
    // 使用NED到LLH的转换矩阵，将速度增量转换为位置增量
    Eigen::Vector3d midPos = prevState_.pos + EarthModel::NDE2LLhMatrix(prevState_.pos[0],prevState_.pos[2]) * midVel * dt * 0.5;

    // 2. 获取中间时刻的地球模型参数（用于计算运移角速度）
    auto naviParam = EarthModel::computeNavParam(midPos[0],midPos[2],midVel[0],midVel[1]);

    // 3. 计算导航系到地心系的四元数qne
    // qne表示从导航系（n）到地心地固系（e）的旋转
    Eigen::Quaterniond preQne = EarthModel::getQne(prevState_.pos[0],prevState_.pos[1]);
    
    // qnn: 导航系从k-1到k时刻的旋转（由运移角速度引起）
    auto qnn = EarthModel::getQnn(naviParam,dt,1);
    
    // qee: 地心系从k到k-1时刻的旋转（由地球自转引起，注意方向）
    auto qee = EarthModel::getQee(dt,-1);
    
    // 更新qne: qne_k = qee * qne_{k-1} * qnn
    // 这表示：n_k -> n_{k-1} -> e_{k-1} -> e_k
    auto currQne = qee * preQne * qnn;
    currQne = currQne.normalized();

    // 4. 利用qne和高度更新位置
    // 高度更新：h_k = h_{k-1} - v_d * dt（NED坐标系中，向下为正）
    double h = prevState_.pos[2] - midVel[2]*dt;
    // 从qne反算得到新的经纬度
    currState_.pos = EarthModel::getLLhByQne(currQne,h);
}

/**
 * @brief 更新姿态（使用中点法）
 * 
 * 姿态更新方程：q_{b->n}_k = q_{n_{k-1}->n_k} * q_{b->n}_{k-1} * q_{b_{k-1}->b_k}
 * 其中：
 *   - q_{b->n}: 从载体坐标系到导航坐标系的旋转四元数
 *   - q_{n_{k-1}->n_k}: 导航系从k-1到k时刻的旋转（由运移角速度引起）
 *   - q_{b_{k-1}->b_k}: 载体系从k-1到k时刻的旋转（由角速度积分得到，包含Coning补偿）
 * 
 * Coning补偿：当载体存在圆锥运动时，需要补偿的交叉项
 * 
 * @param dt 时间间隔 (s)
 */
void Pins::updateAttitude(double dt)
{
    // 1. 计算中间时刻的速度（用于计算中间时刻的地球模型参数）
    Eigen::Vector3d midVel = (prevState_.vel + currState_.vel) / 2.0;

    // 2. 利用qne计算中间时刻的位置
    // 通过前后时刻的qne插值得到中间时刻的qne，进而得到中间位置
    auto preQne = EarthModel::getQne(prevState_.pos[0],prevState_.pos[1]);
    auto currQne = EarthModel::getQne(currState_.pos[0],currState_.pos[1]);
    auto qee = EarthModel::getQee(dt,-1);  // 地心系从k到k-1时刻的旋转
    
    // 计算导航系从k-1到k时刻的旋转角（通过qne的变化反推）
    Eigen::Vector3d midThetaQnn = Sophus::SO3d((preQne.inverse() * qee.inverse() * currQne)).log();
    Eigen::Quaterniond midQnn = Sophus::SO3d::exp(midThetaQnn*0.5).unit_quaternion();
    
    // 计算中间时刻的qne
    auto midQee = EarthModel::getQee(dt*0.5,-1);
    auto midQne = (midQee * preQne * midQnn);
    midQne = midQne.normalized();
    
    // 从qne反算中间位置
    Eigen::Vector3d midPos = (prevState_.pos + currState_.pos) * 0.5;
    midPos = EarthModel::getLLhByQne(midQne,midPos[2]);

    // 3. 使用中间时刻的位置和速度重新计算地球模型参数
    auto naviParam = EarthModel::computeNavParam(midPos[0],midPos[2],midVel[0],midVel[1]);

    // 4. 计算导航系和载体系的旋转四元数
    // qnn: 导航系从k到k-1时刻的旋转（注意方向，用于更新方程）
    auto qnn = EarthModel::getQnn(naviParam,dt,-1);
    
    // qbb: 载体系从k-1到k时刻的旋转（由角速度积分得到）
    // 包含Coning补偿：θ = ω + 1/12 * (ω_{k-1} × ω_k)
    // 这是对圆锥运动的一阶补偿
    auto thetaQbb = currImuData_.gyro + preImuData_.gyro.cross(currImuData_.gyro)/12;
    Eigen::Quaterniond qbb = Sophus::SO3d::exp(thetaQbb).unit_quaternion();

    // 5. 更新姿态四元数
    // q_{b->n}_k = q_{n_k->n_{k-1}} * q_{b->n}_{k-1} * q_{b_{k-1}->b_k}
    // 注意：qnn是n_k->n_{k-1}，所以顺序是 qnn * q_{k-1} * qbb
    currState_.quat = (qnn * prevState_.quat * qbb);
    currState_.quat = currState_.quat.normalized();
}

/**
 * @brief 更新姿态（简化版本，使用中点法）
 * 
 * 这是简化版本的姿态更新，使用前一时刻的参数计算导航系旋转。
 * 相比完整版本，计算量更小但精度略低。
 * 
 * @param dt 时间间隔 (s)
 */
void Pins::updateAttitudeByMid(double dt)
{
    // 计算载体系旋转增量（包含Coning补偿）
    // Coning补偿项：1/12 * (ω_{k-1} × ω_k)，用于补偿圆锥运动
    Eigen::Vector3d dtheta =
        currImuData_.gyro +
        (1.0 / 12.0) * preImuData_.gyro.cross(currImuData_.gyro);

    // 将旋转增量转换为四元数
    Eigen::Quaterniond qbb =
        Sophus::SO3d::exp(dtheta).unit_quaternion();

    // 计算导航系旋转（使用前一时刻的参数，简化计算）
    auto nav = EarthModel::computeNavParam(
        prevState_.pos[0],
        prevState_.pos[2],
        prevState_.vel[0],
        prevState_.vel[1]);

    // qnn: 导航系从k到k-1时刻的旋转（注意方向）
    auto qnn = EarthModel::getQnn(nav,dt,-1);

    // 更新姿态：q_{b->n}_k = q_{n_k->n_{k-1}} * q_{b->n}_{k-1} * q_{b_{k-1}->b_k}
    currState_.quat =
        (qnn * prevState_.quat * qbb).normalized();
}

/**
 * @brief 更新速度（简化版本）
 * 
 * 使用前一时刻的参数进行速度更新，计算量小但精度略低。
 * 包含Sculling补偿和哥式力补偿。
 * 
 * @param dt 时间间隔 (s)
 */
void Pins::updateVelocityByMid(double dt)
{
    // 计算前一时刻的地球模型参数
    auto nav = EarthModel::computeNavParam(
        prevState_.pos[0],
        prevState_.pos[2],
        prevState_.vel[0],
        prevState_.vel[1]);

    // 获取重力加速度
    double g = EarthModel::getGravity(
        prevState_.pos[0],
        prevState_.pos[2]);

    // Sculling补偿（一阶补偿）
    // 当载体同时存在角运动和线运动时，需要补偿的交叉项
    // 一阶项：0.5 * (ω × a)
    Eigen::Vector3d dvB =
        currImuData_.accel +
        0.5 * currImuData_.gyro.cross(currImuData_.accel);

    // 将比力增量从载体坐标系转换到导航坐标系
    Eigen::Vector3d dvN =
        prevState_.quat.toRotationMatrix() * dvB;

    // 计算哥式力（Coriolis force）
    // 哥式力 = -2*(ω_ie + ω_en) × v
    Eigen::Vector3d coriolis =
        -(2.0 * nav.wie_n + nav.wen_n)
            .cross(prevState_.vel) * dt;

    // 重力加速度增量（NED坐标系中向下为正）
    Eigen::Vector3d gravity{0, 0, g * dt};

    // 更新速度：v_k = v_{k-1} + Δv（比力增量 + 重力增量 + 哥式力增量）
    currState_.vel =
        prevState_.vel + dvN + gravity + coriolis;
}

/**
 * @brief 更新位置（简化版本）
 * 
 * 使用中点速度进行位置更新，使用前一时刻的转换矩阵。
 * 
 * @param dt 时间间隔 (s)
 */
void Pins::updatePositionByMid(double dt)
{
    // 计算中点速度
    Eigen::Vector3d v_mid =
        0.5 * (prevState_.vel + currState_.vel);

    // 获取NED到LLH的转换矩阵（使用前一时刻的位置）
    Eigen::Matrix3d T =
        EarthModel::NDE2LLhMatrix(
            prevState_.pos[0],
            prevState_.pos[2]);

    // 更新位置：LLH_k = LLH_{k-1} + T * v_mid * dt
    // T矩阵将NED坐标系下的速度转换为LLH坐标系下的位置增量
    currState_.pos =
        prevState_.pos + T * v_mid * dt;
}

bool Pins::checkTime(const ImuData& data) const
{
    double dt = data.time - preImuData_.time;

    return (dt > 0.0 && dt <= MinTimeDelta);
}
