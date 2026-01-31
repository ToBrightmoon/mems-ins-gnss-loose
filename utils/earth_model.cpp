#include "earth_model.h"

#include <sophus/so3.hpp>
#include "utils/exceptions.h"

EarthModel::Ellipsoid const EarthModel::wgs84;

/**
 * @brief 计算导航系参数
 * 
 * 计算给定位置和速度下的导航坐标系参数，包括：
 * 1. 地球曲率半径（子午圈半径Rm、卯酉圈半径Rn）
 * 2. 地球自转角速度在导航系的投影（wie_n）
 * 3. 运移角速度在导航系的投影（wen_n）
 * 
 * 这些参数用于：
 * - 位置更新时的坐标转换
 * - 速度更新时的哥式力计算
 * - 姿态更新时的导航系旋转计算
 * 
 * @param latRad 纬度（弧度）
 * @param h 高度（米）
 * @param vn 北向速度（m/s）
 * @param ve 东向速度（m/s）
 * @return 导航系参数结构
 */
EarthModel::NavEarthParam EarthModel::computeNavParam(double latRad, double h, double vn, double ve)
{
    // 验证输入参数范围
    if (latRad < -M_PI/2 || latRad > M_PI/2) {
        throw mems_ins_gps::ValidationException("latRad", 
            "Latitude must be in [-90°, 90°]");
    }
    if (h < -1000 || h > 100000) {
        throw mems_ins_gps::ValidationException("h", 
            "Height must be in [-1000m, 100000m]");
    }
    if (!std::isfinite(vn) || !std::isfinite(ve)) {
        throw mems_ins_gps::ValidationException("velocity", 
            "Velocity must be finite");
    }
    if (std::abs(vn) > 1000 || std::abs(ve) > 1000) {
        throw mems_ins_gps::ValidationException("velocity", 
            "Velocity too large (> 1000 m/s)");
    }

    NavEarthParam res;

    const double sin_phi = std::sin(latRad);
    const double cos_phi = std::cos(latRad);
    const double tan_phi = std::tan(latRad);

    // 1. 计算地球曲率半径
    // Rn: 卯酉圈半径（东西方向）
    // Rm: 子午圈半径（南北方向）
    // 公式基于WGS84椭球模型
    double den = 1.0 - wgs84.e2 * sin_phi * sin_phi;
    res.Rn = wgs84.a / std::sqrt(den);  // 卯酉圈半径
    res.Rm = res.Rn * (1.0 - wgs84.e2) / den;  // 子午圈半径

    // 考虑高度后的曲率半径
    res.Rmh = res.Rm + h;  // 子午圈半径 + 高度
    res.Rnh = res.Rn + h;  // 卯酉圈半径 + 高度

    // 2. 地球自转角速度在导航系的投影（NED坐标系）
    // 地球自转轴指向北极，在NED坐标系中的投影为：
    // [ω_ie * cos(φ), 0, -ω_ie * sin(φ)]
    // 其中：北向分量 = ω_ie * cos(φ)，地向分量 = -ω_ie * sin(φ)
    res.wie_n << wgs84.wie * cos_phi,
                 0.0,
                 -wgs84.wie * sin_phi;

    // 3. 运移角速度在导航系的投影（NED坐标系）
    // 运移角速度由载体运动引起，导航坐标系随载体位置变化而旋转
    // 北向分量：ω_en_N = ve / (Rn + h)  （东向速度引起的北向旋转）
    // 东向分量：ω_en_E = -vn / (Rm + h) （北向速度引起的东向旋转，负号表示方向）
    // 地向分量：ω_en_D = -ve * tan(φ) / (Rn + h) （东向速度引起的地向旋转）
    res.wen_n << ve / res.Rnh,
                -vn / res.Rmh,
                -(ve / res.Rnh) * tan_phi;

    return res;
}

Eigen::Vector2d EarthModel::computeNavRmRn(double latRad)
{
    // 验证输入参数范围
    if (latRad < -M_PI/2 || latRad > M_PI/2) {
        throw mems_ins_gps::ValidationException("latRad", 
            "Latitude must be in [-90°, 90°]");
    }

    Eigen::Vector2d res;
    const double sin_phi = std::sin(latRad);

    // 1. 计算曲率半径 Rm, Rn
    double den = 1.0 - wgs84.e2 * sin_phi * sin_phi;
    res[1] = wgs84.a / std::sqrt(den);
    res[0] = res[1] * (1.0 - wgs84.e2) / den;

    return res;
}

/**
 * @brief 计算重力加速度
 * 
 * 使用Somigliana公式计算给定纬度和高度下的重力加速度。
 * 
 * 公式：
 * 1. 海平面重力：g0 = g_e * (1 + k * sin²(φ)) / sqrt(1 - e² * sin²(φ))
 *    其中：
 *      - g_e: 赤道重力加速度
 *      - k: Somigliana常数
 *      - e²: 第一偏心率平方
 *      - φ: 纬度
 * 
 * 2. 高度修正：g = g0 - 3.086e-6 * h
 *    这是惯导中常用的高精度近似公式，考虑了高度对重力的影响
 * 
 * @param latRad 纬度（弧度）
 * @param height 高度（米）
 * @return 重力加速度（m/s²）
 */
double EarthModel::getGravity(double latRad, double height)
{
    // 验证输入参数范围
    if (latRad < -M_PI/2 || latRad > M_PI/2) {
        throw mems_ins_gps::ValidationException("latRad", 
            "Latitude must be in [-90°, 90°]");
    }
    if (!std::isfinite(height)) {
        throw mems_ins_gps::ValidationException("height", 
            "Height must be finite");
    }

    double sinLat = std::sin(latRad);
    double sinSqLat = sinLat * sinLat;

    // 1. 计算海平面重力（使用Somigliana公式）
    // 这是WGS84标准重力模型，考虑了地球扁率对重力的影响
    double g0 = wgs84.gravityEquator * (1.0 + wgs84.gravityConstantK * sinSqLat)
                / std::sqrt(1.0 - wgs84.e2 * sinSqLat);

    // 2. 高度修正（惯导中常用的高精度近似公式）
    // 重力随高度增加而减小，修正系数约为 3.086e-6 m/s² per meter
    double hCorr = 3.086e-6 * height;
    return g0 - hCorr;  // 返回修正后的重力值
}

/**
 * @brief 计算NED到LLH的转换矩阵
 * 
 * 将NED坐标系下的位置增量转换为LLH坐标系下的位置增量。
 * 
 * 转换关系：
 *   dLLH = T * dNED
 * 
 * 矩阵形式（对角矩阵）：
 *   T = [1/(Rm+h)      0           0    ]
 *       [0             1/((Rn+h)*cos(φ)) 0    ]
 *       [0             0           -1    ]
 * 
 * 物理意义：
 *   - 纬度增量：dLat = dN / (Rm + h)  （考虑子午圈曲率半径）
 *   - 经度增量：dLon = dE / ((Rn + h) * cos(φ))  （考虑卯酉圈曲率半径和纬度）
 *   - 高度增量：dh = -dD  （NED中向下为正，高度向上为正）
 * 
 * @param latRad 纬度（弧度）
 * @param h 高度（米）
 * @return 3x3转换矩阵
 */
Eigen::Matrix3d EarthModel::NDE2LLhMatrix(const double latRad, const double h)
{
    // 验证输入参数范围
    if (latRad < -M_PI/2 || latRad > M_PI/2) {
        throw mems_ins_gps::ValidationException("latRad", 
            "Latitude must be in [-90°, 90°]");
    }
    if (!std::isfinite(h)) {
        throw mems_ins_gps::ValidationException("h", 
            "Height must be finite");
    }

    Eigen::Matrix3d dri = Eigen::Matrix3d::Zero();

    Eigen::Vector2d rmn = computeNavRmRn(latRad);

    dri(0, 0) = 1.0 / (rmn[0] + h);  // 纬度：考虑子午圈曲率半径
    dri(1, 1) = 1.0 / ((rmn[1] + h) * cos(latRad));  // 经度：考虑卯酉圈曲率半径和纬度
    dri(2, 2) = -1;  // 高度：方向相反（NED向下为正，高度向上为正）
    return dri;
}

/**
 * @brief 计算LLH到NED的转换矩阵
 * 
 * 将LLH坐标系下的位置增量转换为NED坐标系下的位置增量。
 * 这是NDE2LLhMatrix的逆矩阵。
 * 
 * 转换关系：
 *   dNED = T * dLLH
 * 
 * 矩阵形式（对角矩阵）：
 *   T = [Rm+h           0           0    ]
 *       [0              (Rn+h)*cos(φ)  0    ]
 *       [0              0           -1    ]
 * 
 * @param latRad 纬度（弧度）
 * @param h 高度（米）
 * @return 3x3转换矩阵
 */
Eigen::Matrix3d EarthModel::LLh2NEDMatrix(const double latRad, const double h)
{
    // 验证输入参数范围
    if (latRad < -M_PI/2 || latRad > M_PI/2) {
        throw mems_ins_gps::ValidationException("latRad", 
            "Latitude must be in [-90°, 90°]");
    }
    if (!std::isfinite(h)) {
        throw mems_ins_gps::ValidationException("h", 
            "Height must be finite");
    }

    Eigen::Matrix3d dr = Eigen::Matrix3d::Zero();

    Eigen::Vector2d rmn = computeNavRmRn(latRad);

    dr(0, 0) = rmn[0] + h;  // 北向：考虑子午圈曲率半径
    dr(1, 1) = (rmn[1] + h) * cos(latRad);  // 东向：考虑卯酉圈曲率半径和纬度
    dr(2, 2) = -1;  // 地向：方向相反
    return dr;
}

double EarthModel::getWie()
{
    return wgs84.wie;
}

/**
 * @brief 计算导航系到地心系的四元数 qne
 * 
 * qne表示从导航坐标系（n，NED）到地心地固坐标系（e，ECEF）的旋转。
 * 
 * 旋转顺序：
 * 1. 绕Z轴旋转经度（从地心系到中间系）
 * 2. 绕Y轴旋转(90°-纬度)（从中间系到导航系）
 * 
 * 四元数表示：qne = q(lon) * q(90°-lat)
 * 
 * 用途：
 * - 位置更新时计算导航系随位置变化的旋转
 * - 从LLH位置计算导航系到地心系的旋转关系
 * 
 * @param latRad 纬度（弧度）
 * @param lonRad 经度（弧度）
 * @return 四元数 qne
 */
Eigen::Quaterniond EarthModel::getQne(const double latRad, const double lonRad)
{
    // 验证输入参数范围
    if (latRad < -M_PI/2 || latRad > M_PI/2) {
        throw mems_ins_gps::ValidationException("latRad", 
            "Latitude must be in [-90°, 90°]");
    }
    if (lonRad < -M_PI || lonRad > M_PI) {
        throw mems_ins_gps::ValidationException("lonRad", 
            "Longitude must be in [-180°, 180°]");
    }

    Eigen::Quaterniond quat;

    // 计算经度旋转的四元数（绕Z轴）
    double cosLon = cos(lonRad * 0.5);
    double sinLon = sin(lonRad * 0.5);
    // 计算纬度旋转的四元数（绕Y轴，注意角度为90°-lat）
    double cosLat = cos(-M_PI * 0.25 - latRad * 0.5);
    double sinLat = sin(-M_PI * 0.25 - latRad * 0.5);

    quat.w() = cosLat * cosLon;
    quat.x() = -sinLat * sinLon;
    quat.y() = sinLat * cosLon;
    quat.z() = cosLat * sinLon;

    return quat.normalized();
}

/**
 * @brief 计算地心系（e系）的旋转四元数
 * 
 * 地心系（ECEF）随地球自转而旋转。
 * 旋转轴：Z轴（指向北极）
 * 旋转角速度：ω_ie（地球自转角速度）
 * 
 * 旋转四元数：qee = exp([0, 0, ±ω_ie*dt])
 * 
 * @param dt 时间间隔 (s)
 * @param forward 方向：1表示k-1到k，-1表示k到k-1
 * @return 地心系旋转四元数
 */
Eigen::Quaterniond EarthModel::getQee(const double dt, const int forward)
{
    // 验证输入参数
    if (dt <= 0 || dt > 86400) {  // 不超过一天
        throw mems_ins_gps::ValidationException("dt", 
            "Time interval must be in (0, 86400] seconds");
    }
    if (forward != 1 && forward != -1) {
        throw mems_ins_gps::ValidationException("forward", 
            "Forward direction must be 1 or -1");
    }

    // 旋转向量：绕Z轴旋转，角度为 ±ω_ie*dt
    Eigen::Vector3d theta_e = Eigen::Vector3d {0,0,forward*getWie()*dt};
    return Sophus::SO3d::exp(theta_e).unit_quaternion();
}

/**
 * @brief 计算导航系（n系）的旋转四元数
 * 
 * 导航系（NED）的旋转由两部分组成：
 * 1. 地球自转角速度在导航系的投影（wie_n）
 * 2. 运移角速度（wen_n）：由载体运动引起，导航系随位置变化而旋转
 * 
 * 总角速度：ω_n = wie_n + wen_n
 * 旋转四元数：qnn = exp(±(wie_n + wen_n)*dt)
 * 
 * @param param 导航系参数（包含wie_n和wen_n）
 * @param dt 时间间隔 (s)
 * @param forward 方向：1表示k-1到k，-1表示k到k-1
 * @return 导航系旋转四元数
 */
Eigen::Quaterniond EarthModel::getQnn(const NavEarthParam &param, const double dt, const int forward)
{
    // 验证输入参数
    if (dt <= 0 || dt > 86400) {  // 不超过一天
        throw mems_ins_gps::ValidationException("dt", 
            "Time interval must be in (0, 86400] seconds");
    }
    if (forward != 1 && forward != -1) {
        throw mems_ins_gps::ValidationException("forward", 
            "Forward direction must be 1 or -1");
    }
    if (!std::isfinite(param.Rm) || !std::isfinite(param.Rn) || 
        param.Rm <= 0 || param.Rn <= 0) {
        throw mems_ins_gps::ValidationException("param", 
            "Invalid navigation parameters (Rm or Rn)");
    }

    // 计算导航系旋转向量：±(wie_n + wen_n)*dt
    auto theta_n = forward*(param.wie_n + param.wen_n)*dt;
    return  Sophus::SO3d::exp(theta_n).unit_quaternion().normalized();
}

/**
 * @brief 从四元数qne反算LLH位置
 * 
 * 从导航系到地心系的四元数qne可以反推出经纬度。
 * 这是getQne的逆过程。
 * 
 * 公式推导：
 * - 纬度：从四元数的y和w分量计算
 * - 经度：从四元数的z和w分量计算
 * - 高度：直接使用输入参数
 * 
 * @param qne 导航系到地心系的四元数
 * @param height 高度（米）
 * @return LLH位置向量 [纬度(rad), 经度(rad), 高度(m)]
 */
Eigen::Vector3d EarthModel::getLLhByQne(const Eigen::Quaterniond& qne, const double height)
{
    // 验证四元数归一化
    if (std::abs(qne.norm() - 1.0) > 1e-6) {
        throw mems_ins_gps::ValidationException("qne", 
            "Quaternion is not normalized");
    }

    // 验证高度
    if (!std::isfinite(height)) {
        throw mems_ins_gps::ValidationException("height", 
            "Height must be finite");
    }

    // 从四元数反算纬度：lat = -2*atan(y/w) - π/2
    // 从四元数反算经度：lon = 2*atan2(z, w)
    return {-2 * atan(qne.y() / qne.w()) - M_PI * 0.5, 2 * atan2(qne.z(), qne.w()), height};
}
