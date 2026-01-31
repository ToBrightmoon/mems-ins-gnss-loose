#ifndef MEMS_INS_GPS_EARTH_MODEL_H
#define MEMS_INS_GPS_EARTH_MODEL_H

#include <GeographicLib/Constants.hpp>
#include <Eigen/Dense>

class EarthModel {
public:

    struct Ellipsoid {
        const double a  = GeographicLib::Constants::WGS84_a();      // 赤道半径
        const double f  = GeographicLib::Constants::WGS84_f();      // 扁率
        const double e2 = f * (2.0 - f);                            // 第一偏心率平方
        const double wie = 7.2921151467e-5;                         // 地球自转角速度 (rad/s)

        const double gravityEquator = 9.7803253359;      // 赤道重力加速度 g0
        const double gravityPolar = 9.8321849378;        // 极点重力加速度 gp
        const double gravityConstantK = 0.00193185265241; // Somigliana 常数 k
    };

    struct NavEarthParam {
        double Rm;          // 子午圈半径
        double Rn;          // 卯酉圈半径
        double Rmh;         // 考虑高度后的子午圈半径 (Rm + h)
        double Rnh;         // 考虑高度后的卯酉圈半径 (Rn + h)
        Eigen::Vector3d wie_n; // 地球自转角速度在导航系投影 (North, East, Down)
        Eigen::Vector3d wen_n; // 运移角速度在导航系投影 (North, East, Down)
    };

    /**
     * @brief 计算导航系参数
     * @param latRad 纬度 (弧度)
     * @param h       高度 (米)
     * @param vn     北向速度 (m/s)
     * @param ve     东向速度 (m/s)
     */
    static NavEarthParam computeNavParam(double latRad, double h, double vn, double ve);

    static Eigen::Vector2d computeNavRmRn(double latRad);

    static double getGravity(double latRad, double height);

    static Eigen::Matrix3d NDE2LLhMatrix(const double,const double);

    static Eigen::Matrix3d LLh2NEDMatrix(const double,const double);

    static double getWie() ;

    static Eigen::Quaterniond getQne(const double,const double);

    /**
     * @brief 计算e系从 k-1 到k时刻的变化，1是 k-1到k，-1是k到k-1 (坐标系本身的变化是从上向下看)
     * @param dt 时间间隔
     * @param forward 方向
     */
    static Eigen::Quaterniond getQee(const double dt,const int forward);

    /**
     * @brief 计算n系从 k-1 到k时刻的变化，1是 k-1到k，-1是k到k-1 (坐标系本身的变化是从上向下看)
     * @param param 导航系参数
     * @param dt 时间间隔
     * @param forward 方向
     */
    static Eigen::Quaterniond getQnn(const NavEarthParam& param,const double dt,const int forward);

    static Eigen::Vector3d getLLhByQne(const Eigen::Quaterniond&, const double);

private:

    static const Ellipsoid wgs84;
};

#endif //MEMS_INS_GPS_EARTH_MODEL_H