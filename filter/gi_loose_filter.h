#ifndef MEMS_INS_GPS_GI_LOOSE_FILTER_H
#define MEMS_INS_GPS_GI_LOOSE_FILTER_H

#include "state/state.h"
#include "measurement/imu.h"

class GILooseFilter
{
public:
    GILooseFilter(const Eigen::MatrixXd& ,const Eigen::MatrixXd& , const Eigen::MatrixXd&);

    ~GILooseFilter() =default;

    GILooseFilter(const GILooseFilter&) = default;

    GILooseFilter& operator=(const GILooseFilter&) = default;

    GILooseFilter(GILooseFilter&&) = default;

    GILooseFilter& operator=(GILooseFilter&&) = default;

    void predict(const InsState&, const ImuData&, const double dt);

    void update(const Eigen::Vector3d&); // 注入的是北东地坐标系下的位置误差，单位是m

    [[nodiscard]] GIState getState() const;

    void reset();

private:
    static Eigen::MatrixXd buildPhi(const InsState & insState,const ImuData&,const double) ;

    static  Eigen::MatrixXd buildG(const InsState& insState) ;

private:
    GIState state_;

    Eigen::MatrixXd Q_ = Eigen::MatrixXd::Identity(12,12);
    Eigen::MatrixXd R_ = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd P_ = Eigen::MatrixXd::Identity(15,15);
};

#endif //MEMS_INS_GPS_GI_LOOSE_FILTER_H