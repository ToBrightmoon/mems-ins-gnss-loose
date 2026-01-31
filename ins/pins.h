#ifndef MEMS_INS_GPS_PINS_H
#define MEMS_INS_GPS_PINS_H

#include "measurement/imu.h"
#include "state/state.h"

#define USE_EASY_PINS

class Pins
{
public:
    Pins() = default;

    ~Pins() = default;

    Pins(const Pins&) = default;

    Pins& operator=(const Pins&) = default;

    Pins(Pins&&) = default;

    Pins& operator=(Pins&&) = default;

    void init(const InsState&,const ImuData&);

    void insUpdate(const ImuData&);

    [[nodiscard]] InsState getState() const;

    void resetState(const InsState&);

private:
    void propagate(double dt);

    void updateVelocity(double dt);

    void updatePosition(double dt);

    void updateAttitude(double dt);

    void updateVelocityByMid(double dt);

    void updatePositionByMid(double dt);

    void updateAttitudeByMid(double dt);

    [[nodiscard]] bool checkTime(const ImuData&) const;

private:
    ImuData preImuData_{};

    ImuData currImuData_{};

    InsState currState_{}; // 纬，经，高，北东地下的速度，姿态

    InsState prevState_{}; // 纬，经，高，北东地下的速度，姿态
};

#endif //MEMS_INS_GPS_PINS_H