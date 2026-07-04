#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <stdint.h>
#include "BNO08x.hpp"
#include "constants.h"

class IMUSensor {
public:
    IMUSensor();
    void init();
    void update();

    float ax() const { return accel_x; }
    float ay() const { return accel_y; }
    float az() const { return accel_z; }
    bool is_ready() const { return ready; }

    float qw() const { return q_w; }
    float qi() const { return q_i; }
    float qj() const { return q_j; }
    float qk() const { return q_k; }

    float gx() const { return gyro_x; }
    float gy() const { return gyro_y; }
    float gz() const { return gyro_z; }

private:
    BNO08x imu;
    float accel_x, accel_y, accel_z;
    float q_w, q_i, q_j, q_k, gyro_x, gyro_y, gyro_z; 
    bool ready;
};

#endif