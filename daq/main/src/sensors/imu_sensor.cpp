#include "sensors/imu_sensor.h"
#include <stdio.h>

IMUSensor::IMUSensor()
    : imu([]() {
        bno08x_config_t cfg;
        cfg.io_mosi = (gpio_num_t)IMU_MOSI_PIN;
        cfg.io_miso = (gpio_num_t)IMU_MISO_PIN;
        cfg.io_sclk = (gpio_num_t)IMU_SCK_PIN;
        cfg.io_cs   = (gpio_num_t)IMU_CS_PIN;
        cfg.io_int  = (gpio_num_t)IMU_INT_PIN;
        cfg.io_rst  = (gpio_num_t)IMU_RST_PIN;
        return cfg;
    }()),
      accel_x(0), accel_y(0), accel_z(0),
      q_w(1.0f), q_i(0.0f), q_j(0.0f), q_k(0.0f),
      ready(false)
{}

void IMUSensor::init() {
    if (!imu.initialize()) {
        printf("IMU init failed\n");
        return;
    }
    imu.rpt.accelerometer.enable(10000UL);  // 100Hz
    imu.rpt.rv.enable(10000UL); 
    ready = true;
    printf("IMU init done\n");
}

void IMUSensor::update() {
    if (!ready) return;
    
    if (imu.data_available()) {
            if (imu.rpt.accelerometer.has_new_data()) {
                bno08x_accel_t a = imu.rpt.accelerometer.get();
                accel_x = a.x;
                accel_y = a.y;
                accel_z = a.z;
                //printf("ax=%.2f ay=%.2f az=%.2f\n", a.x, a.y, a.z);
            }
            if (imu.rpt.rv.has_new_data()) {
                bno08x_quat_t q = imu.rpt.rv.get_quat();
                q_w = q.real;
                q_i = q.i;
                q_j = q.j;
                q_k = q.k;
            }
        }
    // static int print_count = 0;
    // if (++print_count >= 50) {
    //     printf("ax=%.2f ay=%.2f az=%.2f | qw=%.3f qi=%.3f qj=%.3f qk=%.3f\n",
    //            accel_x, accel_y, accel_z, q_w, q_i, q_j, q_k);
    //     print_count = 0;
    // }
}