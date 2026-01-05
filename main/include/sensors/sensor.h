#ifndef SENSOR_H
#define SENSOR_H

class Sensor {
public:
    Sensor() {}
    virtual void update_isr() = 0; 
};

#endif // SENSOR_H