#ifndef SENSOR_H
#define SENSOR_H

class Sensor {
public:
    Sensor() {}
    virtual void update() = 0; 
};

#endif // SENSOR_H