#ifndef COMPLEMENTARYFILTER_H
#define COMPLEMENTARYFILTER_H

#include <Arduino.h>

class ComplementaryFilter
{
public:
    ComplementaryFilter(float alpha = 0.9f);

    void update(float ax, float ay, float az, float gx, float gy, float gz, float dt);

    float getRollRad() const;
    float getPitchRad() const;
    float getYawRad() const;

    float getRollDeg() const;
    float getPitchDeg() const;
    float getYawDeg() const;

private:
    float alpha;
    float roll;  // in radianti
    float pitch; // in radianti
    float yaw;   // in radianti
};

#endif // COMPLEMENTARYFILTER_H
