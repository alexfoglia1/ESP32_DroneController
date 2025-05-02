#ifndef COMPLEMENTARYFILTER_H
#define COMPLEMENTARYFILTER_H

#include <Arduino.h>

class ComplementaryFilter
{
public:
    ComplementaryFilter(float alpha = 0.9f);

    void update(float ax, float ay, float az, float gx, float gy, float gz, float dt);

    inline float getRollRad() const
    {
      return roll;
    }


    inline float getPitchRad() const
    {
      return pitch;
    }


    inline float getYawRad() const
    {
      return yaw;
    }


    inline float getRollDeg() const
    {
      return degrees(roll);
    }


    inline float getPitchDeg() const
    {
      return degrees(pitch);
    }


    inline float getYawDeg() const
    {
      return degrees(yaw);
    }

private:
    float alpha;
    float roll;   // in radianti
    float pitch;  // in radianti
    float yaw;    // in radianti
};

#endif // COMPLEMENTARYFILTER_H
