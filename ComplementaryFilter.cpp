#include "ComplementaryFilter.h"

ComplementaryFilter::ComplementaryFilter(float alpha)
    : alpha(alpha), roll(0.0f), pitch(0.0f), yaw(0.0f)
{
}


void ComplementaryFilter::update(float ax, float ay, float az, float gx, float gy, float gz, float dt)
{
    ay = -ay;
    az = -az;
    gx = gx;
    gy = -gy;
    gz = -gz;

    float accRoll  = atan2f(ay, az);
    float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    roll  = alpha * (roll + gx * dt) + (1.0f - alpha) * accRoll;
    pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * accPitch;

    yaw += gz * dt;
}

