#include "trajectory_generator.h"
#include <math.h>
#include <algorithm>

std::vector<float> TrajectoryGenerator::generateTrajectory(const float &initialPosition, const float &finalPosition, const float &samplingTime)
{
    InitialPosition = initialPosition;
    FinalPosition = finalPosition;
    SamplingTime = samplingTime;
    MaxDistance = FinalPosition - InitialPosition;

    float Tin[3];

    Tin[0] = MaxDistance / MaxVelocity;
    Tin[1] = MaxVelocity / MaxAcceleration;
    Tin[2] = MaxAcceleration / MaxJerk;

    float aMax = 0.999999;
    float aMin = 0.95;

    float Tout[3];
    for (int i = 0; i < 3; i++)
    {
        Tout[i] = Tin[i];
    }

    int n = sizeof(Tout) / sizeof(Tout[0]);

    if (Tout[1] < Tout[3])
    {
        Tout[2] = sqrt(Tout[2] * Tout[3]);
        Tout[1] = Tout[2];
    }

    for (int i = 3 - 2; i >= 1; i--)
    {
        if (Tout[i] < Tout[i + 1] + Tout[i + 2])
        {
            double a = -Tout[i + 2] / 2.0 / Tout[i + 1] + sqrt(std::pow(Tout[i + 2] / 2.0 / Tout[i + 1], 2) + Tout[i] / Tout[i + 1]);
            a = std::min(std::max(a, aMin), aMax);
            Tout[i] = Tout[i] / a;
            Tout[i + 1] = Tout[i + 1] * a;
        }
    }
}