#pragma once
#include <vector>

class TrajectoryGenerator
{
public:
    TrajectoryGenerator() = default;
    ~TrajectoryGenerator() = default;

    void setKinematicLimits(float maxVelocity, float maxAcceleration, float maxJerk)
    {
        MaxVelocity = maxVelocity;
        MaxAcceleration = maxAcceleration;
        MaxJerk = maxJerk;
    }

    std::vector<float> generateTrajectory(const float &initialPosition, const float &finalPosition,const float &samplingTime);

private:
    float MaxDistance;   
    float MaxVelocity;
    float MaxAcceleration;
    float MaxJerk;

private:
    float InitialPosition;
    float FinalPosition;
    float SamplingTime;

    std::vector<float> TrajectoryPoints;
};