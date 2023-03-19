#pragma once
#include <vector>
#include "fir_filter.h"
#include "biquad_filter.h"

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(std::vector<float> &kinematicConstraints);
    ~TrajectoryGenerator() = default;

    void setKinematicConstraints(std::vector<float> &kinematicConstraints);
        
    std::vector<float> getTimeVecOut() const { return TimeVecOut; }
    void generateTrajectory(const float &initialPosition, const float &finalPosition, const float &samplingTime);

    std::vector<float> getTrajectoryPoints() const { return TrajectoryPoints; }

private:
    std::vector<float> checkConstraints(const std::vector<float> &timeVecIn);

private:
    float MaxDistance;   
    float MaxVelocity;
    float MaxAcceleration;
    float MaxJerk;
    std::vector<float> KinematicConstraints;

private:
    float InitialPosition;
    float FinalPosition;
    float SamplingTime;

    float TotalDuration;

private:
    std::vector<float> TimeVecOut;

    std::vector<float> TrajectoryPoints;
};