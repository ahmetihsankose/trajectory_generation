#pragma once
#include <vector>
#include "FirFilter.h"
#include "BiquadFilter.h"

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(std::vector<float> &kinematicConstraints);
    ~TrajectoryGenerator() = default;

        
    std::vector<float> getTimeVecOut() const { return TimeVecOut; }
    void generateTrajectory(const float &samplingTime);

    std::vector<float> getTrajectoryPositions() const { return TrajectoryPositions; }
    std::vector<float> getTrajectoryVelocities() const { return TrajectoryVelocities; }
    std::vector<float> getTrajectoryAccelerations() const { return TrajectoryAccelerations; }
    std::vector<float> getTrajectoryJerk() const { return TrajectoryJerk; }

private:
    void setKinematicConstraints(std::vector<float> &kinematicConstraints);
    std::vector<float> checkConstraints(const std::vector<float> &timeVecIn);
    std::vector<float> generateCoefficents(int length, float sampleTime); // this coefficient for the trajectory generation
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

    std::vector<float> TrajectoryPositions;
    std::vector<float> TrajectoryVelocities;
    std::vector<float> TrajectoryAccelerations;
    std::vector<float> TrajectoryJerk;
};