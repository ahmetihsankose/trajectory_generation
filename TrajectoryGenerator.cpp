/*
    Written by Ahmet Ihsan KOSE, Istanbul, Turkey
    Contact koseahmetihsan@gmail.com
*/

#include "TrajectoryGenerator.h"
#include <cmath>
#include <algorithm>
#include <iostream>

TrajectoryGenerator::TrajectoryGenerator(std::vector<float> &kinematicConstraints)
{
    setKinematicConstraints(kinematicConstraints);
}

void TrajectoryGenerator::setKinematicConstraints(std::vector<float> &kinematicConstraints)
{
    KinematicConstraints = kinematicConstraints;
    if (KinematicConstraints.size() != 4)
    {
        std::cout << "Kinematic constraints vector size is not 4" << std::endl;
        return;
    }
    MaxDistance = KinematicConstraints[0];
    MaxVelocity = KinematicConstraints[1];
    MaxAcceleration = KinematicConstraints[2];
    MaxJerk = KinematicConstraints[3];
}

std::vector<float> TrajectoryGenerator::checkConstraints(const std::vector<float> &timeVecIn)
{
    const float aMax = 0.999999;
    const float aMin = 0.95;
    const int n = timeVecIn.size();

    TimeVecOut = timeVecIn;

    if (TimeVecOut[n - 2] < TimeVecOut[n - 1])
    {
        TimeVecOut[n - 1] = std::sqrt(TimeVecOut[n - 2] * TimeVecOut[n - 1]);
        TimeVecOut[n - 2] = TimeVecOut[n - 1];
    }

    for (int i = n - 3; i >= 0; i--)
    {
        if (TimeVecOut[i] < TimeVecOut[i + 1] + TimeVecOut[i + 2])
        {
            float a = -TimeVecOut[i + 2] / 2.0 / TimeVecOut[i + 1] + std::sqrt(std::pow(TimeVecOut[i + 2] / 2.0 / TimeVecOut[i + 1], 2) + TimeVecOut[i] / TimeVecOut[i + 1]);
            a = std::min(std::max(a, aMin), aMax);
            TimeVecOut[i] = TimeVecOut[i] / a;
            TimeVecOut[i + 1] = TimeVecOut[i + 1] * a;
            TimeVecOut = checkConstraints(TimeVecOut);
        }
    }
    return TimeVecOut;
}

std::vector<float> TrajectoryGenerator::generateCoefficents(int length, float sampleTime)
{
    std::vector<float> vec(length);
    vec[0] = 1.0 / (length * sampleTime);
    vec[length - 1] = -1.0 / (length * sampleTime);
    return vec;
}

void TrajectoryGenerator::generateTrajectory(const float &samplingTime)
{
    SamplingTime = samplingTime;

    std::vector<float> timeVecIn;
    timeVecIn.resize(KinematicConstraints.size() - 1);

    if (KinematicConstraints.size() > 1)
    {
        for (int i = 0; i < timeVecIn.size(); i++)
        {
            timeVecIn[i] = std::abs(KinematicConstraints[i] / KinematicConstraints[i + 1]);
        }
        checkConstraints(timeVecIn);
    }

    int n = TimeVecOut.size(); // order of the trajectory

    std::vector<int> Nparams;
    Nparams.resize(n);
    TotalDuration = 0;
    std::vector<float> numerator = {0, SamplingTime}; // biqaud filter
    std::vector<float> denominator = {1, -1};         // biqaud filter

    FirFilter firFilter;
    BiquadFilter biquadFilter;

    for (int i = 0; i < n; i++)
    {
        Nparams[i] = ceil(TimeVecOut[i] / SamplingTime);
        TotalDuration += Nparams[i] * SamplingTime;
    }

    std::vector<float> inputs;
    inputs.resize(TotalDuration / samplingTime);
    std::cout << "TotalDuration: " << TotalDuration << " seconds " << std::endl;

    for (int i = 0; i < inputs.size(); i++)
    {
        inputs[i] = MaxDistance; // input signal (temporary)
    }

    for (int i = 0; i < n; i++)
    {
        if (i == 0)
        {
            firFilter.process(inputs, generateCoefficents(Nparams[i], samplingTime));
            biquadFilter.process(firFilter.getOutputs(), numerator, denominator);
        }
        else
        {
            firFilter.process(biquadFilter.getOutputs(), generateCoefficents(Nparams[i], samplingTime));
            biquadFilter.process(firFilter.getOutputs(), numerator, denominator);
        }
    }

    TrajectoryPositions = biquadFilter.getOutputs();
    TrajectoryVelocities = firFilter.getOutputs();
    std::cout << "Trajectory generated" << std::endl;
}
