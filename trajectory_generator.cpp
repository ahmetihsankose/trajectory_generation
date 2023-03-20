#include "trajectory_generator.h"
#include <math.h>
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
    else if (KinematicConstraints.size() == 4)
    {
        MaxDistance = KinematicConstraints[0];
        MaxVelocity = KinematicConstraints[1];
        MaxAcceleration = KinematicConstraints[2];
        MaxJerk = KinematicConstraints[3];
    }
}

std::vector<float> TrajectoryGenerator::checkConstraints(const std::vector<float> &timeVecIn)
{
    const float aMax = 0.999999;
    const float aMin = 0.95;
    const int n = timeVecIn.size();

    TimeVecOut = timeVecIn;

    if (TimeVecOut[n - 2] < TimeVecOut[n - 1])
    {
        TimeVecOut[n - 1] = sqrt(TimeVecOut[n - 2] * TimeVecOut[n - 1]);
        TimeVecOut[n - 2] = TimeVecOut[n - 1];
    }

    for (int i = n - 3; i >= 0; i--)
    {
        if (TimeVecOut[i] < TimeVecOut[i + 1] + TimeVecOut[i + 2])
        {
            float a = -TimeVecOut[i + 2] / 2.0 / TimeVecOut[i + 1] + sqrt(pow(TimeVecOut[i + 2] / 2.0 / TimeVecOut[i + 1], 2) + TimeVecOut[i] / TimeVecOut[i + 1]);
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
    // InitialPosition = initialPosition;
    // FinalPosition = finalPosition;
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

    ///////////////////////////////////////

    // first block

    // FirFilter firFilter1;
    // BiquadFilter biquadFilter1;

    // std::vector<float> outputFirFilter1 = firFilter1.process(inputs, generateCoefficents(Nparams[0], samplingTime));
    // std::vector<float> outputIIRFilter1 = biquadFilter1.process(outputFirFilter1, numerator, denominator);

    // // second block

    // FirFilter firFilter2;
    // BiquadFilter biquadFilter2;

    // std::vector<float> outputFirFilter2 = firFilter2.process(outputIIRFilter1, generateCoefficents(Nparams[1], samplingTime));
    // std::vector<float> outputIIRFilter2 = biquadFilter2.process(outputFirFilter2, numerator, denominator);

    // // third block

    // FirFilter firFilter3;
    // BiquadFilter biquadFilter3;

    // std::vector<float> outputFirFilter3 = firFilter3.process(outputIIRFilter2, generateCoefficents(Nparams[2], samplingTime));
    // std::vector<float> outputIIRFilter3 = biquadFilter3.process(outputFirFilter3, numerator, denominator);

    TrajectoryPoints = biquadFilter.getOutputs();
    TrajectoryVelocities = firFilter.getOutputs();
    std::cout << "Trajectory generated" << std::endl;
}