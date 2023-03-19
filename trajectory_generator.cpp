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

void TrajectoryGenerator::generateTrajectory(const float &initialPosition, const float &finalPosition, const float &samplingTime)
{
    InitialPosition = initialPosition;
    FinalPosition = finalPosition;
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
    for (int i = 0; i < n; i++)
    {
        Nparams[i] = ceil(TimeVecOut[i] / SamplingTime);
        TotalDuration += Nparams[i] * SamplingTime;
        std::cout << "Nparams[" << i << "] = " << Nparams[i] << std::endl;
    }

    ///////////////////////////////////////

    std::vector<float> numerator = {0, SamplingTime}; // biqaud filter
    std::vector<float> denominator = {1, -1};         // biqaud filter

    // first block

    std::vector<float> coefficients1(Nparams[0] + 1);

    for (size_t i = 0; i < coefficients1.size(); i++)
    {
        if (i == 0)
        {
            coefficients1[i] = 1.0 / (coefficients1.size() - 1) / samplingTime;
            continue;
        }
        else if (i == coefficients1.size() - 1)
        {
            coefficients1[i] = -1.0 / (coefficients1.size() - 1) / samplingTime;
            continue;
        }
        else
        {
            coefficients1[i] = 0;
        }
    }

    FirFilter firFilter1;
    BiquadFilter biquadFilter1;

    std::vector<float> inputs;
    inputs.resize(TotalDuration/samplingTime);
    std::cout << "inputs.size() = " << inputs.size() << std::endl;
    std::cout << "TotalDuration = " << TotalDuration << std::endl;
    for (int i = 0; i < inputs.size(); i++)
    {
        inputs[i] = FinalPosition; // input signal (temporary)
    }

    std::vector<float> outputFirFilter1 = firFilter1.process(inputs, coefficients1);
    std::vector<float> outputIIRFilter1 = biquadFilter1.process(outputFirFilter1, numerator, denominator);

    // second block
    std::vector<float> coefficients2(Nparams[1] + 1);
    std::cout << "coefficients2.size() = " << coefficients2.size() << std::endl;

    for (size_t i = 0; i < coefficients2.size(); i++)
    {
        if (i == 0)
        {
            coefficients2[i] = 1.0 / (coefficients2.size() - 1) / samplingTime;
            continue;
        }
        else if (i == coefficients2.size() - 1)
        {
            coefficients2[i] = -1.0 / (coefficients2.size() - 1) / samplingTime;
            continue;
        }
        else
        {
            coefficients2[i] = 0;
        }
    }

    FirFilter firFilter2;
    BiquadFilter biquadFilter2;

    std::vector<float> outputFirFilter2 = firFilter2.process(outputIIRFilter1, coefficients2);
    std::vector<float> outputIIRFilter2 = biquadFilter2.process(outputFirFilter2, numerator, denominator);

    // third block
    std::vector<float> coefficients3(Nparams[2] + 1);

    for (size_t i = 0; i < coefficients3.size(); i++)
    {
        if (i == 0)
        {
            coefficients3[i] = 1.0 / (coefficients3.size() - 1) / samplingTime;
            continue;
        }
        else if (i == coefficients3.size() - 1)
        {
            coefficients3[i] = -1.0 / (coefficients3.size() - 1) / samplingTime;
            continue;
        }
        else
        {
            coefficients3[i] = 0;
        }
    }

    FirFilter firFilter3;
    BiquadFilter biquadFilter3;

    std::vector<float> outputFirFilter3 = firFilter3.process(outputIIRFilter2, coefficients3);
    std::vector<float> outputIIRFilter3 = biquadFilter3.process(outputFirFilter3, numerator, denominator);

    TrajectoryPoints = outputIIRFilter3;
    std::cout << "Trajectory generated" << std::endl;
}