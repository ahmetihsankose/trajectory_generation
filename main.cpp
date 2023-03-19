#include <iostream>
#include <vector>
#include "biquad_filter.h"
#include "fir_filter.h"
#include "logger.h"
#include "trajectory_generator.h"

int main()
{
    int coefficientsLength1 = 501;
    int coefficientsLength2 = 201;
    int coefficientsLength3 = 101;

    float samplingTime = 0.01;
    std::vector<float> coefficients1(coefficientsLength1);
    std::vector<float> coefficients2(coefficientsLength2);
    std::vector<float> coefficients3(coefficientsLength3);

    for (size_t i = 0; i < coefficientsLength1; i++)
    {
        if (i == 0)
        {
            coefficients1[i] = 1.0 / (coefficientsLength1 - 1) / samplingTime;
            continue;
        }
        else if (i == coefficientsLength1 - 1)
        {
            coefficients1[i] = -1.0 / (coefficientsLength1 - 1) / samplingTime;
            continue;
        }
        else
        {
            coefficients1[i] = 0;
        }
    }

    for (size_t i = 0; i < coefficientsLength2; i++)
    {
        if (i == 0)
        {
            coefficients2[i] = 1.0 / (coefficientsLength2 - 1) / samplingTime;
            continue;
        }
        else if (i == coefficientsLength2 - 1)
        {
            coefficients2[i] = -1.0 / (coefficientsLength2 - 1) / samplingTime;
            continue;
        }
        else
        {
            coefficients2[i] = 0;
        }
    }

    for (size_t i = 0; i < coefficientsLength3; i++)
    {
        if (i == 0)
        {
            coefficients3[i] = 1.0 / (coefficientsLength3 - 1) / samplingTime;
            continue;
        }
        else if (i == coefficientsLength3 - 1)
        {
            coefficients3[i] = -1.0 / (coefficientsLength3 - 1) / samplingTime;
            continue;
        }
        else
        {
            coefficients3[i] = 0;
        }
    }

    std::vector<float> inputsFIR;
    inputsFIR.resize(801);
    for (size_t i = 0; i < inputsFIR.size(); i++)
    {
        inputsFIR[i] = 100;
    }

    FirFilter firFilter1;
    std::vector<float> outputsFir1 = firFilter1.process(inputsFIR, coefficients1);

    // biquad filter
    std::vector<float> inputsIIR1;

    inputsIIR1.resize(outputsFir1.size());
    for (size_t i = 0; i < outputsFir1.size(); i++)
    {
        inputsIIR1[i] = outputsFir1[i];
    }
    std::vector<float> numerator = {0, 0.01};
    std::vector<float> denominator = {1, -1};

    BiquadFilter biquadFilter1;
    std::vector<float> outputsBiquad1 = biquadFilter1.process(inputsIIR1, numerator, denominator);

    FirFilter firFilter2;
    std::vector<float> outputsFir2 = firFilter2.process(outputsBiquad1, coefficients2);

    BiquadFilter biquadFilter2;
    std::vector<float> outputsBiquad2 = biquadFilter2.process(outputsFir2, numerator, denominator);

    FirFilter firFilter3;
    std::vector<float> outputsFir3 = firFilter3.process(outputsBiquad2, coefficients3);

    BiquadFilter biquadFilter3;
    std::vector<float> outputsBiquad3 = biquadFilter3.process(outputsFir3, numerator, denominator);


    // std::vector<float> timeVecIn = {1.9672, 2.6522, 0.5111};
    // std::vector<float> timeVecOut;
    // timeVecOut.resize(timeVecIn.size());

    std::vector<float> kinematicConstarints = {120, 61, 23, 45};
    TrajectoryGenerator trajectoryGenerator(kinematicConstarints);

    trajectoryGenerator.generateTrajectory(0, 120, 0.001);
    std::vector<float> trajectoryPoints = trajectoryGenerator.getTrajectoryPoints();
    // timeVecOut = trajectoryGenerator.checkConstraints(timeVecIn);
    // for (size_t i = 0; i < timeVecOut.size(); i++)
    // {
    //     std::cout << timeVecOut[i] << std::endl;
    // }

    // trajectoryGenerator.generateTrajectory(0, 100, 0.001);




    Logger logger("/home/kose/workspace/trajectory-generator/output.txt");
    logger.Log(trajectoryPoints);


    return 0;
}
