#pragma once
#include "filter.h"
#include <cstddef>
#include <stdio.h>
#include <stdlib.h>

class FirFilter : public Filter
{

public:
    FirFilter(){};

    std::vector<float> process(const std::vector<float> &inputs, const std::vector<float> &coefficients);
    float *process(const float *inputs, int inputsSize, const float *coefficients, int coefficientsSize);

private:
    std::vector<float> Coefficients;

private:
    struct Input
    {
        float *Inputs;
        int InputsSize;
        float *Outputs;
        int OutputsSize;
        float *Coefficients;
        int CoefficientsSize;
    } input;
};
