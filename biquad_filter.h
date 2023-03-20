#pragma once
#include "filter.h"
#include <stdlib.h>


class BiquadFilter : public Filter
{
public:
    BiquadFilter() = default;
    ~BiquadFilter() = default;
    
    std::vector<float> process(const std::vector<float> &inputs,
                               const std::vector<float> &numCoeff,
                               const std::vector<float> &denCoeff);

    float *process(const float *inputs, int inputsSize,
                   const float *numCoeff, const float *denCoeff, 
                   const int coeffSize);

private:
    struct Input
    {
        float *Outputs;
        int OutputsSize;
    } input;
};