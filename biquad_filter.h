#pragma once
#include "filter.h"

class BiquadFilter : public Filter
{
public:
    BiquadFilter() = default;
    ~BiquadFilter() = default;
    
    std::vector<float> process(const std::vector<float> &inputs,
                               const std::vector<float> &numCoeff,
                               const std::vector<float> &denCoeff);
};