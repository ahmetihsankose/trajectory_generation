/*
    Written by Ahmet Ihsan KOSE, Istanbul, Turkey
    Contact koseahmetihsan@gmail.com
*/

#pragma once
#include "Filter.h"
#include <omp.h>

#define ENABLE_PARALLEL_PROCESSING TRUE

class BiquadFilter : public Filter
{
public:
    BiquadFilter() = default;
    ~BiquadFilter() = default;
    
    std::vector<float> process(const std::vector<float> &inputs,
                               const std::vector<float> &numCoeff,
                               const std::vector<float> &denCoeff);
};