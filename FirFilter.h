/*
    Written by Ahmet Ihsan KOSE, Istanbul, Turkey
    Contact koseahmetihsan@gmail.com
*/

#pragma once
#include "Filter.h"
#include <omp.h>

#define ENABLE_PARALLEL_PROCESSING TRUE

class FirFilter : public Filter
{
public:
    FirFilter() = default;

    std::vector<float> process(const std::vector<float> &inputs, const std::vector<float> &coefficients);
};
