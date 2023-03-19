#pragma once
#include "filter.h"
#include <cstddef>

class FirFilter : public Filter
{

public:
    FirFilter(){};

    std::vector<float> process(const std::vector<float> &inputs, const std::vector<float> &coefficients);

private:
    std::vector<float> Coefficients;

};
