#pragma once
#include <vector>

class Filter
{
public:
    std::vector<float> getOutputs() const { return Outputs; }

protected:
    std::vector<float> Outputs;
};
