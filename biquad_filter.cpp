#include "biquad_filter.h"


std::vector<float> BiquadFilter::process(const std::vector<float> &inputs,
                                         const std::vector<float> &numCoeff,
                                         const std::vector<float> &denCoeff)
{
    Outputs.resize(inputs.size());
    std::vector<float> w(3, 0.0);
    for (int i = 0; i < inputs.size(); i++)
    {
        w[0] = inputs[i] - denCoeff[1] * w[1] - denCoeff[2] * w[2];
        Outputs[i] = numCoeff[0] * w[0] + numCoeff[1] * w[1] + numCoeff[2] * w[2];
        w[2] = w[1];
        w[1] = w[0];
    }
    return Outputs;
}

