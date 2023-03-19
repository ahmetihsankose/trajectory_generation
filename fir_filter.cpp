#include "fir_filter.h"

std::vector<float> FirFilter::process(const std::vector<float> &inputs, const std::vector<float> &coefficients)
{
    // Outputs.resize(inputs.size() + coefficients.size() - 1); // full version
    Outputs.resize(inputs.size()); // truncated version
 
    for (int i = 0; i < Outputs.size(); i++)
    {
        Outputs[i] = 0;
        for (int j = 0; j < coefficients.size(); j++)
        {
            if (i - j >= 0 && i - j < inputs.size())
            {
                Outputs[i] += coefficients[j] * inputs[i - j];
            }
        }
    }
    return Outputs;
}