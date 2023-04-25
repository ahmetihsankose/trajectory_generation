#include "FirFilter.h"

std::vector<float> FirFilter::process(const std::vector<float> &inputs, const std::vector<float> &coefficients)
{
    Outputs.resize(inputs.size());

    #if ENABLE_PARALLEL_PROCESSING
    #pragma omp parallel for
    #endif
    for (int i = 0; i < Outputs.size(); i++)
    {
        float temp = 0;
        int j_start = std::max(0, i - static_cast<int>(inputs.size()) + 1);
        for (int j = j_start; j < coefficients.size(); j++)
        {
            int input_idx = i - j;
            if (input_idx >= 0)
            {
                temp += coefficients[j] * inputs[input_idx];
            }
        }
        Outputs[i] = temp;
    }
    return Outputs;
}