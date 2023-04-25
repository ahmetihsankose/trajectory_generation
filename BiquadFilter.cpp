#include "BiquadFilter.h"


std::vector<float> BiquadFilter::process(const std::vector<float> &inputs,
                                         const std::vector<float> &numCoeff,
                                         const std::vector<float> &denCoeff)
{
    Outputs.resize(inputs.size());

    #if ENABLE_PARALLEL_PROCESSING
    #pragma omp parallel
    #endif
    {
        float w1 = 0.0, w2 = 0.0;

        #if ENABLE_PARALLEL_PROCESSING
        #pragma omp for
        #endif
        for (int i = 0; i < inputs.size(); i++)
        {
            float w0 = inputs[i] - denCoeff[1] * w1 - denCoeff[2] * w2;
            Outputs[i] = numCoeff[0] * w0 + numCoeff[1] * w1 + numCoeff[2] * w2;
            w2 = w1;
            w1 = w0;
        }
    }

    return Outputs;
}