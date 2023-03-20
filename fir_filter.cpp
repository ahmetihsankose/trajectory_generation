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

float *FirFilter::process(const float *inputs, int inputsSize, const float *coefficients, int coefficientsSize)
{
    input.OutputsSize = inputsSize + coefficientsSize - 1;
    input.Outputs = (float *)malloc(input.OutputsSize * sizeof(float));

    if (input.Outputs == NULL)
    {
        fprintf(stderr, "Error: failed to allocate memory for outputs\n");
        return NULL;
    }

    for (int i = 0; i < input.OutputsSize; i++)
    {
        input.Outputs[i] = 0;
        for (int j = 0; j < coefficientsSize; j++)
        {
            if (i - j >= 0 && i - j < inputsSize)
            {
                input.Outputs[i] += coefficients[j] * inputs[i - j];
            }
        }
    }

    return input.Outputs;
}