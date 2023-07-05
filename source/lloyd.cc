#include <lloyd.h>

using namespace std;

void LloydQuantizer::initialize(float min, float max, int bits)
{
    this->min = min;
    this->max = max;
    level = pow(2, bits);

    T = new float[level + 1];
    R = new float[level];

    T[0] = min;
    T[level] = max;

    // initially set all representation values with linear quantization
    for (int i = 0; i < level; i++)
    {
        R[i] = min + (max - min) * i / level;
    }

    adjustThresholds();
}

void LloydQuantizer::adjustThresholds()
{
    for (int i = 1; i < level; i++)
    {
        T[i] = (R[i - 1] + R[i]) / 2;
    }
}

void LloydQuantizer::adjustRepresentationValues(vector<pair<float, float>> &prob)
{
    float floor, ceiling;
    float numOfPoints, density;
    for (int i = 0; i < level; i++)
    {
        ceiling = ceilf(T[i]);

        if(i == level-1) floor = ceilf(T[i+1]);
        else floor = floorf(T[i+1]);

        numOfPoints = 0;
        density = 0;

        for(int j = floor; j <= ceiling; j++)
        {
            numOfPoints += j * prob[j].second;
            density += prob[j].second;
        }

        if(density != 0)
            R[i] = numOfPoints / density;
    }
}

float LloydQuantizer::calculateMSE(vector<float> &values)
{
    float sum = 0;
    for (int i = 0; i < values.size(); i++)
    {
        sum += pow(values[i] - getQuantizedValue(values[i]), 2);
    }
    return sum / values.size();
}

float LloydQuantizer::getQuantizedValue(float value)
{
    if (value == max) return T[level-1];

    int i;
    for (i = 0; i < level; i++)
    {
        if (value >= T[i] && value < T[i + 1]) break;
    }
    return R[i];
}

void LloydQuantizer::train(vector<float> &values, vector<pair<float, float>> &prob, int iterations)
{
    cout << "Started Training......" << endl;
    float prevMSE, prevPSNR, curMSE, curPSNR;

    int iterationNo = 1;

    cout << "Iteration ( " << iterationNo++ << " ) : " << endl;
    adjustRepresentationValues(prob);
    curMSE = calculateMSE(values);
    curPSNR = 10 * log10(pow(max, 2) / curMSE);
    cout << "\tMSE: " << curMSE << ", PSNR: " << curPSNR << endl;

    // Recurse until distortion rate less than 0.001
    while(iterationNo <= iterations)
    {
        cout << "Iteration ( " << iterationNo++ << " ) : " << endl;

        adjustThresholds();
        adjustRepresentationValues(prob);
        prevMSE = curMSE;
        prevPSNR = curPSNR;
        curMSE = calculateMSE(values);
        curPSNR = 10 * log10(pow(max, 2) / curMSE);

        cout << "\tMSE: " << curMSE << ", PSNR: " << curPSNR << endl;
        cout << "\tMSE delta: " << curMSE - prevMSE << ", PSNR delta: " << curPSNR - prevPSNR << endl;
    }

    cout << "Done training .. :)" << endl;
}

vector<float> LloydQuantizer::quantize(vector<float> &values)
{
    vector<float> quantizedValues;
    for (int i = 0; i < values.size(); i++)
    {
        quantizedValues.push_back(getQuantizedValue(values[i]));
    }
    float MSE = calculateMSE(values);
    float PSNR = 10 * log10(pow(max, 2) / MSE);
    cout << "\tMSE: " << MSE << ", PSNR: " << PSNR << endl;

    return quantizedValues;
}

void LloydQuantizer::test(vector<float> &values, float &MSE, float &PSNR)
{
    MSE = calculateMSE(values);
    PSNR = 10 * log10(pow(max, 2) / MSE);
}