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

void LloydQuantizer::adjustRepresentationValues(vector<pair<float, int>> &count)
{
  int countIdx = 0;
  for (int i = 0; i < level; i++)
  {
    float floor, ceiling;
    float accValue = 0, numPoints = 0;

    floor = T[i];
    ceiling = T[i+1];

    cout << "Level " << i << ": floor " << floor << ", ceiling " << ceiling << endl;

    for (float j = floor; j < ceiling; j+=0.005)
    {
      if(count[countIdx].first <= floor)
      {
        accValue += j * count[countIdx].second;
        numPoints += count[countIdx].second;
        countIdx++;
      }
    }

    if (numPoints != 0)
    {
      R[i] = accValue / numPoints;
      cout << "\t Representation value: " << R[i] << endl;
    }
  }
}

float LloydQuantizer::calculateMSE(vector<float> &values)
{
    float sum = 0;
    for (int i = 0; i < values.size(); i++)
    {
        sum += pow((values[i] - getQuantizedValue(values[i])), 2);
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

void LloydQuantizer::train(vector<float> &values, vector<pair<float, int>> &count, int iterations)
{
    cout << "Started Training......" << endl;
    float prevMSE, prevPSNR, curMSE, curPSNR;

    int iterationNo = 1;

    cout << "Iteration ( " << iterationNo++ << " ) : " << endl;
    adjustRepresentationValues(count);
    curMSE = calculateMSE(values);
    curPSNR = 10 * log10(pow(max, 2) / curMSE);
    cout << "\tMSE: " << curMSE << ", PSNR: " << curPSNR << endl;

    // Recurse until distortion rate less than 0.001
    while(iterationNo <= iterations)
    {
        cout << "Iteration ( " << iterationNo++ << " ) : " << endl;

        adjustThresholds();
        adjustRepresentationValues(count);
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