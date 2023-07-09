#pragma once
#include <vector>
#include <bits/stdc++.h>
#include <math.h>

using namespace std;

class LloydQuantizer
{
public:
    float min = 0, max = 80;
    int level = 8;
    float *T, *R; // Thresholds, Representation values

    void initialize(float min, float max, int bits);
    void adjustThresholds();
    void adjustRepresentationValues(vector<pair<float, int>> &count);
    void train(vector<float> &values, vector<pair<float, int>> &count, int iterations);
    vector<float> quantize(vector<float> &values);
    void test(vector<float> &values, float &MSE, float &PSNR);

    float calculateMSE(vector<float> &values);
    float getQuantizedValue(float value);
};