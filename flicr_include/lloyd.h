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
    void adjustRepresentationValues(vector<pair<float, float>> &prob);
    void train(vector<float> &values, vector<pair<float, float>> &prob, int iterations);
    vector<float> test(vector<float> &values);

    float calculateMSE(vector<float> &values);
    float getQuantizedValue(float value);
};