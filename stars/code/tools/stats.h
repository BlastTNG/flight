/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef TOOLS__STATS_H
#define TOOLS__STATS_H

#include <cmath>
#include <vector>

template <typename T>
double get_mean(std::vector<T>& values)
{
    if (values.size() < 1) {
        return 0.0;
    }
    double total = 0;
    for (unsigned int i=0; i<values.size(); i++) {
        total += values[i];
    }
    return total/double(values.size());
}

template <typename T>
double get_mean(T values[], int num_values)
{
    if (num_values < 1) {
        return 0.0;
    }
    double total = 0;
    for (int i=0; i<num_values; i++) {
        total += values[i];
    }
    return total/double(num_values);
}

template <typename T>
double get_sample_stdev(std::vector<T>& values, double& mean)
{
    if (values.size() < 2) {
        return 1.0;
    }
    double s = 0;
    for (unsigned int i=0; i<values.size(); i++) {
        s += pow(double(values[i])-mean, 2);
    }
    s = s / (double(values.size()) - 1.0);
    return sqrt(s);
}

template <typename T>
double get_sample_stdev(T values[], int num_values, double& mean)
{
    if (num_values < 2) {
        return 1.0;
    }
    double s = 0;
    for (int i=0; i<num_values; i++) {
        s += pow(double(values[i])-mean, 2);
    }
    s = s / (double(num_values) - 1.0);
    return sqrt(s);
}

template <typename T>
double get_sample_stdev(std::vector<T>& values)
{
    double mean = get_mean(values);
    return get_sample_stdev(values, mean);
}

template <typename T>
double get_sample_stdev(T values[], int num_values)
{
    double mean = get_mean(values, num_values);
    return get_sample_stdev(values, num_values, mean);
}


#endif
