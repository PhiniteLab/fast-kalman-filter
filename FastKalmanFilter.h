/*
 * Fast Kalman Filter is a non-model based Kalman Filter with one-dimensional input. 
 * It's more likely an implementation of a Low Pass Filter with minimal latency.
 * Created by Phinite Lab - www.phinitelab.com
 * 
 */

#ifndef FastKalmanFilter_h
#define FastKalmanFilter_h

class FastKalmanFilter
{
private:

    // system parameters
    double Bparam;
    double Fparam;
    double Qparam;
    double Hparam;
    double Rparam;
    double Kparam;

    // system dynamic parameters
    double XHatPresent;
    double XHatForward;
    double SPresent;
    double SForward;
    double PPresent;
    double PForward;
    double Y;

public:
    FastKalmanFilter();
    FastKalmanFilter(double Qparam, double Rparam, double samplingPeriod, double PNStd, double MNstd, double initialValue);
    double GetEstimation(double measuredData, double FinputValue);
};

#endif