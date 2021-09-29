#include "FastKalmanFilter.h"

FastKalmanFilter::FastKalmanFilter()
{
    this->Bparam = 0.001;
    this->Fparam = 1;
    this->Qparam = 0.0001;
    this->Hparam = 1;
    this->Rparam = 0.1;
}

FastKalmanFilter::FastKalmanFilter(double Qparam, double Rparam, double samplingPeriod, double PNStd, double MNstd, double initialValue)
{
    this->Fparam = 1;
    this->Hparam = 1;
    this->Y = 0.0;
    this->Bparam = samplingPeriod;
    this->Qparam = Qparam * PNStd * PNStd;
    this->Rparam = Rparam * MNstd * MNstd;
    double PPresent = MNstd * MNstd;
    // system dynamic parameters
    this->XHatPresent = initialValue;
    this->SPresent = MNstd * MNstd;
}

double FastKalmanFilter::GetEstimation(double measuredData, double FinputValue)
{
    // state
    this->XHatForward = this->Fparam * this->XHatPresent + this->Bparam * FinputValue;

    // state uncertainty
    this->PForward = this->Fparam * this->PPresent * this->Fparam + this->Qparam;

    // update
    this->Y = measuredData - this->Hparam * this->XHatForward;

    // innovation
    this->SForward = this->Hparam * this->PForward * this->Hparam + this->Rparam;

    // coefficient
    this->Kparam = this->PForward * this->Hparam * 1.0 / this->SForward;
    this->XHatForward = this->XHatForward + this->Kparam * this->Y;
    this->PForward = (1.0 - this->Kparam * this->Hparam) * this->PForward;

    // update previous values
    this->PPresent = this->PForward;
    this->XHatPresent = this->XHatForward;
    this->SPresent = this->SForward;

    return this->XHatForward;
}
