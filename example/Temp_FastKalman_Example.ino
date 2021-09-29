
#include "FastKalmanFilter.h"

long tSeconds = 0;
int analogValue1 = 0;

double filteredValue1 = 0;
double temp = 0;

// Fast Kalman Filter Parameters
double Qparameter = 0.1;      // Covariance Process Noise Coefficient
double Rparameter = 10;       // Covariance Measurement Noise Coefficient
double samplingPeriod = 0.01; // Fixed Sampling time of System
double PNStd = 0.04;          // Initial Process Noise Deviation
double MNstd = 0.04;          // Initial Measurement Noise Deviation
double initialValue = 25;     // Known or estimated Initial Value

FastKalmanFilter SensorFilter1(Qparameter, Rparameter, samplingPeriod, PNStd, MNstd, initialValue);

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  tSeconds = millis();
  analogValue1 = analogRead(A0);
  temp = (double)analogValue1 / 1024;
  temp = temp * 5;
  temp = temp - 0.5;
  temp = temp * 100;

  filteredValue1 = SensorFilter1.GetEstimation(temp, 0.0);
  Serial.println(String(tSeconds) + " " + String(temp) + " " + String(filteredValue1));

  delay(10);
}
