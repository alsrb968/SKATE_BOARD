#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() 
{
	next = 0;
	out = 0;
	P_next = 0;
	P = 0;
	K = 0;
	Z = 0;
}

KalmanFilter::~KalmanFilter() 
{
}

double KalmanFilter::filtering(double input)
{
	// Predictive stage
	next = out;						// x_next: Correction x Predict x  // x : estimated value
	P_next = P + ValueQ;

	// Calibration stage
	K = P_next / (P_next + ValueR);	// compute the Kalman gain
	Z = input;						// z: noisy original signal
	out = next + K * (Z - next);	// x: filtered signal  // Measurement Update
	P = (1.0 - K) * P_next;

	return out;
}
