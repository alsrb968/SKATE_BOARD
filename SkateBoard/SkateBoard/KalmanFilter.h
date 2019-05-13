#pragma once

constexpr auto ValueQ = 1.0/100000.0;
constexpr auto ValueR = 0.1*0.1;

class KalmanFilter 
{
private:
	double next;
	double out;
	double P_next;
	double P;
	double K;
	double Z;
public:
	KalmanFilter();
	~KalmanFilter();
	double filtering(double input);
};