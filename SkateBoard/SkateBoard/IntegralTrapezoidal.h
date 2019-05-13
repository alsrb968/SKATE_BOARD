#pragma once
class IntegralTrapezoidal
{
private:
	double dt;	// 1ms => 0.001
	double past;
	double out;
public:
	IntegralTrapezoidal(double dt);
	~IntegralTrapezoidal();
	double implementation(double present);
};

