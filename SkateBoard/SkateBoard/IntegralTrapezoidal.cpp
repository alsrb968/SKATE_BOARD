#include "IntegralTrapezoidal.h"

IntegralTrapezoidal::IntegralTrapezoidal(double dt)
{
	this->dt = dt / 1000.0;
	past = 0;
	out = 0;
}

IntegralTrapezoidal::~IntegralTrapezoidal()
{
}

double IntegralTrapezoidal::implementation(double present)
{
	out = (present + past) * dt / 2.0;
	past = present;

	return out;
}
