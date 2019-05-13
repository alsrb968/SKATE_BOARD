#pragma once

#include <string.h>

class MovingAverageFilter 
{
private:
	int size;
	int cnt;
	bool fulled;
	float *buf;
	float out;
public:
	MovingAverageFilter(int size);
	~MovingAverageFilter();
	float filtering(float input);
};
