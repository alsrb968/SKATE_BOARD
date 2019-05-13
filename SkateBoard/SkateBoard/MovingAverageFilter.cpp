#include "MovingAverageFilter.h"

MovingAverageFilter::MovingAverageFilter(int size) 
{
	this->size = size;
	cnt = 0;
	fulled = false;
	buf = new float(size);
	memset(buf, 0, sizeof(buf));
	out = 0;
}

MovingAverageFilter::~MovingAverageFilter() 
{
	delete buf;
}

float MovingAverageFilter::filtering(float input) 
{
	float total = 0;

	buf[cnt++] = input;
	cnt %= size;
	if (cnt >= size - 1) fulled = true;
	if (!fulled) {
		for (int i = 0; i < cnt; i++) {
			total += buf[i];
		}
		out = total / (float)cnt;
	}
	else {
		for (int i = 0; i < size; i++) {
			total += buf[i];
		}
		out = total / (float)size;
	}

	return out;
}
