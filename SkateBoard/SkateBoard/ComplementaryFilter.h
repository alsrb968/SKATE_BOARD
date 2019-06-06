#pragma once

#include <Arduino.h>
#include <math.h>
#include "IntegralTrapezoidal.h"

#define RAD2DEG(rad)	(double)(rad * 180.0 / PI)
#define DEG2RAD(deg)	(double)(deg / 180.0 * PI)
#define GYRO_FACTOR		(double)(0.90)
#define ACCEL_FACTOR	(double)(1 - GYRO_FACTOR)

enum COMPLEMENTARY_ELEMENT
{
	ACC_ANGLE_X_RAD,
	ACC_ANGLE_Y_RAD,
	ACC_ANGLE_X_DEG,
	ACC_ANGLE_Y_DEG,
	GYRO_ANGLE_X_DEG,
	GYRO_ANGLE_Y_DEG,
	PITCH_DEG,
	ROLL_DEG,
	PITCH_RAD,
	ROLL_RAD,
};

class ComplementaryFilter
{
private:
	double dt;
	double accelAngleXRad;
	double accelAngleYRad;
	double accelAngleXDeg;
	double accelAngleYDeg;
	double gyroAngleXDeg;
	double gyroAngleYDeg;
	double pitchDeg;
	double rollDeg;
	double pitchRad;
	double rollRad;
	IntegralTrapezoidal *integralGx;
	IntegralTrapezoidal *integralGy;
public:
	ComplementaryFilter(double dt);
	~ComplementaryFilter();
	void implementation(double ax, double ay, double az,
						double gx, double gy, double gz);
	double get(COMPLEMENTARY_ELEMENT ce);
};

