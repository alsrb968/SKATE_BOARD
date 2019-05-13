#include "ComplementaryFilter.h"

ComplementaryFilter::ComplementaryFilter(double dt)
{
	this->dt = dt;
	accelAngleXDeg = 0;
	accelAngleYDeg = 0;
	gyroAngleXDeg = 0;
	gyroAngleYDeg = 0;
	pitchDeg = 0;
	rollDeg = 0;
	pitchRad = 0;
	rollRad = 0;
	integralGx = new IntegralTrapezoidal(dt);
	integralGy = new IntegralTrapezoidal(dt);
}

ComplementaryFilter::~ComplementaryFilter()
{
	delete integralGx;
	delete integralGy;
}

void ComplementaryFilter::implementation(double ax, double ay, double az, double gx, double gy, double gz)
{
	//Accelerator -> Radian Angle -> Degree Angle
	accelAngleXDeg = RAD2DEG(atanf(-ax / sqrtf(powf(ay, 2) + powf(az, 2))));
	accelAngleYDeg = RAD2DEG(atanf(ay / sqrtf(powf(ax, 2) + powf(az, 2))));

	gyroAngleXDeg = integralGx->implementation(gx);
	gyroAngleYDeg = integralGy->implementation(gy);
	//Angle = 0.98 * (Angle + GyroAngle) + 0.02 * AcclAngle
	pitchDeg = GYRO_FACTOR * (pitchDeg + gyroAngleYDeg) + ACCEL_FACTOR * accelAngleXDeg;
	rollDeg = GYRO_FACTOR * (rollDeg + gyroAngleXDeg) + ACCEL_FACTOR * accelAngleYDeg;

	pitchRad = DEG2RAD(pitchDeg);
	rollRad = DEG2RAD(rollDeg);
}

double ComplementaryFilter::get(COMPLEMENTARY_ELEMENT ce)
{
	double ret = 0;
	switch (ce)
	{
	case ACC_ANGLE_X_DEG:	ret = accelAngleXDeg;	break;
	case ACC_ANGLE_Y_DEG:	ret = accelAngleYDeg;	break;
	case GYRO_ANGLE_X_DEG:	ret = gyroAngleXDeg;	break;
	case GYRO_ANGLE_Y_DEG:	ret = gyroAngleYDeg;	break;
	case PITCH_DEG:			ret = pitchDeg;			break;
	case ROLL_DEG:			ret = rollDeg;			break;
	case PITCH_RAD:			ret = pitchRad;			break;
	case ROLL_RAD:			ret = rollRad;			break;
	}
	return ret;
}

