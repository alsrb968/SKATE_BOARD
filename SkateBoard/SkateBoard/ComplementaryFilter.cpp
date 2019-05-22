#include "ComplementaryFilter.h"

ComplementaryFilter::ComplementaryFilter(double dt)
{
	this->dt = dt;
	accelAngleXRad = 0;
	accelAngleYRad = 0;
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
#if 1
	//Accelerator -> Radian Angle -> Degree Angle
	accelAngleXRad = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2)));
	accelAngleYRad = atan(ay / sqrt(pow(ax, 2) + pow(az, 2)));
	accelAngleXDeg = RAD2DEG(accelAngleXRad);
	accelAngleYDeg = RAD2DEG(accelAngleYRad);

	gyroAngleXDeg = integralGx->implementation(gx);
	gyroAngleYDeg = integralGy->implementation(gy);
#else
	accelAngleXDeg = RAD2DEG(atan2(ax, az));
	accelAngleYDeg = RAD2DEG(atan2(ay, az));

	gyroAngleXDeg = gy / 131. * dt;
	gyroAngleYDeg = gx / 131. * dt;
#endif
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
	case ACC_ANGLE_X_RAD:	ret = accelAngleXRad;	break;
	case ACC_ANGLE_Y_RAD:	ret = accelAngleYRad;	break;
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

