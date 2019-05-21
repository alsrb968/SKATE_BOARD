#pragma once

#include <Wire.h>
#include <math.h>

constexpr auto accelScale = 100.0/16384.0;
constexpr auto gyroScale = 100.0/16384.0;

enum MPU6050_ELEMENT
{
	ACCEL_X,
	ACCEL_Y,
	ACCEL_Z,
	GYRO_X,
	GYRO_Y,
	GYRO_Z,
	ONE_GRAVIRY,
};

class MPU6050 
{
private:
	const int MPU = 0x68;
public:
	double ax, ay, az, tmp, gx, gy, gz;
	double _1g;
	double Threshold;
	int cnt;
	int toggle;
public:
	MPU6050();
	~MPU6050();
	void init();
	void implementation();
	double get(MPU6050_ELEMENT me);
	void getDatas(double* _ax, double* _ay, double* _az,
				  double* _gx, double* _gy, double* _gz,
				  double* __1g);
};
