#include "MPU6050.h"

MPU6050::MPU6050() 
{
	Threshold = 10.0;
}

MPU6050::~MPU6050() 
{

}

void MPU6050::init() 
{
	Wire.begin();      //Wire 라이브러리 초기화
	Wire.beginTransmission(MPU); //MPU로 데이터 전송 시작
	Wire.write(0x6B);  // PWR_MGMT_1 register
	Wire.write(0);     //MPU-6050 시작 모드로
	Wire.endTransmission(true);
}

void MPU6050::implementation() 
{
	Wire.beginTransmission(MPU);		// start data send
	Wire.write(0x3B);					// register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);		// on link
	Wire.requestFrom(MPU, 14, true);	// require data to MPU

	ax = (double)(Wire.read() << 8 | Wire.read()) * accelScale;  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
	ay = (double)(Wire.read() << 8 | Wire.read()) * accelScale;  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	az = (double)(Wire.read() << 8 | Wire.read()) * accelScale;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	tmp = (double)(Wire.read() << 8 | Wire.read());  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	gx = (double)(Wire.read() << 8 | Wire.read()) * gyroScale;  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	gy = (double)(Wire.read() << 8 | Wire.read()) * gyroScale;  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	gz = (double)(Wire.read() << 8 | Wire.read()) * gyroScale;  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
	_1g = sqrtf(powf(ax, 2) + powf(ay, 2) + powf(az, 2));//*9.8;
}

double MPU6050::get(MPU6050_ELEMENT me)
{
	double ret = 0;
	switch (me)
	{
	case ACCEL_X:		ret = ax;	break;
	case ACCEL_Y:		ret = ay;	break;
	case ACCEL_Z:		ret = az;	break;
	case GYRO_X:		ret = gx;	break;
	case GYRO_Y:		ret = gy;	break;
	case GYRO_Z:		ret = gz;	break;
	case ONE_GRAVIRY:	ret = _1g;	break;
	}
	return ret;
}

void MPU6050::getDatas(double* ax, double* ay, double* az, double* gx, double* gy, double* gz, double* _1g)
{
	*ax = this->ax;
	*ay = this->ay;
	*az = this->az;
	*gx = this->gx;
	*gy = this->gy;
	*gz = this->gz;
	*_1g = this->_1g;
}
