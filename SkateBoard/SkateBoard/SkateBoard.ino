/*
 * Name:    SkateBoard.ino
 * Created: 2019-05-12 오후 3:50:14
 * Author:  Minkyu Kang
 */

#include "defconfig.h"
#include "IntegralTrapezoidal.h"
#include "KalmanFilter.h"
#include "MovingAverageFilter.h"
#include "MPU6050.h"
#include "ComplementaryFilter.h"
#include <Adafruit_NeoPixel.h>
#include <math.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define SAMPLING_RATE	1 // 1ms

#define NEOPIXEL_PIN	6 // D6
#define NEOPIXEL_NUM	30 // LED SIZE 30

#define STYLE_3_AMOUNT	1

double mFilteredAccelX;
MPU6050 mMPU6050; // SDA:A4, SCL:A5
ComplementaryFilter mComplementaryFilter(SAMPLING_RATE);
MovingAverageFilter mMovingAverageFilter(10);
KalmanFilter mKalmanFilter;
IntegralTrapezoidal mIntegralTrapezoidal(SAMPLING_RATE);
Adafruit_NeoPixel mNeoPixel(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
uint32_t mColorValue;
bool mColorDir;

double SamplingAccel() {
	mMPU6050.implementation();
	double ax, ay, az, gx, gy, gz;

	mMPU6050.getDatas(&ax, &ay, &az, &gx, &gy, &gz, NULL);
	mComplementaryFilter.implementation(ax, ay, az, gx, gy, gz);
	double accelXDeg = mComplementaryFilter.get(ACC_ANGLE_X_DEG);
	double gyroXDeg = mComplementaryFilter.get(GYRO_ANGLE_X_DEG);
	double gyroYDeg = mComplementaryFilter.get(GYRO_ANGLE_Y_DEG);
	double pitchRad = mComplementaryFilter.get(PITCH_RAD);
	double pitchDeg = mComplementaryFilter.get(PITCH_DEG);
	double filteredINT = mIntegralTrapezoidal.implementation(ax);
//	double filteredMAF = mMovingAverageFilter.filtering(filteredINT);
	double filteredKAL = mKalmanFilter.filtering(accelXDeg - pitchDeg);
	
//	Serial.print(accelX);
//	Serial.print('\t');
//	Serial.print(filteredINT);
//	Serial.print('\t');
//	Serial.print(filteredMAF);
//	Serial.print('\t');
//	Serial.println(filteredKAL);
	
	Serial.print(accelXDeg);
	Serial.print('\t');
	Serial.println(pitchDeg);

	return fabs(filteredKAL);
}

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	mMPU6050.init();
	mNeoPixel.begin(); // This initializes the NeoPixel library.

	mColorValue = 0;
	mColorDir = false;
}

// the loop function runs over and over again until power down or reset
void loop() {
	delay(SAMPLING_RATE);
	mFilteredAccelX = SamplingAccel();

	for (int i = 0; i < NEOPIXEL_NUM; i++) {
#if (NEOPIXEL_STYLE_1 == 1)
		if (i < ((int)(mFilteredAccelX * 2) % NEOPIXEL_NUM)) {
			mNeoPixel.setPixelColor(i, mNeoPixel.Color(255, 255, 255));
		}
		else {
			mNeoPixel.setPixelColor(i, mNeoPixel.Color(0, 0, 0));
		}
#elif (NEOPIXEL_STYLE_2 == 1)
		double dim = 255.0 / ACC_GYRO_MAX;
		int res = (int)(mFilteredAccelX * dim) % 255;
		mNeoPixel.setPixelColor(i, mNeoPixel.Color(res, res, res));
#elif (NEOPIXEL_STYLE_3 == 1)
		if (mColorDir) {
			mColorValue += STYLE_3_AMOUNT;
			if (mColorValue > (uint32_t)0xFFFFFF) {
				mColorValue = (uint32_t)0xFFFFFF;
				mColorDir = false;
			}
		}
		else {
			mColorValue -= STYLE_3_AMOUNT;
			if (mColorValue < (uint32_t)0x000000) {
				mColorValue = (uint32_t)0;
				mColorDir = true;
			}
		}
		
		uint8_t red = (uint8_t)((mColorValue >> 16) & 0xFF) / ACC_GYRO_MAX * (int)mFilteredAccelX;
		uint8_t green = (uint8_t)((mColorValue >> 8) & 0xFF) / ACC_GYRO_MAX * (int)mFilteredAccelX;
		uint8_t blue = (uint8_t)(mColorValue & 0xFF) / ACC_GYRO_MAX * (int)mFilteredAccelX;
		mNeoPixel.setPixelColor(i, mNeoPixel.Color(red, green, blue));
#endif //end NEOPIXEL_STYLE_N
		mNeoPixel.show();
	}
}
