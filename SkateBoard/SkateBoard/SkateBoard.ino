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

	//double accelYDeg = mComplementaryFilter.get(ACC_ANGLE_Y_DEG);
	//double pitchDeg = mComplementaryFilter.get(PITCH_DEG);
	double rollDeg = mComplementaryFilter.get(ROLL_DEG);

	double filteredKAL = mKalmanFilter.filtering(rollDeg);
	
	Serial.println(fabs(filteredKAL));

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
		if ((int)mFilteredAccelX < ACC_GYRO_MAX_VALUE / 3) {
			if (i < ((int)(mFilteredAccelX) % (ACC_GYRO_MAX_VALUE / 3))) {
				mNeoPixel.setPixelColor(i, mNeoPixel.Color(255, 255, 255));
			}
			else {
				mNeoPixel.setPixelColor(i, mNeoPixel.Color(0, 0, 0));
			}
		}
		else if ((int)mFilteredAccelX < ACC_GYRO_MAX_VALUE / 3 * 2) {
			if (i < ((int)(mFilteredAccelX) % (ACC_GYRO_MAX_VALUE / 3 * 2) - NEOPIXEL_NUM)) {
				mNeoPixel.setPixelColor(i, mNeoPixel.Color(0, 0, 0));
			}
			else {
				mNeoPixel.setPixelColor(i, mNeoPixel.Color(255, 255, 255));
			}
		}
		else {
			if (i < ((int)(mFilteredAccelX) % ACC_GYRO_MAX_VALUE - 2 * NEOPIXEL_NUM)) {
				mNeoPixel.setPixelColor(i, mNeoPixel.Color(255, 255, 255));
			}
			else {
				mNeoPixel.setPixelColor(i, mNeoPixel.Color(0, 0, 0));
			}
		}
		
#elif (NEOPIXEL_STYLE_2 == 1)
		double dim = 255.0 / ACC_GYRO_MAX;
		int res = (int)(mFilteredAccelX * dim) % 255;
		mNeoPixel.setPixelColor(i, mNeoPixel.Color(res, res, res));
#elif (NEOPIXEL_STYLE_3 == 1)
#define STYLE_3_AMOUNT	10
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
		int val = (int)((255.0 / (double) ACC_GYRO_MAX_VALUE) * ((int)mFilteredAccelX % ACC_GYRO_MAX_VALUE));
		uint8_t red = (uint8_t)((mColorValue >> 16) & 0xFF);
		uint8_t green = (uint8_t)((mColorValue >> 8) & 0xFF);
		uint8_t blue = (uint8_t)(mColorValue & 0xFF);
		mNeoPixel.setPixelColor(i, mNeoPixel.Color(red, green, blue, val));
#elif (NEOPIXEL_STYLE_4 == 1)
		int val = (int)pow((int)mFilteredAccelX % ACC_GYRO_MAX_VALUE, 2);

		if (mColorDir) {
			mColorValue += val;
			if (mColorValue > (uint32_t)0xFFFFFF) {
				mColorValue = (uint32_t)0xFFFFFF;
				mColorDir = false;
			}
	}
		else {
			mColorValue -= val;
			if (mColorValue < (uint32_t)0x000000) {
				mColorValue = (uint32_t)0;
				mColorDir = true;
			}
		}

		uint8_t red = (uint8_t)((mColorValue >> 16) & 0xFF);
		uint8_t green = (uint8_t)((mColorValue >> 8) & 0xFF);
		uint8_t blue = (uint8_t)(mColorValue & 0xFF);
		mNeoPixel.setPixelColor(i, mNeoPixel.Color(red, green, blue));
#endif //end NEOPIXEL_STYLE_N
		mNeoPixel.show();
	}
}
