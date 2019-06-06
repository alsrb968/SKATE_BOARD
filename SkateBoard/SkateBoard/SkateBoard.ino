/*
 * Name:    SkateBoard.ino
 * Created: 2019-05-12 오후 3:50:14
 * Author:  Minkyu Kang
 */

#include "defconfig.h"
#include "KalmanFilter.h"
#include "MPU6050.h"
#include "ComplementaryFilter.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define SAMPLING_RATE	10 // 1ms

#define NEOPIXEL_PIN	6 // D6
#define NEOPIXEL_NUM	30 // LED SIZE 30

MPU6050 mMPU6050; // SDA:A4, SCL:A5
ComplementaryFilter mComplementaryFilter(SAMPLING_RATE);
KalmanFilter mKalmanFilter;
Adafruit_NeoPixel mNeoPixel(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
uint32_t mColorValue;
bool mColorDir;

double Sampling() {
	mMPU6050.implementation();
	double ax, ay, az, gx, gy, gz;
	mMPU6050.getDatas(&ax, &ay, &az, &gx, &gy, &gz, NULL);
	mComplementaryFilter.implementation(ax, ay, az, gx, gy, gz);
	double rollDeg = mComplementaryFilter.get(ROLL_DEG);
	double filteredKAL = mKalmanFilter.filtering(rollDeg);
	
	double ret = rollDeg;
	Serial.println(ret);
	return ret;
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
	double mFilteredAccelX = Sampling();

	for (int i = 0; i < NEOPIXEL_NUM; i++) {
#if (NEOPIXEL_STYLE_1 == 1)
		mFilteredAccelX = fabs(mFilteredAccelX);
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
		mFilteredAccelX = fabs(mFilteredAccelX);
		double dim = 255.0 / ACC_GYRO_MAX_VALUE;
		int res = (int)(mFilteredAccelX * dim) % 255;
		mNeoPixel.setPixelColor(i, mNeoPixel.Color(res, res, res));
#elif (NEOPIXEL_STYLE_3 == 1)
#define STYLE_3_AMOUNT	10
		mFilteredAccelX = fabs(mFilteredAccelX);
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
		double threshold = 10.0;
		mFilteredAccelX = fabs(mFilteredAccelX);
		int val = (int)pow((int)mFilteredAccelX % ACC_GYRO_MAX_VALUE, 2);
		uint8_t red, green, blue;
		if (mFilteredAccelX < threshold) {
			red = 255;
			green = 255;
			blue = 255;
		}
		else {
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
			red = (uint8_t)((mColorValue >> 16) & 0xFF);
			green = (uint8_t)((mColorValue >> 8) & 0xFF);
			blue = (uint8_t)(mColorValue & 0xFF);
		}
		
		mNeoPixel.setPixelColor(i, mNeoPixel.Color(red, green, blue));
#elif (NEOPIXEL_STYLE_5 == 1)
		double threshold = 10.0;
		double _max = 255.0;
		uint8_t red, green, blue;
		if (-threshold < mFilteredAccelX && mFilteredAccelX < threshold) {
			red = (uint8_t)_max;
			green = (uint8_t)_max;
			blue = (uint8_t)_max;
		}
		else if (mFilteredAccelX >= threshold) {
			red = (uint8_t)_max;
			green = (uint8_t)(_max - (_max / (double)ACC_GYRO_MAX_VALUE * mFilteredAccelX)); 
			blue = (uint8_t)(_max - (_max / (double)ACC_GYRO_MAX_VALUE * mFilteredAccelX));
		}
		else if (mFilteredAccelX <= -threshold) {
			blue = (uint8_t)_max;
			green = (uint8_t)(_max + (_max / (double)ACC_GYRO_MAX_VALUE * mFilteredAccelX));
			red = (uint8_t)(_max + (_max / (double)ACC_GYRO_MAX_VALUE * mFilteredAccelX));
		}
		mNeoPixel.setPixelColor(i, mNeoPixel.Color(red, green, blue));
#elif (NEOPIXEL_STYLE_6 == 1)
		double _max = 255.0;
		uint8_t val;
		if (mFilteredAccelX > 0) {
			if (i < NEOPIXEL_NUM / 2) {
				val = (uint8_t)(_max / (double)ACC_GYRO_MAX_VALUE * mFilteredAccelX);
				mNeoPixel.setPixelColor(i, mNeoPixel.Color(val, val, val));
			}
			
		}
		else {
			if (i >= NEOPIXEL_NUM / 2) {
				val = (uint8_t)(_max / (double)ACC_GYRO_MAX_VALUE * -mFilteredAccelX);
				mNeoPixel.setPixelColor(i, mNeoPixel.Color(val, val, val));
			}
		}
		
#endif //end NEOPIXEL_STYLE_N
		mNeoPixel.show();
	}
}
