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

#define SAMPLING_RATE	1 // 10ms

#define NEOPIXEL_PIN	6
#define NEOPIXEL_NUM	30

#ifdef LEAD_SWITCH_SENSOR
#define LEAD_SWITCH_PIN 3
#endif //end LEAD_SWITCH_SENSOR


#ifdef LEAD_SWITCH_SENSOR
const int numReadings = 10;
uint32_t readings[numReadings];
int readIndex = 0;
int total = 0;

bool oldLeadState;
uint32_t rpm;
uint32_t startT;
uint32_t termT;
const uint32_t oneM = 60000;
#endif //end LEAD_SWITCH_SENSOR

double mFilteredAccelX;
MPU6050 mMPU6050;
ComplementaryFilter mComplementaryFilter(SAMPLING_RATE);
MovingAverageFilter mMovingAverageFilter(10);
KalmanFilter mKalmanFilter;
IntegralTrapezoidal mIntegralTrapezoidal(SAMPLING_RATE);
Adafruit_NeoPixel mNeoPixel(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

#ifdef LEAD_SWITCH_SENSOR
void MovingAverageFilterInit() {
	int readIndex = 0;
	int total = 0;
	for (int i : readings) {
		i = 0;
	}
}

unsigned long MovingAverageFilterRpm(unsigned long rpm) {
	readIndex %= numReadings;
	readings[readIndex++] = rpm;
	if (total < numReadings) total++;
	uint32_t aver = 0;
	for (uint32_t i : readings) {
		aver += i;
	}
	aver /= total;

	return aver;
}
#endif //end LEAD_SWITCH_SENSOR

double SamplingAccel() {
	mMPU6050.implementation();
	double ax, ay, az, gx, gy, gz;
	ax = mMPU6050.get(ACCEL_X);
	ay = mMPU6050.get(ACCEL_Y);
	az = mMPU6050.get(ACCEL_Z);
	gx = mMPU6050.get(GYRO_X);
	gy = mMPU6050.get(GYRO_Y);
	gz = mMPU6050.get(GYRO_Z);
//	mMPU6050.getDatas(&ax, &ay, &az, &gx, &gy, &gz, NULL);
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
	
//	Serial.print((filteredKAL));
//	Serial.print('\t');
	Serial.println(fabs(filteredKAL));

	return fabs(filteredKAL);
}

// the setup function runs once when you press reset or power the board
void setup() {
#ifdef LEAD_SWITCH_SENSOR
	pinMode(LEAD_SWITCH_PIN, INPUT);
	oldLeadState = 0;
	rpm = 0;
	startT = 0;
	termT = 0;
	MovingAverageFilterInit();
#endif //end LEAD_SWITCH_SENSOR
	Serial.begin(115200);
	mMPU6050.init();
	mNeoPixel.begin(); // This initializes the NeoPixel library.
}

// the loop function runs over and over again until power down or reset
void loop() {
#ifdef ACC_GYRO_SENSOR
	delay(SAMPLING_RATE);
	mFilteredAccelX = SamplingAccel();

	for (int i = 0; i < NEOPIXEL_NUM; i++) {
#if (NEOPIXEL_STYLE_1 == 1)
		if (i < ((int)(mFilteredAccelX * 2) % 30))  mNeoPixel.setPixelColor(i, mNeoPixel.Color(255, 255, 255));
		else            mNeoPixel.setPixelColor(i, mNeoPixel.Color(0, 0, 0));
#elif (NEOPIXEL_STYLE_2 == 1)
		double dim = 255.0 / ACC_GYRO_MAX;
		int res = (int)(mFilteredAccelX * dim) % 255;
		mNeoPixel.setPixelColor(i, mNeoPixel.Color(res, res, res));
#elif (NEOPIXEL_STYLE_3 == 1)
		//TODO:
#endif
		mNeoPixel.show();
	}
#endif //end ACC_GYRO_SENSOR

#ifdef LEAD_SWITCH_SENSOR  
	int newLeadState = digitalRead(LEAD_SWITCH_PIN);
	if (oldLeadState > newLeadState) { //Falling Edge
		termT = millis() - startT;
		rpm = MovingAverageFilterRpm(oneM / termT);
		Serial.println(rpm);
		startT = millis();
	}
	else {
		if (millis() - startT > 2000) {
			rpm = MovingAverageFilterRpm(0);
			Serial.println(rpm);
			delay(100);
		}
	}
	oldLeadState = newLeadState;

	for (int i = 0; i < NEOPIXEL_NUM; i++) {
#if (NEOPIXEL_STYLE_1 == 1)
		if (i < rpm / 20)  mNeoPixel.setPixelColor(i, mNeoPixel.Color(255, 255, 255));
		else            mNeoPixel.setPixelColor(i, mNeoPixel.Color(0, 0, 0));
#elif (NEOPIXEL_STYLE_2 ==1)
		double dim = 255.0 / 600.0;
		int res = (int)(rpm * dim);
		mNeoPixel.setPixelColor(i, mNeoPixel.Color(res, res, res));
#elif (NEOPIXEL_STYLE_3 == 1)
		//TODO:
#endif
		mNeoPixel.show();
	}
#endif //end LEAD_SWITCH_SENSOR
}
