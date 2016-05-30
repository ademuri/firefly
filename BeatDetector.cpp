/*
 * BeatDetector.cpp
 *
 *  Created on: May 30, 2016
 *      Author: Adam
 */

#include "BeatDetector.h"

// Liberally uses code from http://dpeckett.com/beat-detection-on-the-arduino

// 20 - 200hz Single Pole Bandpass IIR Filter
float bassFilter(float sample) {
	static float v[3] = {0, 0, 0};
	v[0] = v[1];
				v[1] = v[2];
				v[2] = (6.849759199635238049e-1 * sample)
					 + (0.36002215309575646973 * v[0])
					 + (0.28164801021172070072 * v[1]);
				return
					 (v[2] - v[0]);
}

// 10hz Single Pole Lowpass IIR Filter
float envelopeFilter(float sample) {
	static float v[2] = {0, 0};
	v[0] = v[1];
				v[1] = (5.919070381840546569e-2 * sample)
					 + (0.88161859236318906863 * v[0]);
				return
					 (v[0] + v[1]);
}

// 1.7 - 3.0hz Single Pole Bandpass IIR Filter
float beatFilter(float sample) {
	static float v[3] = {0, 0, 0};
	v[0] = v[1];
				v[1] = v[2];
				v[2] = (1.425578634058458360e-1 * sample)
					 + (-0.71698617409790299515 * v[0])
					 + (1.44536535009277544717 * v[1]);
				return
					 (v[2] - v[0]);
}

BeatDetector::BeatDetector() {
	pinMode(8, OUTPUT);
}

void BeatDetector::cast(void* param) {
	((BeatDetector*)param)->taskFunc();
}

void BeatDetector::taskFunc() {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	float sample, value, envelope, beat;
	const float thresh = 0.5f;
	while(1) {
		for (byte i=0;; i++) {
			// Read ADC and center so +-512
			sample = (float)analogRead(INPUT_PIN)-503.f;

			// Filter only bass component
			value = bassFilter(sample);

			// Take signal amplitude and filter
			if(value < 0)value=-value;
			envelope = envelopeFilter(value);

			if(i == 20) {
				// Filter out repeating bass sounds 100 - 180bpm
				beat = beatFilter(envelope);

				if (beat > thresh) {
					digitalWrite(8, HIGH);
				} else {
					digitalWrite(8, LOW);
				}
				i = 0;
			}

			// Maintain fixed sample frequency
			// Note that since this is trying to delay only 2 ticks, it may not maintain a very
			// fixed sample frequency. The filters rely on a fixed frequency, so it may make them
			// unreliable.
			vTaskDelayUntil( &xLastWakeTime, 2 );
		}
	}
}
