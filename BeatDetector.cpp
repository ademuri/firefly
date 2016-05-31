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
				v[2] = (8.179212904903934711e-3 * sample)
					 + (-0.98379571669365140085 * v[0])
					 + (1.98299691795931032345 * v[1]);
				return
					 (v[2] - v[0]);
}

BeatDetector::BeatDetector() {
	pinMode(8, OUTPUT);
}

void BeatDetector::cast(void* param) {
	((BeatDetector*)param)->taskFunc();
}

extern boolean beatDetected;

void BeatDetector::taskFunc() {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	float sample, value, envelope, beat;
	const float thresh = 1.0f;

	const long DEBOUNCE_MS = 200; // at 180bpm there'll be 330ms between beats
	unsigned long prevHighAt = 0;
	bool prevState = LOW;

	while(1) {
		for (byte i=0;; i++) {
			// Read ADC and center so +-512
			sample = (float)analogRead(INPUT_PIN)-503.f;

			// Filter only bass component
			value = bassFilter(sample);

			// Take signal amplitude and filter
			if (value < 0) {
				value = -value;
			}
			envelope = envelopeFilter(value);

			// Find repeating bass sounds 100 - 180bpm
			beat = beatFilter(envelope);
			if (beat > thresh) {
				digitalWrite(8, HIGH);

				if (prevState == LOW && (millis() > (prevHighAt + DEBOUNCE_MS))) {
					beatDetected = true;
					prevState = HIGH;
					prevHighAt = millis();
				}
			} else {
				digitalWrite(8, LOW);

				if (millis() > prevHighAt + DEBOUNCE_MS) {
					prevState = LOW;
				}
			}

			// Maintain fixed sample frequency
			// Note that since this is trying to delay only 2 ticks, it may not maintain a very
			// fixed sample frequency. The filters rely on a fixed frequency, so it may make them
			// unreliable.
			vTaskDelayUntil( &xLastWakeTime, 2 );
		}
	}
}
