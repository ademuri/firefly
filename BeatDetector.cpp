/*
 * BeatDetector.cpp
 *
 *  Created on: May 30, 2016
 *      Author: Adam
 */

#include "BeatDetector.h"

// Liberally uses code from http://dpeckett.com/beat-detection-on-the-arduino
// Filters generated using http://www.schwietering.com/jayduino/filtuino/index.php

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

// Minimum threshold. Higher values reject more noise, at the expense of ignoring valid beats.
const float MIN_THRESH = 0.1f;
// Maximum threshold. Lower values prevent noise from spiking the threshold. Higher values allow
// more dynamic range.
const float MAX_THRESH = 10.f;
// The amount that the threshold decays per cycle. The sampling code (and thus the decay) runs
// at 500hz.
const float THRESH_DECAY = .001f;
// What ratio of the peak incoming level to set the threshold to.
const float THRESH_RATIO = 0.8f;

void BeatDetector::taskFunc() {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	float sample, value, envelope, beat;
	float thresh = MIN_THRESH;

	const long DEBOUNCE_MS = 100; // at 180bpm there'll be 330ms between beats
	unsigned long prevHighAt = 0;
	bool prevState = LOW;

	// There's a brief spike in the filtered signal. Run through the sampling for a little while
	// to initialize the filter state.
	for (int i = 0; i < 300; i++) {
		sample = (float)analogRead(INPUT_PIN)-855.f;
		value = bassFilter(sample);
		envelope = envelopeFilter(value);
		beatFilter(envelope);
		vTaskDelayUntil( &xLastWakeTime, 2 );
	}

	while(1) {
		for (byte i=0;; i++) {
			// Correct for DC offset so that with no audio signal, our input is about 0.
			sample = (float)analogRead(INPUT_PIN)-855.f;

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

				// If (THRESH_RATIO * beat) > thresh, set thresh := (THRESH_RATIO * beat)
				float maybeNewThresh = beat * THRESH_RATIO;
				if (maybeNewThresh > MAX_THRESH) {
					maybeNewThresh = MAX_THRESH;
				}
				if (maybeNewThresh > thresh) {
#ifdef DEBUG
					if (i % 50 == 0) {
						Serial.println(maybeNewThresh);
					}
#endif
					thresh = maybeNewThresh;
				}

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

			if (thresh > MIN_THRESH) {
				thresh -= THRESH_DECAY;
			} else {
				thresh = MIN_THRESH;
			}

			// Maintain fixed sample frequency
			// Note that since this is trying to delay only 2 ticks, it may not maintain a very
			// fixed sample frequency. The filters rely on a fixed frequency, so it may make them
			// unreliable.
			vTaskDelayUntil( &xLastWakeTime, 2 );
		}
	}
}
