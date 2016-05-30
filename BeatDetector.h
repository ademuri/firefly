/*
 * BeatDetector.h
 *
 *  Created on: May 30, 2016
 *      Author: Adam
 */

#ifndef BEATDETECTOR_H_
#define BEATDETECTOR_H_

#include "Arduino.h"

#include "Arduino_FreeRTOS.h"

// Analog pin 0 == ATmega328P pin 23
#define INPUT_PIN 0

class BeatDetector {
private:
public:
	BeatDetector();

	/**
	 * Loop function that the beat detector task is registered with. Never exits. Takes no params.
	 */
	void taskFunc();

	/**
	 * Glue for FreeRTOS.
	 */
	static void cast(void *params);
};




#endif /* BEATDETECTOR_H_ */
