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

	/*
	 * When a master becomes a slave, it suspends the beat detector thread. When this happens,
	 * xLastWakeTime will be in the past, and so calls to vTaskDelayUntil will return immediately
	 * until xLastWakeTime has been incremented to be now.
	 *
	 * What this means is, when this object's thread is woken up, it will run as many filter cycles
	 * as it missed when it was sleeping (without a delay between cycles). Since this thread has
	 * the highest priority, it'll own the system until it's done.
	 *
	 * This function is to be called when this thread is woken. It resets xLastWakeTime to avoid
	 * this problem.
	 *
	 * See also: http://www.freertos.org/FreeRTOS_Support_Forum_Archive/June_2007/freertos_VTaskDelayUntil_and_VTaskResume_Problem_1764930.html
	 * and the note at: http://www.freertos.org/vtaskdelayuntil.html
	 */
	void resetLastWakeTime();
};




#endif /* BEATDETECTOR_H_ */
