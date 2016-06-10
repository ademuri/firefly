/*
 * PatternController.h
 *
 *  Created on: May 7, 2016
 *      Author: Adam
 */

#ifndef PATTERNCONTROLLER_H_
#define PATTERNCONTROLLER_H_

#include "Arduino_FreeRTOS.h"
#include "queue.h"
#include "task.h"

enum Pattern {
	OFF,
	BLINK_ONCE,
	BLINK_TWICE,
	SIMPLE_BLINK,	// On and off repeatedly
};

class PatternController {
private:
	Pattern currentPattern = OFF;
	bool taskChanged = false;
	Led* led;
	QueueHandle_t ledQueue;
	TaskHandle_t taskHandle;
	byte r;
	byte g;
	byte b;

	/**
	 * FreeRTOS task loop.
	 */
	void taskFunc();
public:
	PatternController(Led* led, QueueHandle_t ledQueue);

	/**
	 * Uses the given pattern.
	 */
	void setPattern(Pattern, byte r, byte g, byte b);

	/**
	 * Glue for FreeRTOS.
	 */
	static void cast(void *params);
};



#endif /* PATTERNCONTROLLER_H_ */
