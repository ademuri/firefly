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
	SIMPLE_BLINK,	// On and off repeatedly
};

class PatternController {
private:
	Pattern currentPattern = OFF;
	bool taskChanged = false;
	QueueHandle_t ledQueue;
	TaskHandle_t taskHandle;
	byte brightness;

	/**
	 * FreeRTOS task loop.
	 */
	void taskFunc();
public:
	PatternController(QueueHandle_t ledQueue);

	void setPattern(Pattern, byte brightness);

	/**
	 * Glue for FreeRTOS.
	 */
	static void cast(void *params);
};



#endif /* PATTERNCONTROLLER_H_ */
