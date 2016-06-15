/*
 * PatternController.cpp
 *
 *  Created on: May 7, 2016
 *      Author: Adam
 */

#include "Led.h"
#include "PatternController.h"
#include "task.h"




void PatternController::cast(void* param) {
	PatternController* ctrl = (PatternController*)param;
	ctrl->taskHandle = xTaskGetCurrentTaskHandle();
	ctrl->taskFunc();
}

void PatternController::setPattern(Pattern pattern, byte r, byte g, byte b) {
	this->led->writeColor(r, g, b);

	if (pattern != this->currentPattern) {
		this->currentPattern = pattern;
		this->r = r;
		this->g = g;
		this->b = b;
		this->taskChanged = true;
		xTaskNotifyGive(this->taskHandle);
	}
}

PatternController::PatternController(Led* led, QueueHandle_t ledQueue) {
	this->led = led;
	this->ledQueue = ledQueue;
	this->taskHandle = NULL;
	this->r = 63;
	this->g = 63;
	this->b = 63;
}

void PatternController::taskFunc() {
	LedMsg offMsg = {0, 0, 0, 1};
	LedMsg onMsg = {255, 255, 255, 100};
	while(1) {
		// If we're here, the pattern changed
		xQueueReset(ledQueue);
		this->taskChanged = false;
		onMsg.r = r;
		onMsg.g = g;
		onMsg.b = b;

		switch(this->currentPattern) {
		case OFF:
			offMsg.duration = 1;
			xQueueSendToFront(ledQueue, &offMsg, 0);
			break;

		case BLINK_ONCE:
			onMsg.duration = 100;
			offMsg.duration = 1;
			xQueueSendToBack(ledQueue, &onMsg, 0);
			xQueueSendToBack(ledQueue, &offMsg, 0);
			this->currentPattern = OFF;
			break;

		case LONG_BLINK:
			onMsg.duration = 1000;
			offMsg.duration = 1;
			xQueueSendToBack(ledQueue, &onMsg, 0);
			xQueueSendToBack(ledQueue, &offMsg, 0);
			this->currentPattern = OFF;
			break;

		case BLINK_TWICE:
			onMsg.duration = 100;
			offMsg.duration = 100;
			xQueueSendToBack(ledQueue, &onMsg, 0);
			xQueueSendToBack(ledQueue, &offMsg, 0);
			xQueueSendToBack(ledQueue, &onMsg, 0);
			xQueueSendToBack(ledQueue, &offMsg, 0);
			this->currentPattern = OFF;
			break;

		case SIMPLE_BLINK:
			onMsg.duration = 100;
			offMsg.duration = 100;
			while (!this->taskChanged) {
				xQueueSendToBack(ledQueue, &onMsg, portMAX_DELAY);
				xQueueSendToBack(ledQueue, &offMsg, portMAX_DELAY);
			}
			break;
		}

		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	} // Must never exit
}
