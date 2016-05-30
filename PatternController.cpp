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

void PatternController::setPattern(Pattern pattern, byte brightness) {
	if (pattern != this->currentPattern) {
		this->currentPattern = pattern;
		this->brightness = brightness;
		this->taskChanged = true;
		xTaskNotifyGive(this->taskHandle);
	}
}

PatternController::PatternController(QueueHandle_t ledQueue) {
	this->ledQueue = ledQueue;
	this->taskHandle = NULL;
	this->brightness = 255;
}

void PatternController::taskFunc() {
	LedMsg offMsg = {0, 0, 0, 100};
	LedMsg onMsg = {255, 255, 255, 100};
	while(1) {
		// If we're here, the pattern changed
		xQueueReset(ledQueue);
		this->taskChanged = false;
		onMsg.r = brightness;
		onMsg.g = brightness;
		onMsg.b = brightness;

		switch(this->currentPattern) {
		case OFF:
			xQueueSendToFront(ledQueue, &offMsg, 0);
			break;

		case BLINK_ONCE:
			xQueueSendToBack(ledQueue, &onMsg, 0);
			xQueueSendToBack(ledQueue, &offMsg, 0);
			this->currentPattern = OFF;
			break;

		case BLINK_TWICE:
			xQueueSendToBack(ledQueue, &onMsg, 0);
			xQueueSendToBack(ledQueue, &offMsg, 0);
			xQueueSendToBack(ledQueue, &onMsg, 0);
			xQueueSendToBack(ledQueue, &offMsg, 0);
			this->currentPattern = OFF;
			break;

		case SIMPLE_BLINK:
			while (!this->taskChanged) {
				xQueueSendToBack(ledQueue, &onMsg, portMAX_DELAY);
				xQueueSendToBack(ledQueue, &offMsg, portMAX_DELAY);
			}
			break;
		}

		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	} // Must never exit
}