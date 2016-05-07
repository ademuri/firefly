/*
 * Led.c
 *
 *  Created on: May 5, 2016
 *      Author: Adam
 */

#include "Led.h"

#define LED_R 3 // ATMega328P pin 5
#define LED_G 5 // ATMega328P pin 11
#define LED_B 6 // ATMega328P pin 12

void Led::cast(void* param) {
	((Led*)param)->taskFunc();
}


Led::Led(QueueHandle_t ledMsgQueue) {
	this->ledMsgQueue = ledMsgQueue;
}

void Led::taskFunc() {
	LedMsg msg;
	while(1) {
		xQueueReceive(this->ledMsgQueue, &msg, portMAX_DELAY);
		writeColor(msg.r, msg.g, msg.b);
		vTaskDelay(msg.duration);
	}	// Must never exit
}

void Led::writeColor(byte r, byte g, byte b) {
	analogWrite(LED_R, r);
	analogWrite(LED_G, g);
	analogWrite(LED_B, b);
}
