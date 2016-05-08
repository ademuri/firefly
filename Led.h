/*
 * Led.h
 *
 *  Created on: May 5, 2016
 *      Author: Adam
 */


#ifndef LED_H_
#define LED_H_

#include "Arduino.h"

#include "Arduino_FreeRTOS.h"
#include "queue.h"

//#define LED_R 3 // ATMega328P pin 5
//#define LED_G 5 // ATMega328P pin 11
//#define LED_B 6 // ATMega328P pin 12

struct LedMsg {
	byte r;
	byte g;
	byte b;
	long duration;
};

class Led {
public:
	// Holds LedMsgs
	QueueHandle_t ledMsgQueue;

	Led(QueueHandle_t);

	/**
	 * Loop function that the LED task is registered with. Never exits. Takes no params.
	 */
	void taskFunc();

	/**
	 * Gamma-corrects the given color and writes it out to the LED.
	 */
	void writeColor(byte r, byte g, byte b);

	/**
	 * Glue for FreeRTOS.
	 */
	static void cast(void *params);
};



#endif /* LED_H_ */
