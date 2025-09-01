/*
 * InterruptEncoder.h
 *
 *  Created on: Oct 8, 2020
 *      Author: hephaestus
 */

#ifndef INTERRUPTENCODER_H_
#define INTERRUPTENCODER_H_

#define MAX_ENCODERS 16
#define US_DEBOUNCE 10
#include <Arduino.h>

class InterruptEncoder {
private:
bool attached=false;



public:

	int apin=0;
	int bpin=0;
	InterruptEncoder();
	virtual ~InterruptEncoder();
	void attach(int aPinNum, int bPinNum);
	volatile bool aState=0;
	volatile bool bState=0;
	volatile int64_t count=0;
	volatile unsigned long microsLastA=0;
	volatile unsigned long microsLastI=0;
	volatile int64_t microsTimeBetweenTicks=0;
	int64_t getCount();
	void setCount(int64_t val);
	void clearCount(void);
};

#endif /* INTERRUPTENCODER_H_ */
