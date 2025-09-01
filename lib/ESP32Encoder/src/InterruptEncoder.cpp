/*
 * InterruptEncoder.cpp
 *
 *  Created on: Oct 8, 2020
 *      Author: hephaestus
 */
#include "InterruptEncoder.h"
void IRAM_ATTR encoderAISR(void * arg) {
	InterruptEncoder* object=(InterruptEncoder*)arg;
	unsigned long start = micros();
	unsigned long duration = start - object->microsLastA; 
	// object->microsLastI = start; // for debug mainly
	// if time goes unsigned long (2^32) + < 10 us, a valid isr will be debounced, but rarely happend, TODO fix it
	if (duration >= US_DEBOUNCE) {
		object->microsLastA = start;
		object->microsTimeBetweenTicks = duration;
		object->aState = digitalRead(object->apin);
		object->bState = digitalRead(object->bpin);
		if (object->aState == object->bState)
			object->count++;
		else
			object->count--;
	}
}
InterruptEncoder::InterruptEncoder() {}
InterruptEncoder::~InterruptEncoder() {
	if(attached)
		detachInterrupt(digitalPinToInterrupt(apin));
}
int64_t InterruptEncoder::getCount(){
	return count * 2; // TODO overflow, in this project, only used for lifter motor, limited motion range, so is ok
}
void InterruptEncoder::setCount(int64_t val){
	count = val / 2;
}
void InterruptEncoder::clearCount(void){
	count = 0;
}
void InterruptEncoder::attach(int aPinNum, int bPinNum) {
	if(attached)
		return;
	apin = aPinNum;
	bpin = bPinNum;
	pinMode(apin, INPUT_PULLUP);
	pinMode(bpin, INPUT_PULLUP);
	attachInterruptArg(digitalPinToInterrupt(apin), encoderAISR,this, CHANGE);
}

