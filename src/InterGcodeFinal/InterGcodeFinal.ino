//Libraries
#include <TimerOne.h>
#define  GPIO2_PREFER_SPEED    1
#include "arduino2.h"

#define COMMAND_SIZE 	128 //Buffer size


char palabra[COMMAND_SIZE]; //Buffer gcodes
byte serial_count;			//Chars in buffer
int no_data = 0;			//No data control

void setup(){

	//init serial comunication
	Serial.begin(19200);
	Serial.println("start");

	//init buffer and pins
	init_process_gcodes();
	init_hw_pins();
	
	//Timer interrupt for stepper motors
	Timer1.attachInterrupt(step);
}

void loop(){

	char c;

	if(Serial.available() > 0){

			c = Serial.read();
			no_data = 0;

			if(c != '\n'){
				palabra[serial_count]=c;
				serial_count++;	
			}
	}
	
	else{
		no_data++;
		delayMicroseconds(100);	
	}

	if(serial_count && (c == '\n'|| no_data > 100)){
			
			//process gcode
			process_gcodes(palabra, serial_count);

			//clear command
			init_process_gcodes();
	}
	
	if(no_data > 1000)
		motors_off();
}
