//Global variables linear motion
volatile int flag_step = 0;
GPIO_pin_t master_pin;
GPIO_pin_t slave_pin;

/**
 * 
**/
void init_hw_pins(){

	pinMode2f(X_STEP_PIN,OUTPUT);
	pinMode2f(X_DIR_PIN, OUTPUT);

	pinMode2f(Y_STEP_PIN,OUTPUT);
	pinMode2f(Y_DIR_PIN, OUTPUT);

	pinMode2f(ENABLE_PIN,OUTPUT);

	pinMode2f(X_END_STOP,INPUT);
	pinMode2f(Y_END_STOP,INPUT);

	servoInit();
		
}



/**
 * Turn on all stepper motors
**/
void motors_on(){
	digitalWrite2f(ENABLE_PIN,LOW);
}
/**
 * Off all stepper motors
**/
void motors_off(){
	digitalWrite2f(ENABLE_PIN,HIGH);
}


void MakeSpeedStep(float v, GPIO_pin_t step_pin){

	digitalWrite2f(step_pin,HIGH);

	delayMicroseconds(1);

	digitalWrite2f(step_pin,LOW);

	delayMicroseconds((1.0/v)*1E6);
	
}

void step(){

	if(flag_step==1){
		digitalWrite2f(master_pin,HIGH);
		delayMicroseconds(1);
		digitalWrite2f(master_pin,LOW);
	}
	
	if(flag_step==2){
		digitalWrite2f(master_pin,HIGH);
		digitalWrite2f(slave_pin,HIGH);
		delayMicroseconds(1);
		digitalWrite2f(master_pin,LOW);
		digitalWrite2f(slave_pin,LOW);
	}

	flag_step = 0;
}


bool can_step(GPIO_pin_t end_pin, long max_length_axis, long current, long target, byte direction){
	if(target == current)
		return false;
	else if (digitalRead2f(end_pin) == HIGH && !direction)
		return false;
	else if (current >= max_length_axis && direction )
		return false;
	return true;
}
