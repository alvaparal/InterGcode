

void move_line(){
		//variables (bresemham)
		long max_delta = 0;
		bool x_can_step = 0;
		bool y_can_step = 0;

		//variables (acceleration)
		long accel_counter = -1;
		unsigned long t0 = sqrt(2.0/ACCEL_AXIS_XY)*676000000; //coma fija
		unsigned long newtp = t0;
		
		//Longest axis
		unsigned long SMax = 0;
		if(delta_steps.x > delta_steps.y){
			SMax = delta_steps.x;         //fallo esto es delta_step.x
			master_pin = X_STEP_PIN;
			slave_pin = Y_STEP_PIN;
		}
		else{
			SMax = delta_steps.y;		  //fallo esto es delta_step.y
			master_pin = Y_STEP_PIN;
			slave_pin = X_STEP_PIN;
		}
		unsigned long countLim = calc_accel_point(SMax);
		
		
		//enable steppers
		motors_on();

		//master axis (bresemham)
		max_delta = max(delta_steps.x,delta_steps.y);

		long x_counter = -max_delta/2;
		long y_counter = -max_delta/2;

		Timer1.initialize(newtp/1000.0);
		
		do{
			
			x_can_step = can_step(X_END_STOP, MAX_LENGTH_X, current_steps.x, target_steps.x, x_direction);
			y_can_step = can_step(Y_END_STOP, MAX_LENGTH_Y, current_steps.y, target_steps.y, y_direction);

					
			if(x_can_step){
				x_counter += delta_steps.x;
				if(x_counter > 0){
						noInterrupts();
						flag_step++;
						interrupts();
						x_counter -= max_delta;
						if(x_direction)
							current_steps.x++;
						else
							current_steps.x--;
				}
			}

			if(y_can_step){
				y_counter += delta_steps.y;
				if(y_counter > 0){
						noInterrupts();
						flag_step++;
						interrupts();
						y_counter -= max_delta;
						if(y_direction)
							current_steps.y++;
						else
							current_steps.y--;
				}
			}
			
			
			if(accel_counter <= countLim)
				newtp = newtp - 2.0*newtp/(4.0*accel_counter+1.0);
			else if ((countLim < accel_counter) && (accel_counter <= (SMax-countLim)))
				;
			else
				newtp = newtp + 2.0*newtp/(4.0*(SMax-accel_counter)+1.0);

			Timer1.setPeriod(newtp/1000.0);		
			Timer1.restart();
			while(flag_step>0);
			
			noInterrupts();
			accel_counter++;
			interrupts();
			
			
		}while(x_can_step || y_can_step);
		
		motors_off();

		set_position(target_units.x,target_units.y,target_units.z);
		
}

long calc_accel_point(long master_axis){
	
	unsigned long countLimSMax = feedrate_angular*feedrate_angular/(2*ACCEL_AXIS_XY);
	unsigned long countAccelLim = master_axis/2;
	unsigned long countLim = (countLimSMax<countAccelLim)?countLimSMax:countAccelLim;
	return countLim;
	
}


/**
 * In this method the targets set, the dirreccion is calculated and transforms steps.
**/
void set_motion(float x, float y, float z){

	//set targets
	target_units.x = x;
	target_units.y = y;
	target_units.z = z;
        
	//figure our delta
	delta_units.x = abs(target_units.x - current_units.x);
	delta_units.y = abs(target_units.y - current_units.y);
	delta_units.z = abs(target_units.z - current_units.z);

	//transforms steps
	target_steps.x = X_STEPS_PER_MM *  target_units.x;
	target_steps.y = Y_STEPS_PER_MM * target_units.y;
	//target_steps.z = 

	delta_steps.x = abs(target_steps.x - current_steps.x);
	delta_steps.y = abs(target_steps.y - current_steps.y);
	//delta_steps.z =
	
	//set direction

	x_direction = (target_units.x >= current_units.x);
	y_direction = (target_units.y >= current_units.y);

	digitalWrite2f(X_DIR_PIN, x_direction);
	digitalWrite2f(Y_DIR_PIN, y_direction);
	
}

/**
 * Updated position
**/
void set_position(float x, float y, float z){

	//set currents
	current_units.x = x;
	current_units.y = y;
	current_units.z = z;

	//transforms steps
	current_steps.x = X_STEPS_PER_MM *  current_units.x;
	current_steps.y = Y_STEPS_PER_MM * current_units.y;
	//current_steps.z = 
        
	
}
/**
 * Gets the angular velocity
 */
void set_feedrate(float feedrate){

	float radio = 6.15;

	feedrate_angular = (feedrate/radio)*FACTOR_STEPING; //rad/min
	
}

/**
 * Lleva a casa la maquina
**/
void home(){
	
	//Servo move up to avoid obstacles
	//servoUp();
	
	//Move home X

	motors_on();
	digitalWrite2f(X_DIR_PIN,DIR_HOMING_X);
	
	
	while(true){		
		if(digitalRead2f(X_END_STOP) == HIGH){
			motors_off();
			break;
		}
		MakeSpeedStep(FEEDRATE_HOME, X_STEP_PIN);
	}

	
	//Move home Y
	digitalWrite2f(Y_DIR_PIN,DIR_HOMING_Y);
	motors_on();
	while(true){			
		if(digitalRead2f(Y_END_STOP) == HIGH){
			motors_off();
			break;
		}
		MakeSpeedStep(FEEDRATE_HOME, Y_STEP_PIN);
	}

	//Move home Z (servo)
	servoDown();		
	set_position(0.0,0.0,0.0);
	
}
