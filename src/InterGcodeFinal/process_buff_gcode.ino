//movement points
struct LongPoint{
	long x;
	long y;
	long z;
	
};

struct FloatPoint{
	float x;
	float y;
	float z;
};


FloatPoint current_units; //current positions
FloatPoint target_units; 
FloatPoint delta_units;

//steps

LongPoint current_steps; //current positions
LongPoint target_steps; 
LongPoint delta_steps;

//direction axis
byte x_direction = 1;
byte y_direction = 1;
//byte z_direction = 1;

//absolute mode
bool abs_mode = true;

//feedrate variables
float feedrate = 0.0;
float lastfeedrate = 0.0;
float feedrate_angular = 0.0;


/**
 * Read and execute gcodes
**/
void process_gcodes(char buffer[], int buffer_size){

	FloatPoint fp;
	fp.x = 0.0;
	fp.y = 0.0;
	fp.z = 0.0;

	byte command = 0;

	
	//gcodes?
	if(
		find_command('G', buffer,buffer_size)||
		find_command('X', buffer,buffer_size)||
		find_command('Y', buffer,buffer_size)||
		find_command('Z', buffer,buffer_size)	
	){
		//search gcode type g
		command = (int) parse_Number('G',buffer,buffer_size);
		
		//Get coordinates
		switch(command){
			case 0:
			case 1:
			case 2:
			case 3:
				if(abs_mode){
						if(find_command('X', buffer, buffer_size)){
							fp.x = parse_Number('X',buffer,buffer_size);}
						else
							fp.x = current_units.x;

						if(find_command('Y', buffer, buffer_size))
							fp.y = parse_Number('Y',buffer,buffer_size);
						else
							fp.y = current_units.y;

						if(find_command('Z', buffer, buffer_size))
							fp.z = parse_Number('Z',buffer,buffer_size);
						else
							fp.z = current_units.z;
				}

				else{
					fp.x = parse_Number('X', buffer, buffer_size) + current_units.x;
					fp.y = parse_Number('Y', buffer, buffer_size) + current_units.y;
					fp.z = parse_Number('Z', buffer, buffer_size) + current_units.z;
				}
				break;
		}

		//servo control before movement
		if(fp.z>0)
			servoUp();
		else
			servoDown();
		
		switch(command){
			//Linear motion
			case 0:
			case 1:
					//set target
					set_motion(fp.x,fp.y,fp.z);
					
					if(find_command('G',buffer,buffer_size)){
						if(command == 1){
							//assigned feedrate
							feedrate = parse_Number('F',buffer,buffer_size);
							
							if(feedrate>0){
									set_feedrate(feedrate);
							}
							//no feedrate
							else{
								if (lastfeedrate>0)
								{
									set_feedrate(lastfeedrate);	
								}	
								else
									feedrate_angular = MAX_FEEDRATE_XY;				
							}					
						}
						else{
							feedrate_angular = MAX_FEEDRATE_XY;
						}
					}
					else{
						if (feedrate > 0)
							set_feedrate(feedrate);
						else
							feedrate_angular = MAX_FEEDRATE_XY;	
					} 
					//linear motion
					move_line();
			break;
			//Arc motion
			case 2:
			case 3:
				FloatPoint center;

				center.x = parse_Number('I',buffer,buffer_size) + current_units.x;
				center.y = parse_Number('J',buffer,buffer_size) + current_units.y;

				float angleA, angleB, angle, radius, length, aX, aY, bX, bY;

				aX = (current_units.x - center.x);
				aY = (current_units.y - center.y);
				bX = (fp.x - center.x);
				bY = (fp.y - center.y);

				
				if(command == 2){ //clockwise
					angleA = atan2(bY,bX);
					angleB = atan2(aY,aX);
				}
				else{ //counterclockwise
					angleA = atan2(aY,aX);
					angleB = atan2(bY,bX);
				}

				// Make sure angleB is always greater than angleA
				if(angleB <= angleA) angleB += 2*M_PI;
				angle = angleB - angleA;

				radius = sqrt(aX*aX + aY*aY);
				length = radius * angle;
				int portions, portion;
				portions = (int) ceil(length/CURVE_SECTION_MM);

				FloatPoint auxPoint;
				for(int p = 1; p<=portions; p++){
					portion = (command == 3) ? p : portions - p; //Work backwards
					auxPoint.x = center.x + radius * cos(angleA + angle *((float) portion/portions));
					auxPoint.y = center.y + radius * sin(angleA + angle *((float) portion/portions));
					set_motion(auxPoint.x,auxPoint.y,fp.z);

					if(feedrate > 0)
						set_feedrate(feedrate);
					else{
						if(lastfeedrate > 0)
							set_feedrate(lastfeedrate);
						else
							feedrate_angular = MAX_FEEDRATE_XY;
					}
					//linear motion
					move_line();
				}

				
			break;
			//Delay
			case 4:
				delay((int)parse_Number('P',buffer,buffer_size));
			break;
			//Go home
			case 28:
				home();
			break;
			//Absolute position on
			case 90:
				abs_mode = true;
			break;
			//Incremental position on
			case 91:
				abs_mode = false;
			break;
			//Set as home
			case 92:
				set_position(0.0, 0.0, 0.0);
			break;
			default:
				;//Serial.println(command,DEC);
		}
            }

		if (find_command('M', buffer, buffer_size))
		{
			command = (int) parse_Number('M',buffer,buffer_size);
			switch (command){
				case 0:
					true;
				break;
				case 3:
					motors_on();
					Serial.println("MOTORS ON");
				break;
				case 5:
					motors_off();
					Serial.println("MOTORS OFF");
				break;
				default:
					;//Serial.println(command,DEC);
			}	
			
	        }
}

/**
 * Init our buffer processing
**/
void init_process_gcodes(){
	
		for (byte i = 0; i<COMMAND_SIZE; i++)
			palabra[i] = 0;	
		
		serial_count = 0;
}

/**
 * Returns the number there after the command
**/
float parse_Number(char key, char buffer[], int buffer_size){

	char aux[10] = "";
	
	for (byte i = 0; i < buffer_size; i++){
  
		if (buffer[i]==key){
			
			i++;
			int j = 0;
			while (i < buffer_size && j < 10){

					if(buffer[i] == 0 || buffer[i] == ' ')
						break;

					aux[j] = buffer[i];
					i++;
					j++;
			}
			return strtod(aux, NULL);
			//return (float) aux;		
		}		
	}
	return 0;
}


/**
 * Search for the command if it exists.
 */
bool find_command(char key, char buffer[], int buffer_size){

	for	(byte i = 0; i<buffer_size; i++){
		
		if(buffer[i]==key)
			return true;	
	}

	return false;
	
}
