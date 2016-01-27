

//Parameter settings
#define X_STEPS_PER_MM	5.10*FACTOR_STEPING

#define Y_STEPS_PER_MM  5.10*FACTOR_STEPING

//Units curve
#define CURVE_SECTION_MM 2.5

//Parameters Feedrate
unsigned long	FACTOR_STEPING  =  16;
unsigned long 	FEEDRATE_HOME   =  60*FACTOR_STEPING;		//velocidad angular rad/min
unsigned long   MAX_FEEDRATE_XY = 200*FACTOR_STEPING;		//velocidad angular rad/min

#define DIR_HOMING_X	LOW
#define DIR_HOMING_Y	LOW	

unsigned long 	MAX_LENGTH_X = 10000*FACTOR_STEPING;		//steps
unsigned long 	MAX_LENGTH_Y = 10000*FACTOR_STEPING;		//steps

//#define ACCEL_AXIS_X	2000*FACTOR_STEPING					//aceleracion angular
//#define ACCEL_AXIS_Y	2000*FACTOR_STEPING					//aceleracion angular

unsigned long ACCEL_AXIS_XY = 250*FACTOR_STEPING;			//aceleracion angular


//Hardware Pins

GPIO_pin_t X_STEP_PIN =	DP2;
GPIO_pin_t X_DIR_PIN  = DP3;


GPIO_pin_t Y_STEP_PIN = DP11;
GPIO_pin_t Y_DIR_PIN  =	DP12;


GPIO_pin_t ENABLE_PIN = DP8;

GPIO_pin_t X_END_STOP = DP6;
GPIO_pin_t Y_END_STOP =	DP7;

GPIO_pin_t SERVO_PIN = DP9;
