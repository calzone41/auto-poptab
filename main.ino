// Sweep
// by BARRAGAN <http://barraganstudio.com> 

//////////////////Libraries///////////

#include <Servo.h> 
#include <PulsePosition.h>
#ifndef _INC_SERVOSIGNALSCALER
#include "ServoSignalScaler.h"
#endif
#ifndef _INC_DEADLIFTUTILS
#include "DeadLiftUtils.h"
#endif
#include "Adafruit_VL53L0X_Cal.h"
#include "VL6180X_Cal.h"

///////// enumerations and macros ///////


////////// function declarations
void initialize_sensors();
void boot_reassign_i2c_addresses();
void right_sensor_read_process();
void left_sensor_read_process();
void lifter_sensor_read_process();
void print_ppm_servo_data();
void process_servo_signals();

//////////// Common Variables, assumptions, and configuration//////////////
int left_motor_pin              = 04;
int right_motor_pin             = 05;
int left_lifter_servo_pin       = 02; //TODO: Need to program in a way to match the lifter servos so they don't fight eachoter
int right_lifter_servo_pin      = 03;
int ppm_in_pin                  = 10;
int autonomous_seek_led_pin     = 24; //TODO: write code for these
int autonomous_lifter_led_pin   = 25; //TODO: write code for these
//  I2C Data Pin                = 18    : requires 4.7K resistor pullup
//  I2C Clock Pin               = 19    : requires 4.7K resistor pullup
int right_sensor_xshut_pin      = 35; //:requires pull-up of 10k Ohm
int left_sensor_xshut_pin       = 36; //:requires pull-up of 10k Ohm
int lifter_sensor_xshut_pin 	= 15; //:requires pull-up of 10k Ohm
//  teensy board led pin		= 13

Servo left_motor_servo;  // create servo object to control a servo  
Servo right_motor_servo;  // create servo object to control a servo  
Servo left_lifter_servo;  // create servo object to control a servo  
Servo right_lifter_servo;  // create servo object to control a servo  

PulsePositionInput ppmIn; //create 1 input to receive the pulses

ServoSignalScaler PWscaler;

//assumption: channel 1 is right stick, left/right motion
//assumption: channel 2 is right stick, up/down motion
//assumption: channel ? is auto/manual switch
int translate_channel = 2;
int translate_index = translate_channel -1;
int rotate_channel = 1;
int rotate_index = rotate_channel -1;
int opmode_channel = 6; //TODO: find out what this is.
int opmode_index = opmode_channel -1;
int lifter_channel = 3;
int lifter_index = lifter_channel -1;
int liftmode_channel = 5;
int liftmode_index = liftmode_channel -1;

int serial_count = 0;

servomotor_enum left_motor_index = left_motor;
servomotor_enum right_motor_index = right_motor;
servomotor_enum left_lifter_index = left_lifter;
servomotor_enum right_lifter_index = right_lifter;

int reverse_left_motor = 0;
int reverse_right_motor = 0;
int reverse_left_lifter = 1;
int reverse_right_lifter = 0;

int left_motor_trim = 0; //in scaled units, not PW
int right_motor_trim = 8; //8
int left_lifter_trim = 22; //22
int right_lifter_trim = 0;

float deadband_factor = .02;

op_mode_enum op_mode = unknown;
op_mode_enum lift_mode = unknown;

boolean been_in_manual_once = false;

elapsedMillis ms_since_last_ppm = 0; //ms
boolean signal_loss = false; //flag denoting signal loss
unsigned int signal_timeout_threshold = 65; //ms, 60 = 3 lost ppm signals

elapsedMillis ms_in_autonomous_mode = 0; //ms
//re-use the been_in_manual_once variable as a flag for this
unsigned int autonomous_disable_threshold = 60000; //ms 60,000 = 1 minute

/// for Distance sensors
Adafruit_VL53L0X right_sensor 	= Adafruit_VL53L0X();
Adafruit_VL53L0X left_sensor  	= Adafruit_VL53L0X();
Adafruit_VL6180X lifter_sensor 	= Adafruit_VL6180X();

uint8_t right_sensor_new_address 	= 0x27;
uint8_t left_sensor_new_address  	= 0x28;
uint8_t lifter_sensor_new_address 	= 0x26;

unsigned int sensor_reboot_threshold = 2000;

VL53L0X_RangingMeasurementData_t 	data_struct_right;
VL53L0X_RangingMeasurementData_t 	data_struct_left;
VL6180x_RangeData_t 				data_struct_lifter;

boolean err;

//int serial_count=0;
int reading_right = 0;
int reading_history_right[3] = {0,0,0};
int reading_count_right = 1;
int target_distance_right = 0;
boolean found_target_right = false;

int reading_left = 0;
int reading_history_left[3] = {0,0,0};
int reading_count_left = 1;
int target_distance_left = 0;
boolean found_target_left = false;

int reading_lifter = 0;
int reading_history_lifter[3] = {0,0,0};
int reading_count_lifter = 1;
int target_distance_lifter = 0;
boolean found_target_lifter = false;

int reading_allowable_deviation = 60; //plus/minus that three readings can be in before target detection is asserted
int allowable_left_right_target_difference = 200; //if a target is found left and right, but the distance difference is > this value, then declared not a target.
int minimum_target_detection = 20; //mm
int maximum_target_detection = 600; //mm
int num_sequential_detections_needed = 2; //for target declaration

//for pursuing target
boolean pursuing_target = false;
boolean seeking_target = true;
elapsedMillis ms_pursuing_target = 0;
unsigned int pursue_target_threshold = 250;

//signal arrays
int ppm_data_raw[9]; //array to contain raw ppm widths
int ppm_data_scaled[9]; //array to contain scaled info from RX
int ppm_data_scaled_mod[9]; //array to contain the ppm scaled values that have been modified
int num_ppm; //denotes how many ppm channels are outputting data
    
int servo_data_scaled[4]; //array containing the scaled values to send to the servos 
int servo_data_pw[4]; //array containing the pw values to send to the servos (microseconds)

elapsedMillis ms_since_power_on = 0;
elapsedMillis ms_since_print = 0;
elapsedMillis ms_since_sensor_reboot = 0;

///////// Initialization Code/////////////////
void setup() 
{ 
    
    
    PWscaler.initializeDeadLiftValues();
    
    
    //initialize servos to neutral
    ppm_data_scaled_mod[rotate_index]    	= 0;            
    ppm_data_scaled_mod[translate_index] 	= 0;           
    ppm_data_scaled_mod[lifter_index]  		= -100;
    
    process_servo_signals();
     
    left_motor_servo.writeMicroseconds(   servo_data_pw[left_motor_index]);
    right_motor_servo.writeMicroseconds(  servo_data_pw[right_motor_index]);
    left_lifter_servo.writeMicroseconds(  servo_data_pw[left_lifter_index]);
    right_lifter_servo.writeMicroseconds( servo_data_pw[right_lifter_index]);
    
    //attach to pins
    left_motor_servo.attach(left_motor_pin);  // attaches the servo on pin 20 
    right_motor_servo.attach(right_motor_pin);  // attaches the servo on pin 20 
    left_lifter_servo.attach(left_lifter_servo_pin);  // attaches the servo on pin 20 
    right_lifter_servo.attach(right_lifter_servo_pin);  // attaches the servo on pin 20
    
    /*
    //initialize servos/motors to neutral
    left_motor_servo.writeMicroseconds(   PWscaler.left_motor_neutral);
    right_motor_servo.writeMicroseconds(  PWscaler.right_motor_neutral);
    left_lifter_servo.writeMicroseconds(  PWscaler.left_lifter_min);
    right_lifter_servo.writeMicroseconds( PWscaler.right_lifter_min);
    */
    
    //setup ranging sensors
    Serial.begin(115200);
    // wait until serial port opens for native USB devices
    //while (! Serial) {
        delay(500);
    //}
    
    ppmIn.begin(ppm_in_pin);
    
    
    pinMode(autonomous_seek_led_pin, OUTPUT);           // set pin to output
    pinMode(autonomous_lifter_led_pin, OUTPUT);           // set pin to output
    digitalWrite(autonomous_seek_led_pin, LOW); //disable autonomous seek mode LED
    digitalWrite(autonomous_lifter_led_pin, LOW); //disable autonomous seek mode LED    
  
  	boot_reassign_i2c_addresses();
	initialize_sensors();
   
} 



////////// Main loop /////////////////


// Note: Interrupt service routines are used for:
//      PPM Input (Timer 0)
//      Servo Output (Timer 1)?


void loop() {
    
    //Serial.println(F("Entering main loop\n\n"));
    //int op_mode_ppm_channel = 6; //corresponds to the tx channel connected to the auto/manual switch
    //int op_mode_pulse_width = 0; //the pulse width (microseconds) of the op_mode channel
    //op_mode_enum op_mode = unknown;    //1 = manual, 2 = auto
    
    int i;

    //perform a check to make sure the bot is not powered up in autonomous mode
    //if the bot has never been in manual mode, do not allow it to be in autonomous mode.
    if(been_in_manual_once == false && op_mode != manual){
        op_mode = unknown;
    }
    
    
    //if manual mode, begin while loop to read ppm, assist driving, and drive servos
    while(op_mode == manual) {
        // read ppm signal
        // convert to scaled values
        // do assist functions, and mixing
        // descale servo values
        // send to servo controller routine
        //Serial.println(F("manual mode\n\n"));
        
        digitalWrite(autonomous_seek_led_pin, LOW); //disable the LED for autonomous seek mode
        
        been_in_manual_once = true; //this lets the bot know it's safe to be in autonomous mode later
		ms_in_autonomous_mode = 0; //not in autonomous mode so reset this variable to zero, it is not reset in auto mode
		
        
        num_ppm = ppmIn.available();
        if(num_ppm >0){  //do not do any of the below unless new ppm data is available
            //read ppm signal, scale values
            for (i = 1; i <= num_ppm; i++){
                ppm_data_raw[i-1] = ppmIn.read(i);
                ppm_data_scaled[i-1] = PWscaler.pw2scale(i,ppm_data_raw[i-1]);
            }
            //identify op_mode
            if(-100 <= ppm_data_scaled[opmode_index] && ppm_data_scaled[opmode_index] <= -10) {
                op_mode = manual;
            } else if (10 <= ppm_data_scaled[opmode_index] && ppm_data_scaled[opmode_index] <= 100) {
                op_mode = autonomous;
            }else{
                op_mode = unknown;
            }
            //identify lifter mode
            if(-100 <= ppm_data_scaled[liftmode_index] && ppm_data_scaled[liftmode_index] <= -10) {
                lift_mode = manual;
                digitalWrite(autonomous_lifter_led_pin, LOW); //disable the LED for autonomous lifter mode
            } else if (10 <= ppm_data_scaled[liftmode_index] && ppm_data_scaled[liftmode_index] <= 100) {
                lift_mode = autonomous;
                digitalWrite(autonomous_lifter_led_pin, HIGH); //enable the LED for autonomous lifter mode
            }else{
                lift_mode = unknown;
                digitalWrite(autonomous_lifter_led_pin, LOW); //disable the LED for autonomous lifter mode
            }              
            
            ms_since_last_ppm = 0;                 
            signal_loss = false;
        }
         
        //sensor read
        right_sensor_read_process();
    	left_sensor_read_process();
    	lifter_sensor_read_process();
         
		         
        if(num_ppm >0){    //process servo signals for pass through
            
            
            //do assist functions, mixing
            //note assumptions listed about channel meanings
            
            //deadband - steering signals are set to zero if they are close to zero. The deadband widens as the speed increases.
            //if within 5% of forward signal, set to zero
            
            float deadband_limit = deadband_factor*abs(ppm_data_scaled[translate_index]);
            if(abs(ppm_data_scaled[rotate_index]) <= deadband_limit){
                ppm_data_scaled_mod[rotate_index] = 0;
            } else { //signal is greater than limit, need to subtract the deadband limit from the signal to make sure there is not a sharp transition
                if(ppm_data_scaled[rotate_index] > 0){
                    ppm_data_scaled_mod[rotate_index] = ppm_data_scaled[rotate_index] - deadband_limit;
                } else {
                    ppm_data_scaled_mod[rotate_index] = ppm_data_scaled[rotate_index] + deadband_limit;
                }
            }
            ppm_data_scaled_mod[translate_index] = ppm_data_scaled[translate_index]; //no modifications for this, so pass through            
            
            
            if(lift_mode == manual){
            	ppm_data_scaled_mod[lifter_index]    = ppm_data_scaled[lifter_index]; //no modifications for this, so pass through            
            }else if(lift_mode == autonomous){
            	if(found_target_lifter){
            		ppm_data_scaled_mod[lifter_index]    = 100;
            	}else{
            		ppm_data_scaled_mod[lifter_index]    = -100;
            	}
            }else{ //unknown
            	ppm_data_scaled_mod[lifter_index]    = -100;
            }
            
            process_servo_signals();
            /*
            //mixing
            //mixing is done according to formulas that add the different signals, then scale them to ensure the signals aren't more than [-100, 100]
            float rotate_scaling_factor = 0.7;
            servo_data_scaled[left_motor_index]  = ppm_data_scaled_mod[translate_index] + (rotate_scaling_factor*ppm_data_scaled_mod[rotate_index]);            
            servo_data_scaled[right_motor_index] = ppm_data_scaled_mod[translate_index] - (rotate_scaling_factor*ppm_data_scaled_mod[rotate_index]); 
            
            int max_motor_signal = max(abs(servo_data_scaled[left_motor_index]), abs(servo_data_scaled[right_motor_index]));
            if(max_motor_signal > 100) { //a signal is too high, need to scale them both to shrink them
                servo_data_scaled[left_motor_index]  = servo_data_scaled[left_motor_index]*(100.0/max_motor_signal);
                servo_data_scaled[right_motor_index] = servo_data_scaled[right_motor_index]*(100.0/max_motor_signal);
            }
            //now do lifter servos            
            
            servo_data_scaled[left_lifter_index]   = ppm_data_scaled_mod[lifter_index];
            servo_data_scaled[right_lifter_index]  = ppm_data_scaled_mod[lifter_index];


            //manage trim and reversals	
            if(reverse_left_motor == 1){
                servo_data_scaled[left_motor_index] = -servo_data_scaled[left_motor_index];
            }
            if(reverse_right_motor == 1){
                servo_data_scaled[right_motor_index] = -servo_data_scaled[right_motor_index];
            }
            if(reverse_left_lifter == 1){
                servo_data_scaled[left_lifter_index] = -servo_data_scaled[left_lifter_index];
            }
            if(reverse_right_lifter == 1){
                servo_data_scaled[right_lifter_index] = -servo_data_scaled[right_lifter_index];
            }
            
            
            servo_data_scaled[left_motor_index] += left_motor_trim;
            servo_data_scaled[right_motor_index] += right_motor_trim;
            servo_data_scaled[left_lifter_index] += left_lifter_trim;
            servo_data_scaled[right_lifter_index] += right_lifter_trim;
            
            
            
            
            // descale servo values, send to servo controller routine
            
            servo_data_pw[left_motor_index] =   PWscaler.scale2pw(left_motor,  servo_data_scaled[left_motor_index]);
            servo_data_pw[right_motor_index] =  PWscaler.scale2pw(right_motor, servo_data_scaled[right_motor_index]);
            servo_data_pw[left_lifter_index] =  PWscaler.scale2pw(left_lifter, servo_data_scaled[left_lifter_index]);
            servo_data_pw[right_lifter_index] = PWscaler.scale2pw(right_lifter,servo_data_scaled[right_lifter_index]);
            */
            
            if(ms_since_print >=100){
                print_ppm_servo_data();
                ms_since_print = 0;
            }
            
            
            left_motor_servo.writeMicroseconds(   servo_data_pw[left_motor_index]);
            right_motor_servo.writeMicroseconds(  servo_data_pw[right_motor_index]);
            left_lifter_servo.writeMicroseconds(  servo_data_pw[left_lifter_index]);
            right_lifter_servo.writeMicroseconds( servo_data_pw[right_lifter_index]);            
            
        }
        if(ms_since_last_ppm >= signal_timeout_threshold){
            signal_loss = true;
            op_mode = unknown;
            Serial.println(F("signal loss\n\n"));
            break; //break out of manual loop
        }
        
        /*
        if(ms_since_sensor_reboot >= sensor_reboot_threshold){
		    boot_reassign_i2c_addresses();
			initialize_sensors();
			ms_since_sensor_reboot = 0;
        }
        */
        
            
        
    }
	
	if(op_mode == autonomous){ //resset a bunch of stuff for autonomous mode
		pursuing_target = false; //reset this for use in autonomous mode
		seeking_target = true; //reset this for use in autonomous mode
		reading_count_right = 1; //reset this for use in autonomous mode
		reading_count_left  = 1; //reset this for use in autonomous mode
		found_target_right 	= false;
		found_target_left 	= false;
		found_target_lifter = false;
		boot_reassign_i2c_addresses();
		initialize_sensors();
	}
    //auto mode, begin while loop to manage auto functions
    while(op_mode == autonomous){
        //Serial.println(F("autonomous mode\n\n"));
        
        num_ppm = ppmIn.available();
        if(num_ppm >0){  //do not do any of the below until new ppm data is available
            //Serial.println("got ppm data in auto");
            //read ppm signal, scale values
            for (i = 1; i <= num_ppm; i++){
                ppm_data_raw[i-1] = ppmIn.read(i);
                ppm_data_scaled[i-1] = PWscaler.pw2scale(i,ppm_data_raw[i-1]);
            }
            
           
            
            //identify op_mode
            if(-100 <= ppm_data_scaled[opmode_index] && ppm_data_scaled[opmode_index] <= -10) {
                op_mode = manual;
            } else if (10 <= ppm_data_scaled[opmode_index] && ppm_data_scaled[opmode_index] <= 100) {
                op_mode = autonomous;
            }else{
                op_mode = unknown;
            }            
            //identify lifter mode
            if(-100 <= ppm_data_scaled[liftmode_index] && ppm_data_scaled[liftmode_index] <= -10) {
                lift_mode = manual;
                digitalWrite(autonomous_lifter_led_pin, LOW); //disable the LED for autonomous lifter mode
            } else if (10 <= ppm_data_scaled[liftmode_index] && ppm_data_scaled[liftmode_index] <= 100) {
                lift_mode = autonomous;
                digitalWrite(autonomous_lifter_led_pin, HIGH); //enable the LED for autonomous lifter mode
            }else{
                lift_mode = unknown;
                digitalWrite(autonomous_lifter_led_pin, LOW); //disable the LED for autonomous lifter mode
            }
            
            
            ms_since_last_ppm = 0;
            signal_loss = false;
            
        } 
        if(ms_since_last_ppm >= signal_timeout_threshold){
            signal_loss = true;
            op_mode = unknown;
            Serial.print("signal loss\n");
            break; //break out of autonomous loop
        }
        if(ms_in_autonomous_mode >= autonomous_disable_threshold){
            been_in_manual_once = false; //this will force to user to re-enter manual mode before doing more autonomous activity
            Serial.println(F("autonomous mode timeout\n\n"));
            //force servos/motors to neutral- should remain so until user switches to manual mode
            
            
            left_motor_servo.writeMicroseconds(   PWscaler.left_motor_neutral);
            right_motor_servo.writeMicroseconds(  PWscaler.right_motor_neutral);
            left_lifter_servo.writeMicroseconds(  PWscaler.left_lifter_min);
            right_lifter_servo.writeMicroseconds( PWscaler.right_lifter_min);            
            
            break; //exit autonomous loop
        }
        
        
        
        digitalWrite(autonomous_seek_led_pin, HIGH); //power the LED for autonomous seek mode
                                 
        
	    /////////////// get and process distance sensor readings
	    
    	
    	right_sensor_read_process();
    	left_sensor_read_process();
    	lifter_sensor_read_process();
        	
        //}
                       
                       
        ////////// Do seeking math //////////
        if(found_target_right && !found_target_left){ //on right side
        	ppm_data_scaled_mod[rotate_index]    = 30;            
            ppm_data_scaled_mod[translate_index] = 45;           
            //ppm_data_scaled_mod[lifter_index]    = 0;
        }else if(!found_target_right && found_target_left){// on left side
        	ppm_data_scaled_mod[rotate_index]    = -30;            
            ppm_data_scaled_mod[translate_index] = 45;          
            //ppm_data_scaled_mod[lifter_index]    = 0;
        }else if(found_target_right && found_target_left){// in front
        	ppm_data_scaled_mod[rotate_index]    = 0;            
            ppm_data_scaled_mod[translate_index] = 40;           
            //ppm_data_scaled_mod[lifter_index]    = 0;
        }else{	//keep seeking
        	ppm_data_scaled_mod[rotate_index]    = 70;            
            ppm_data_scaled_mod[translate_index] = 0 ;           
            //ppm_data_scaled_mod[lifter_index]    = 0;
        }
        
        if(lift_mode == manual){
        	ppm_data_scaled_mod[lifter_index]    = ppm_data_scaled[lifter_index]; //no modifications in manual, so pass through            
        }else if(lift_mode == autonomous){
        	if(found_target_lifter){
        		ppm_data_scaled_mod[lifter_index]    = 100;
        	}else{
        		ppm_data_scaled_mod[lifter_index]    = -100;
        	}
        }else{ //unknown
        	ppm_data_scaled_mod[lifter_index]    = -100;
        }
        
        process_servo_signals();
        /*
        //mixing
        //mixing is done according to formulas that add the different signals, then scale them to ensure the signals aren't more than [-100, 100]
        float rotate_scaling_factor = 0.7; //does this make sense in autonomous mode?
        servo_data_scaled[left_motor_index]  = ppm_data_scaled_mod[translate_index] + (rotate_scaling_factor*ppm_data_scaled_mod[rotate_index]);            
        servo_data_scaled[right_motor_index] = ppm_data_scaled_mod[translate_index] - (rotate_scaling_factor*ppm_data_scaled_mod[rotate_index]); 
            
        int max_motor_signal = max(abs(servo_data_scaled[left_motor_index]), abs(servo_data_scaled[right_motor_index]));
        if(max_motor_signal > 100) { //a signal is too high, need to scale them both to shrink them
            servo_data_scaled[left_motor_index]  = servo_data_scaled[left_motor_index]*(100.0/max_motor_signal);
            servo_data_scaled[right_motor_index] = servo_data_scaled[right_motor_index]*(100.0/max_motor_signal);
        }
        //now do lifter servos
        servo_data_scaled[left_lifter_index]   = ppm_data_scaled_mod[lifter_index];
        servo_data_scaled[right_lifter_index]  = ppm_data_scaled_mod[lifter_index];
            
        if(reverse_left_motor == 1){
            servo_data_scaled[left_motor_index] = -servo_data_scaled[left_motor_index];
        }
        if(reverse_right_motor == 1){
            servo_data_scaled[right_motor_index] = -servo_data_scaled[right_motor_index];
        }
        if(reverse_left_lifter == 1){
            servo_data_scaled[left_lifter_index] = -servo_data_scaled[left_lifter_index];
        }
        if(reverse_right_lifter == 1){
            servo_data_scaled[right_lifter_index] = -servo_data_scaled[right_lifter_index];
        }
        
        servo_data_scaled[left_motor_index] += left_motor_trim;
        servo_data_scaled[right_motor_index] += right_motor_trim;
        servo_data_scaled[left_lifter_index] += left_lifter_trim;
        servo_data_scaled[right_lifter_index] += right_lifter_trim;
            
            
        // descale servo values, send to servo controller routine
            
        servo_data_pw[left_motor_index] =   PWscaler.scale2pw(left_motor,  servo_data_scaled[left_motor_index]);
        servo_data_pw[right_motor_index] =  PWscaler.scale2pw(right_motor, servo_data_scaled[right_motor_index]);
        servo_data_pw[left_lifter_index] =  PWscaler.scale2pw(left_lifter, servo_data_scaled[left_lifter_index]);
        servo_data_pw[right_lifter_index] = PWscaler.scale2pw(right_lifter,servo_data_scaled[right_lifter_index]);
        */
        
        if(ms_since_print >=100){
            print_ppm_servo_data();
            ms_since_print = 0;
        }
        
                    
        left_motor_servo.writeMicroseconds(   servo_data_pw[left_motor_index]);
        right_motor_servo.writeMicroseconds(  servo_data_pw[right_motor_index]);
        left_lifter_servo.writeMicroseconds(  servo_data_pw[left_lifter_index]);
        right_lifter_servo.writeMicroseconds( servo_data_pw[right_lifter_index]);
        
        
        if(ms_since_sensor_reboot >= sensor_reboot_threshold){
		    boot_reassign_i2c_addresses();
			initialize_sensors();
			ms_since_sensor_reboot = 0;
        }
        
                     
    }
    
    if(op_mode == unknown){
        //Serial.println(F("unknown mode\n\n"));
        
        //keep trying to read signals
        num_ppm = ppmIn.available();
        if(num_ppm >0){  //do not do any of the below until new ppm data is available
            //read ppm signal, scale values
            for (i = 1; i <= num_ppm; i++){
                ppm_data_raw[i-1] = ppmIn.read(i);
                ppm_data_scaled[i-1] = PWscaler.pw2scale(i,ppm_data_raw[i-1]);
            }
            //identify op_mode
            if(-100 <= ppm_data_scaled[opmode_index] && ppm_data_scaled[opmode_index] <= -10) {
                op_mode = manual;
            } else if (10 <= ppm_data_scaled[opmode_index] && ppm_data_scaled[opmode_index] <= 100) {
                op_mode = autonomous;
            }else{
                op_mode = unknown;
            }       
            //identify lifter mode
            if(-100 <= ppm_data_scaled[liftmode_index] && ppm_data_scaled[liftmode_index] <= -10) {
                lift_mode = manual;
                digitalWrite(autonomous_lifter_led_pin, LOW); //disable the LED for autonomous lifter mode
            } else if (10 <= ppm_data_scaled[liftmode_index] && ppm_data_scaled[liftmode_index] <= 100) {
                lift_mode = autonomous;
                digitalWrite(autonomous_lifter_led_pin, HIGH); //enable the LED for autonomous lifter mode
            }else{
                lift_mode = unknown;
                digitalWrite(autonomous_lifter_led_pin, LOW); //disable the LED for autonomous lifter mode
            }              
            
            ms_since_last_ppm = 0;
            signal_loss = false;
        } 
        if(ms_since_last_ppm >= signal_timeout_threshold){
            signal_loss = true; //still in signal loss
            op_mode = unknown;
        }        
        //NOTE: No need to worry about rollover, will take 49 days for ms_since_last_ppm to rollover to zero
        
        ppm_data_scaled_mod[rotate_index]    	= 0;            
        ppm_data_scaled_mod[translate_index] 	= 0;           
        ppm_data_scaled_mod[lifter_index]  		= -100;
        
        process_servo_signals();
        /*
        //mixing
		//mixing is done according to formulas that add the different signals, then scale them to ensure the signals aren't more than [-100, 100]
		float rotate_scaling_factor = 0.7; //does this make sense in autonomous mode?
		servo_data_scaled[left_motor_index]  = ppm_data_scaled_mod[translate_index] + (rotate_scaling_factor*ppm_data_scaled_mod[rotate_index]);            
		servo_data_scaled[right_motor_index] = ppm_data_scaled_mod[translate_index] - (rotate_scaling_factor*ppm_data_scaled_mod[rotate_index]); 
		    
		int max_motor_signal = max(abs(servo_data_scaled[left_motor_index]), abs(servo_data_scaled[right_motor_index]));
		if(max_motor_signal > 100) { //a signal is too high, need to scale them both to shrink them
		    servo_data_scaled[left_motor_index]  = servo_data_scaled[left_motor_index]*(100.0/max_motor_signal);
		    servo_data_scaled[right_motor_index] = servo_data_scaled[right_motor_index]*(100.0/max_motor_signal);
		}
		//now do lifter servos
		servo_data_scaled[left_lifter_index]   = ppm_data_scaled_mod[lifter_index];
		servo_data_scaled[right_lifter_index]  = ppm_data_scaled_mod[lifter_index];
		    
		if(reverse_left_motor == 1){
		    servo_data_scaled[left_motor_index] = -servo_data_scaled[left_motor_index];
		}
		if(reverse_right_motor == 1){
		    servo_data_scaled[right_motor_index] = -servo_data_scaled[right_motor_index];
		}
		if(reverse_left_lifter == 1){
		    servo_data_scaled[left_lifter_index] = -servo_data_scaled[left_lifter_index];
		}
		if(reverse_right_lifter == 1){
		    servo_data_scaled[right_lifter_index] = -servo_data_scaled[right_lifter_index];
		}
		
		servo_data_scaled[left_motor_index] += left_motor_trim;
		servo_data_scaled[right_motor_index] += right_motor_trim;
		servo_data_scaled[left_lifter_index] += left_lifter_trim;
		servo_data_scaled[right_lifter_index] += right_lifter_trim;
		    
		    
		// descale servo values, send to servo controller routine
		    
		servo_data_pw[left_motor_index] =   PWscaler.scale2pw(left_motor,  servo_data_scaled[left_motor_index]);
		servo_data_pw[right_motor_index] =  PWscaler.scale2pw(right_motor, servo_data_scaled[right_motor_index]);
		servo_data_pw[left_lifter_index] =  PWscaler.scale2pw(left_lifter, servo_data_scaled[left_lifter_index]);
		servo_data_pw[right_lifter_index] = PWscaler.scale2pw(right_lifter,servo_data_scaled[right_lifter_index]);
        */
        
        if(ms_since_print >=100){
            print_ppm_servo_data();
            ms_since_print = 0;
        }
        
                    
        left_motor_servo.writeMicroseconds(   servo_data_pw[left_motor_index]);
        right_motor_servo.writeMicroseconds(  servo_data_pw[right_motor_index]);
        left_lifter_servo.writeMicroseconds(  servo_data_pw[left_lifter_index]);
        right_lifter_servo.writeMicroseconds( servo_data_pw[right_lifter_index]);
       
        /*
        //force servos/motors to neutral while in unknown mode.
        servo_data_pw[left_motor_index] =   PWscaler.left_motor_neutral;
        servo_data_pw[right_motor_index] =  PWscaler.right_motor_neutral;
        servo_data_pw[left_lifter_index] =  PWscaler.left_lifter_min;
        servo_data_pw[right_lifter_index] = PWscaler.right_lifter_min;
        
        if(ms_since_print >=100){
            print_ppm_servo_data();
            ms_since_print = 0;
        }
        
        left_motor_servo.writeMicroseconds(   servo_data_pw[left_motor_index]);
        right_motor_servo.writeMicroseconds(  servo_data_pw[right_motor_index]);
        left_lifter_servo.writeMicroseconds(  servo_data_pw[left_lifter_index]);
        right_lifter_servo.writeMicroseconds( servo_data_pw[right_lifter_index]);
        */
        
        digitalWrite(autonomous_seek_led_pin, LOW); //disable the LED for autonomous seek mode
    }
    
    
    if(signal_loss){
        //Serial.println(F("signal loss mode\n\n"));
        op_mode = unknown;
        if(ms_since_print >=100){
            Serial.println(F("signal loss mode\n\n"));
            ms_since_print = 0;
        }                
    }

    
}


void print_ppm_servo_data(){
    //int i;
    
    Serial.print(ms_since_power_on);
    
    Serial.print("  :  ");
    /*for (i=1; i <= num_ppm; i++) {
        Serial.print(ppm_data_raw[i-1]);
        Serial.print("  ");                
    }*/
    
    Serial.print(ppm_data_raw[0]);Serial.print("  ");
    Serial.print(ppm_data_raw[1]);Serial.print("  ");
    Serial.print(ppm_data_raw[2]);Serial.print("  ");
    Serial.print(ppm_data_raw[3]);Serial.print("  ");
    Serial.print(ppm_data_raw[4]);Serial.print("  ");
    Serial.print(ppm_data_raw[5]);Serial.print("  ");
    
    Serial.print("|  ");
    /*for (i=1; i <= num_ppm; i++) {
        Serial.print(ppm_data_scaled[i-1]);
        Serial.print("  ");                
    }*/
    
    Serial.print(ppm_data_scaled[0]);Serial.print("  ");
    Serial.print(ppm_data_scaled[1]);Serial.print("  ");
    Serial.print(ppm_data_scaled[2]);Serial.print("  ");
    Serial.print(ppm_data_scaled[3]);Serial.print("  ");
    Serial.print(ppm_data_scaled[4]);Serial.print("  ");
    Serial.print(ppm_data_scaled[5]);Serial.print("  ");
    
    Serial.print("|  ");
    /*for (i=1; i <= num_ppm; i++) {
        Serial.print(ppm_data_scaled_mod[i-1]);
        Serial.print("  ");                
    }*/
    
    Serial.print(ppm_data_scaled_mod[0]);Serial.print("  ");
    Serial.print(ppm_data_scaled_mod[1]);Serial.print("  ");
    Serial.print(ppm_data_scaled_mod[2]);Serial.print("  ");
    Serial.print(ppm_data_scaled_mod[3]);Serial.print("  ");
    Serial.print(ppm_data_scaled_mod[4]);Serial.print("  ");
    Serial.print(ppm_data_scaled_mod[5]);Serial.print("  ");
    
    Serial.print("|  ");
    Serial.print(servo_data_scaled[left_motor_index]);Serial.print("  ");
    Serial.print(servo_data_scaled[right_motor_index]);Serial.print("  "); 
    Serial.print(servo_data_scaled[left_lifter_index]);Serial.print("  "); 
    Serial.print(servo_data_scaled[right_lifter_index]);Serial.print("  ");
    
    Serial.print("|  ");
    Serial.print(servo_data_pw[left_motor_index]);Serial.print("  ");
    Serial.print(servo_data_pw[right_motor_index]);Serial.print("  "); 
    Serial.print(servo_data_pw[left_lifter_index]);Serial.print("  "); 
    Serial.print(servo_data_pw[right_lifter_index]);Serial.print("  ");
    
    Serial.print("| ");
    if(op_mode == manual) {
        Serial.print("Manual");
    } else if(op_mode == autonomous){
        Serial.print("Autonomous"); 
    } else if(op_mode == unknown){
        Serial.print("Unknown"); 
    }
    
    Serial.print(" | ");
    if(lift_mode == manual) {
    	Serial.print("Lft_Manual");
    }else if(lift_mode == autonomous){
    	Serial.print("Lft_Auto");
    } else if(lift_mode == unknown){
        Serial.print("Lft_Unknown"); 
    }
    
    Serial.print(" | ");
    Serial.print(reading_left);
    Serial.print(" | ");
    Serial.print(reading_right);
    Serial.print(" | ");
    Serial.print(reading_lifter);
    Serial.print(" | ");
    
    
    if(found_target_right && !found_target_left){ //on right side
    	Serial.print("right");
    }else if(!found_target_right && found_target_left){// on left side
    	Serial.print("left");
    }else if(found_target_right && found_target_left){// in front
    	Serial.print("front");
    }else{	//keep seeking
    	Serial.print("no_target");
    }
    Serial.print("  |   ");
    
    if(found_target_lifter){
    	Serial.print("lifter");
    }else{
    	Serial.print("no)target");
    }
    
    Serial.println();
    

}


void boot_reassign_i2c_addresses(){

	///set new addresses for sensors
  	pinMode(right_sensor_xshut_pin, OUTPUT);           // set pin to output
    pinMode(left_sensor_xshut_pin, OUTPUT);           // set pin to output
    pinMode(lifter_sensor_xshut_pin, OUTPUT);           // set pin to output
    digitalWrite(right_sensor_xshut_pin, LOW); //set 
    digitalWrite(left_sensor_xshut_pin, LOW); //disable sensor    
    digitalWrite(lifter_sensor_xshut_pin, LOW); //disable sensor
    delay(5);
    
    //initialize right
    pinMode(right_sensor_xshut_pin, INPUT); //use pull-up resistor to bring high.
    delay(5);
    Serial.print("booting right sensor .... ");
    if (!right_sensor.begin()) {
        Serial.print("Failure\n");
        //while(1);
    }else{
    	Serial.print("Success\n");
    }
    delay(5);
    
    Serial.print("Changing Address of right sensor... ");
    if(right_sensor.setAddress(right_sensor_new_address)==false){
    	Serial.print("Failure\n");
    }else{
    	Serial.print("Success\n");
    }
    delay(5);
    

	//initialize left
    pinMode(left_sensor_xshut_pin, INPUT); //use pull-up resistor to do this.
    delay(5);
    Serial.print("Booting left sensor... ");
    if (!left_sensor.begin()) {
        Serial.print("Failure\n");
        //while(1);
    }else{
    	Serial.print("Success\n");
    }
    delay(5);
    
    Serial.print("Changing Address of left sensor.... ");
    if(left_sensor.setAddress(left_sensor_new_address)==false){
    	Serial.print("Failure");
    }else{
    	Serial.print("Success\n");
    }
    delay(5); 
    
    //initialize lifter
    pinMode(lifter_sensor_xshut_pin, INPUT); //use pull-up resistor to bring high.
    delay(5);
    Serial.print("booting lifter sensor... ");
    if (!lifter_sensor.begin()) {
        Serial.print("Failure\n");
        //while(1);
    }else{
    	Serial.print("Success\n");
    }
    delay(5);
    
    Serial.print("Changing Address of lifter sensor... ");
    if(lifter_sensor.setAddress(lifter_sensor_new_address)==false){
    	Serial.print("Failure\n");
    }else{
    	Serial.print("Success\n");
    }
    delay(5);

}

void initialize_sensors(){
	
	///////// CONFIGURE SENSORS ///////////
	
	//left sensor
	Serial.print("Enabling continuous ranging mode in right_sensor... ");  
    err = right_sensor.enableContinuousRanging();
    if(err == false){
        Serial.print("Failure\n");
    }else{
    	Serial.print("Success\n");
    }
    delay(5);
        
    Serial.print("Enabling high-speed ranging mode in right_sensor... ");  
    err = right_sensor.enableHighSpeedRanging();
    if(err == false){
        Serial.print("Failure\n");
    }else{
    	Serial.print("Success\n");
    }
    delay(5);      
    
    //right sensor
    Serial.print("Enabling continuous ranging mode in left_sensor... ");  
    err = left_sensor.enableContinuousRanging();
    if(err == false){
        Serial.print("Failure\n");
    }else{
    	Serial.print("Success\n");
    }
    delay(5);
        
    Serial.print("Enabling high-speed ranging mode in left_sensor... ");  
    err = left_sensor.enableHighSpeedRanging();
    if(err == false){
        Serial.print("Failure\n");
    }else{
    	Serial.print("Success\n");
    }
    delay(5);
    
    //lifter sensor
    Serial.print("Enabling continuous ranging mode in lifter sensor... ");  
    err = lifter_sensor.enableContinuousRanging();
    if(err == false){
        Serial.print("Failure\n");
    }else{
    	Serial.print("Success\n");
    }
    delay(5);
    
    Serial.print("Enabling high-speed ranging mode in lifter sensor... ");  
    err = lifter_sensor.enableHighSpeedRanging();
    if(err == false){
        Serial.print("Failure\n");
    }else{
    	Serial.print("Success\n");
    }
    delay(5);
    
    
    ///////////////  START RANGING ///////////////
    
    Serial.print("Starting ranging in right_sensor... ");  
    err = right_sensor.startRanging();
    if(err == false){
        Serial.print("Failure\n");
    }else{
    	Serial.print("Success\n");
    }
    delay(5);
    
    
    Serial.print("Starting ranging in left_sensor... ");  
    err = left_sensor.startRanging();
    if(err == false){
        Serial.print("Failure\n");
    }else{
    	Serial.print("Success\n");
    }
    delay(5);
    
    
    Serial.print("Starting ranging in lifter sensor... ");  
    err = lifter_sensor.startRanging();
    if(err == false){
        Serial.print("Failure\n");
    }else{
    	Serial.print("Success\n");
    }
    delay(5);
    

}

void lifter_sensor_read_process(){
	//now lifter sensor
	int lifter_sensor_status;
	int i;
	
	lifter_sensor_status = lifter_sensor.getDataIfReady(&data_struct_lifter);

	if(lifter_sensor_status!=4){    //reading is good     
		if(data_struct_lifter.errorStatus == 0){
			reading_lifter = data_struct_lifter.range_mm;
			//Serial.print(reading_count); Serial.print("  Distance A (mm): "); Serial.println(reading);
		}else{
			reading_lifter = 0;
			//reading_A = -1;
			//Serial.print(reading_count); Serial.print("  Distance A (mm): "); Serial.println(F("Data not Valid"));
		}
		reading_count_lifter++;
		reading_history_lifter[reading_count_lifter % num_sequential_detections_needed] = reading_lifter; //modulus operator, adds new reading to history buffer	
	}else{
		//data not ready
	}        	        	        	 
	           	
	
	//have past readings in history_lifter, check the history of each neighbor to see if difference within tolerance
	{ //scope block
		int max_diff = 0;
    	for(i=0;i<num_sequential_detections_needed;i++){
    		
    		int diff = 0;
    		diff = abs(reading_history_lifter[(i)%num_sequential_detections_needed] - reading_history_lifter[(i+1)%num_sequential_detections_needed]);
    		max_diff = max(max_diff, diff);
    	}
    	if(	max_diff <= reading_allowable_deviation && //if max is less, and most recent reading is within detection bounds, call it a target
    		reading_history_lifter[reading_count_lifter % num_sequential_detections_needed] >= minimum_target_detection &&
    		reading_history_lifter[reading_count_lifter % num_sequential_detections_needed] <= maximum_target_detection){
			
			target_distance_lifter = reading_history_lifter[reading_count_lifter % num_sequential_detections_needed];
    		found_target_lifter =true;
		}else{
    		target_distance_lifter = 0;
    		found_target_lifter =false;
		}
	}
}

void process_servo_signals(){
	//mixing
    //mixing is done according to formulas that add the different signals, then scale them to ensure the signals aren't more than [-100, 100]
    float rotate_scaling_factor = 0.64; //does this make sense in autonomous mode?
    servo_data_scaled[left_motor_index]  = ppm_data_scaled_mod[translate_index] + (rotate_scaling_factor*ppm_data_scaled_mod[rotate_index]);            
    servo_data_scaled[right_motor_index] = ppm_data_scaled_mod[translate_index] - (rotate_scaling_factor*ppm_data_scaled_mod[rotate_index]); 
        
    int max_motor_signal = max(abs(servo_data_scaled[left_motor_index]), abs(servo_data_scaled[right_motor_index]));
    if(max_motor_signal > 100) { //a signal is too high, need to scale them both to shrink them
        servo_data_scaled[left_motor_index]  = servo_data_scaled[left_motor_index]*(100.0/max_motor_signal);
        servo_data_scaled[right_motor_index] = servo_data_scaled[right_motor_index]*(100.0/max_motor_signal);
    }
    //now do lifter servos
    servo_data_scaled[left_lifter_index]   = ppm_data_scaled_mod[lifter_index];
    servo_data_scaled[right_lifter_index]  = ppm_data_scaled_mod[lifter_index];
        
    if(reverse_left_motor == 1){
        servo_data_scaled[left_motor_index] = -servo_data_scaled[left_motor_index];
    }
    if(reverse_right_motor == 1){
        servo_data_scaled[right_motor_index] = -servo_data_scaled[right_motor_index];
    }
    if(reverse_left_lifter == 1){
        servo_data_scaled[left_lifter_index] = -servo_data_scaled[left_lifter_index];
    }
    if(reverse_right_lifter == 1){
        servo_data_scaled[right_lifter_index] = -servo_data_scaled[right_lifter_index];
    }
    
    servo_data_scaled[left_motor_index] += left_motor_trim;
    servo_data_scaled[right_motor_index] += right_motor_trim;
    servo_data_scaled[left_lifter_index] += left_lifter_trim;
    servo_data_scaled[right_lifter_index] += right_lifter_trim;
        
        
    // descale servo values, send to servo controller routine
        
    servo_data_pw[left_motor_index] =   PWscaler.scale2pw(left_motor,  servo_data_scaled[left_motor_index]);
    servo_data_pw[right_motor_index] =  PWscaler.scale2pw(right_motor, servo_data_scaled[right_motor_index]);
    servo_data_pw[left_lifter_index] =  PWscaler.scale2pw(left_lifter, servo_data_scaled[left_lifter_index]);
    servo_data_pw[right_lifter_index] = PWscaler.scale2pw(right_lifter,servo_data_scaled[right_lifter_index]);
        

}

void right_sensor_read_process(){
	int i;
	
	if(right_sensor.getDataReady()){
        right_sensor.getData(&data_struct_right);        
        if(data_struct_right.RangeStatus != 4){ //reading is good
            reading_right = data_struct_right.RangeMilliMeter;
            //Serial.print(serial_count); Serial.print("  Distance (mm): "); Serial.print(reading_right);Serial.println();
        }else{
            //Serial.print(serial_count); Serial.print("  Distance (mm): "); Serial.print("Data not Valid");Serial.println();
            reading_right = 0;
        }
        if(reading_right >= 8000){
    		reading_right = 0; //catch an error code
    	} 
        reading_count_right++;
		reading_history_right[reading_count_right % num_sequential_detections_needed] = reading_right; //modulus operator, adds new reading to history buffer		        
    }else{
        //Serial.print("data not ready\n");
    }
            
        	
	
	//have past readings in history_right, check the history of each neighbor to see if difference within tolerance
	{ //scope block
		int max_diff = 0;
    	for(i=0;i<num_sequential_detections_needed;i++){
    		
    		int diff = 0;
    		diff = abs(reading_history_right[(i)%num_sequential_detections_needed] - reading_history_right[(i+1)%num_sequential_detections_needed]);
    		max_diff = max(max_diff, diff);
    	}
    	if(	max_diff <= reading_allowable_deviation && //if max is less, and most recent reading is within detection bounds, call it a target
    		reading_history_right[reading_count_right % num_sequential_detections_needed] >= minimum_target_detection &&
    		reading_history_right[reading_count_right % num_sequential_detections_needed] <= maximum_target_detection){
			
			target_distance_right = reading_history_right[reading_count_right % num_sequential_detections_needed];
    		found_target_right =true;
		}else{
    		target_distance_right = 0;
    		found_target_right =false;
		}
	}
}

void left_sensor_read_process(){
	int i;
	
	//now left sensor
	if(left_sensor.getDataReady()){
        left_sensor.getData(&data_struct_left);        
        if(data_struct_left.RangeStatus != 4){ //reading is good
            reading_left = data_struct_left.RangeMilliMeter;
            //Serial.print(serial_count); Serial.print("  Distance (mm): "); Serial.print(reading_left);Serial.println();
        }else{
            //Serial.print(serial_count); Serial.print("  Distance (mm): "); Serial.print("Data not Valid");Serial.println();
            reading_left = 0;
        }
        if(reading_left >= 8000){ //catch an error code
    		reading_left = 0; 
    	} 
        reading_count_left++;
		reading_history_left[reading_count_left % num_sequential_detections_needed] = reading_left; //modulus operator, adds new reading to history buffer		        
    }else{
        //Serial.print("data not ready\n");
    }        
	           	
	
	//have past readings in history_left, check the history of each neighbor to see if difference within tolerance
	{ //scope block
		int max_diff = 0;
    	for(i=0;i<num_sequential_detections_needed;i++){
    		
    		int diff = 0;
    		diff = abs(reading_history_left[(i)%num_sequential_detections_needed] - reading_history_left[(i+1)%num_sequential_detections_needed]);
    		max_diff = max(max_diff, diff);
    	}
    	if(	max_diff <= reading_allowable_deviation && //if max is less, and most recent reading is within detection bounds, call it a target
    		reading_history_left[reading_count_left % num_sequential_detections_needed] >= minimum_target_detection &&
    		reading_history_left[reading_count_left % num_sequential_detections_needed] <= maximum_target_detection){
			
			target_distance_left = reading_history_left[reading_count_left % num_sequential_detections_needed];
    		found_target_left =true;
		}else{
    		target_distance_left = 0;
    		found_target_left =false;
		}
	}
}

