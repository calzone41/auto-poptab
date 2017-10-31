#include "VL6180X_Cal.h"

#define VERSION_REQUIRED_MAJOR  1
#define VERSION_REQUIRED_MINOR  0
#define VERSION_REQUIRED_BUILD  1

#define STR_HELPER( x ) #x
#define STR( x )        STR_HELPER(x)


boolean Adafruit_VL6180X::begin() {
  	//int32_t   status_int;
  	//int32_t   init_done         = 0;

  	//uint32_t  refSpadCount;
  	//uint8_t   isApertureSpads;
  	//uint8_t   VhvSettings;
  	//uint8_t   PhaseCal;

  	// Initialize Comms
  	//pMyDevice->i2c_dev_addr      =  VL6180X_I2C_ADDR;  // default

  	Wire.begin();     // VL6180X_i2c_init();
  	
  	MyDevice = VL6180X_I2C_ADDR; //device is just a uint8_t. Can assign the address to it.

  	initialize_SingleVL6180xDevData();
  	
	Status = VL6180x_InitData(MyDevice); //TODO: returns zero is successful?
	if(Status != 0){
		return false;
	}	
	
	Status = VL6180x_Prepare(MyDevice);  //TODO: returns zero is successful?
	if(Status != 0){
		return false;
	}	
	
	Status = VL6180x_SetupGPIO1(MyDevice, GPIOx_SELECT_OFF, INTR_POL_HIGH);   //TODO: returns zero is successful?
  	if(Status != 0){
		return false;
	}
	
	Status = VL6180x_RangeConfigInterrupt(MyDevice,  CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY);
	if(Status != 0){
		return false;
	}	
	
	/*Status = setAddress(VL6180X_I2C_ADDR) ) {
    if(Status != 0){
		return false;
	}*/	
	
  	return true;
  
}

boolean Adafruit_VL6180X::setAddress(uint8_t newAddr) {
    int Status;
    
    newAddr &= 0x7F;

    Status = VL6180x_SetI2CAddress(MyDevice, newAddr * 2); // 7->8 bit
    
    delay(10);

    if( Status == 0 ) {
        //pMyDevice->i2c_dev_addr = newAddr;  // 7 bit addr
        MyDevice = newAddr;
        return true;
    }
    return false;
}

boolean Adafruit_VL6180X::getAddress(uint8_t *pAddr) {
    int Status;
    
	Status = VL6180x_GetI2CAddress(MyDevice, pAddr);
    
    if( Status == 0 ) {
        
        return true;
    }else{
    	Serial.print("retreive address error code:  ");Serial.println(Status);
    	return false;
   	}
}

int Adafruit_VL6180X::getSingleRangingMeasurement( VL6180x_RangeData_t *pRangeData)
{
    int Status;
    
    
    Status = VL6180x_RangePollMeasurement(MyDevice, pRangeData);
    
    if(pRangeData->errorStatus == 0){ //data is good TODO: validate this
    	return 0;
   	}else{
   		//data is bad
   		return 1;
   	}
    
 
}






boolean Adafruit_VL6180X::enableContinuousRanging(){
    //VL6180X_Error Status = VL6180X_ERROR_NONE;
    int Status = 0;
    
    
    //////////////////TODO: HOW TO ENSURE DEVICE IS STOPPED?////////////////////
    //status=VL6180x_RangeSetSystemMode(theVL6180xDev, MODE_START_STOP | MODE_SINGLESHOT); //put in single shot
    
    
    Status = VL6180x_FilterSetState(MyDevice,0);
    
    Status = VL6180x_RangeStartContinuousMode(MyDevice);
    
    //stop device
    Status=VL6180x_RangeSetSystemMode(MyDevice, MODE_START_STOP | MODE_CONTINUOUS); //toggle bit to stop ranging  
    
    if(Status == 0){
        return true;
    }else{
        return false;
    }

}

boolean Adafruit_VL6180X::enableHighSpeedRanging(){
    //VL6180X_Error Status = VL6180X_ERROR_NONE;
    int Status = 0;
    
    Status = VL6180x_RangeSetInterMeasPeriod(MyDevice, 0);
    
    if(Status == 0){
        return true;
    }else{
        return false;
    }

}

/*boolean Adafruit_VL6180X::enableDefaultSpeedRanging(){
    //VL6180X_Error Status = VL6180X_ERROR_NONE;
    Status = VL6180X_ERROR_NONE;
    
    if (Status == VL6180X_ERROR_NONE) {
        Status = VL6180X_SetLimitCheckValue(pMyDevice, VL6180X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25*65536));
    }
    if (Status == VL6180X_ERROR_NONE) {
        Status = VL6180X_SetLimitCheckValue(pMyDevice, VL6180X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(18*65536));
    }
    if (Status == VL6180X_ERROR_NONE) {
        Status = VL6180X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice, 40000);
    }
    
    if(Status == VL6180X_ERROR_NONE){
        return true;
    }else{
        return false;
    }

}

boolean Adafruit_VL6180X::enableLongRangeRanging(){
    //VL6180X_Error Status = VL6180X_ERROR_NONE;
    Status = VL6180X_ERROR_NONE;
    
    if (Status == VL6180X_ERROR_NONE) {
        Status = VL6180X_SetLimitCheckValue(pMyDevice, VL6180X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
    }
    if (Status == VL6180X_ERROR_NONE) {
        Status = VL6180X_SetLimitCheckValue(pMyDevice,VL6180X_CHECKENABLE_SIGMA_FINAL_RANGE,(FixPoint1616_t)(60*65536));
    }
    if (Status == VL6180X_ERROR_NONE) {
        Status = VL6180X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice, 33000);
    }
    if (Status == VL6180X_ERROR_NONE) {
        Status = VL6180X_SetVcselPulsePeriod(pMyDevice, VL6180X_VCSEL_PERIOD_PRE_RANGE, 18);
    }
    if (Status == VL6180X_ERROR_NONE) {
        Status = VL6180X_SetVcselPulsePeriod(pMyDevice, VL6180X_VCSEL_PERIOD_FINAL_RANGE, 14);
    }
    
    if(Status == VL6180X_ERROR_NONE){
        return true;
    }else{
        return false;
    }
    
}*/

boolean Adafruit_VL6180X::startRanging(){
    //VL6180X_Error Status = VL6180X_ERROR_NONE;
    int Status = 0;
    
    ///// start ranging is a toggle in continuous mode?
    
    //start device
    Status=VL6180x_RangeSetSystemMode(MyDevice, MODE_START_STOP | MODE_CONTINUOUS); //toggle bit to start ranging
    
    if(Status == 0){
        return true;
    }else{
        return false;
    }
}

boolean Adafruit_VL6180X::stopRanging(){
    //VL6180X_Error Status = VL6180X_ERROR_NONE;
    int Status = 0;
    
    //stop device
    Status=VL6180x_RangeSetSystemMode(MyDevice, MODE_START_STOP | MODE_CONTINUOUS); //toggle bit to stop ranging
    
    if(Status == 0){
        return true;
    }else{
        return false;
    }
}

///////IS THERE NO WAY TO DO THIS????///////////////
/*boolean Adafruit_VL6180X::getRangingStopStatus(){
    Status = VL6180X_ERROR_NONE;
    uint32_t stop_status;
    
    Status = VL6180X_GetStopCompletedStatus(pMyDevice, &stop_status);
    
    if(stop_status == 0){ //TODO: Validate assumption that 0 means stopped
        return true;
    }else{
        return false;
    }
        
}*/

int Adafruit_VL6180X::getDataIfReady(VL6180x_RangeData_t *pRangeData){
    //VL6180X_Error Status = VL6180X_ERROR_NONE;
    int Status = 0;
    
    Status = VL6180x_RangeGetMeasurementIfReady(MyDevice, pRangeData);
    
    VL6180x_RangeClearInterrupt(MyDevice);
    
    if(Status == NOT_READY){ //NOT_READY = 4  see def.h line 138              
        return NOT_READY;
    }else if(Status < 0){ //error
        return 1;
    }else{
    	return 0;
   	}
}

/*boolean Adafruit_VL6180X::getDataValid(){
    Status = VL6180X_ERROR_NONE;
    VL6180X_RangingMeasurementData_t data_struct;
    
    Status = VL6180X_GetRangingMeasurementData(pMyDevice, &data_struct);
    
    
    
    if(data_struct.RangeStatus == 0){
        return true;
    }else{
        return false;
    }
    
}

int Adafruit_VL6180X::getRangeMM(){
    Status = VL6180X_ERROR_NONE;
    VL6180X_RangingMeasurementData_t data_struct;
    
    Status = VL6180X_GetRangingMeasurementData(pMyDevice, &data_struct);
    
    if (Status == VL6180X_ERROR_NONE)
		Status = VL6180X_ClearInterruptMask(pMyDevice, 0);
    
    return data_struct.RangeMilliMeter; //note implicit converstion from uint16 to int
}*/

/*
boolean Adafruit_VL6180X::getData(VL6180X_RangingMeasurementData_t *pdata_struct){
    Status = VL6180X_ERROR_NONE;
    //VL6180X_RangingMeasurementData_t data_struct;
    VL6180X_DeviceModes DeviceMode;

	// Get Current DeviceMode 
	Status = VL6180X_GetDeviceMode(pMyDevice, &DeviceMode);
    
    if (Status == VL6180X_ERROR_NONE && DeviceMode == VL6180X_DEVICEMODE_SINGLE_RANGING){
		PALDevDataSet(pMyDevice, PalState, VL6180X_STATE_IDLE);
	}
    
    Status = VL6180X_GetRangingMeasurementData(pMyDevice, pdata_struct);
    
    if (Status == VL6180X_ERROR_NONE){
		Status = VL6180X_ClearInterruptMask(pMyDevice, 0);  
	}  
    
    if(Status == VL6180X_ERROR_NONE){
        return true;
    }else{
        return false;
    }
    
}*/

