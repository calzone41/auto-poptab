/***************************************************
  This is a library for the Adafruit VL6180X Sensor Breakout

  Designed specifically to work with the VL6180X sensor from Adafruit
  ----> https://www.adafruit.com/products/3317

  These sensors use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#if ( ARDUINO >= 100 )
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Wire.h"
#include "vl6180x_api.h"


#define VL6180X_I2C_ADDR  0x29

class Adafruit_VL6180X
{
  public:
    boolean         begin();
    boolean         setAddress(uint8_t newAddr);
    boolean 		getAddress(uint8_t *pAddr);

    int				getSingleRangingMeasurement( VL6180x_RangeData_t *pRangeData);
    
    
    int			   Status = 0;
    
    //New Routines
    boolean         enableContinuousRanging();
    boolean         enableHighSpeedRanging();

    boolean         startRanging();
    boolean         stopRanging();
    int	         	getDataIfReady( VL6180x_RangeData_t *pRangeData);
    
    

 private:
  VL6180xDev_t                       MyDevice;
  VL6180xDev_t                       *pMyDevice  = &MyDevice;
  //VL6180X_Version_t                   Version;
  //VL6180X_Version_t                   *pVersion   = &Version;
  //VL6180X_DeviceInfo_t                DeviceInfo;
};
