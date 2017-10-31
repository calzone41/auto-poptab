#ifndef _INC_SERVOSIGNALSCALER
#define _INC_SERVOSIGNALSCALER

#ifndef _INC_DEADLIFTUTILS
#include "DeadLiftUtils.h"
#endif

class ServoSignalScaler{

    public:
        //transmitter data
        int channel_min[9];     //min time in microseconds
        int channel_neutral[9]; //neutral position time in microseconds
        int channel_max[9];     //max position in microseconds
        
        
        //servo data
        int left_motor_min;
        int left_motor_neutral;
        int left_motor_max;
        int right_motor_min;  
        int right_motor_neutral;  
        int right_motor_max;  
        int left_lifter_min;
        int left_lifter_max;
        int right_lifter_min; 
        int right_lifter_max; 
        
        int pw2scale(int channel, int width);
        int scale2pw(servomotor_enum servo, int scale);
        void initializeScoutBotValues();
        void initializeDeadLiftValues();
    
};

#endif // _INC_SERVOSIGNALSCALER
