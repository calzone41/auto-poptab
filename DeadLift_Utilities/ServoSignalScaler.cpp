
#include "ServoSignalScaler.h"
#include "DeadLiftUtils.h"
//#include <math.h>
#include <algorithm>

using namespace std;

int ServoSignalScaler::pw2scale(int channel, int width){
    
    int index = channel -1; //because index=channel -1
    if(width < channel_min[index]) {
        //error width lower than should be possible, there must be a mistake, perhaps the channel number is wrong?
        return -100;        
    }else if (channel_min[index] <= width && width <= channel_neutral[index]){
        //scale the time according to percentage of bottom range. (width - min)/(neutral - min)*100 -100
        return (float(width - channel_min[index])/float(channel_neutral[index] - channel_min[index])*100) -100;
    }else if (channel_neutral[index] < width && width <= channel_max[index]){
        //scale the time according to percentage of bottom range. (width - neutral)/(max - neutral)*100
        return (float(width - channel_neutral[index])/float(channel_max[index] - channel_neutral[index])*100);
    }else if (channel_max[index] < width){
        //error width higher than should be possible, there must be a mistake, perhaps the channel number is wrong?
        return 100;
    }else{
        //should not get here, should be literally impossible
        return 0;
    }
    
    
}



int ServoSignalScaler::scale2pw(servomotor_enum servo, int scale){
    
    scale = min(max(scale,-100),100); //ensure that scale is -100 to 100. CONSIDER AN ERROR MESSAGE HERE
    
    int return_val;
    
    if(servo == left_motor){
        if(scale <= 0) {
            //return scaled PW value according to (scale+100)/(100)*(neutral - min) + min
            return_val =(float(scale+100)/100.0*(left_motor_neutral - left_motor_min)) + left_motor_min;
        } else if (scale > 0) {
            //return scaled PW value according to (scale/100)*(max - neutral) + neutral
            return_val= (float(scale)/100.0*(left_motor_max - left_motor_neutral)) + left_motor_neutral;
        }else{
            //should never get here
            return_val =left_motor_neutral;
        }
        
        //provide another layer of capping
        return_val = max(min(return_val, left_motor_max),left_motor_min);
        return return_val;
        
    }else if(servo == right_motor){
        if(scale <= 0) {
            //return scaled PW value according to (scale+100)/(100)*(neutral - min) + min
            return_val = (float(scale+100)/100.0*(right_motor_neutral - right_motor_min)) + right_motor_min;
        } else if (scale > 0) {
            //return scaled PW value according to (scale/100)*(max - neutral) + neutral
            return_val = (float(scale)/100.0*(right_motor_max - right_motor_neutral)) + right_motor_neutral;
        }else{
            //should never get here
            return_val = right_motor_neutral;
        }
        
        //provide another layer of capping
        return_val = max(min(return_val, right_motor_max),right_motor_min);
        return return_val;
        
    }else if(servo == left_lifter){
        int mid;
        
        mid = (left_lifter_min+left_lifter_max)/2;                      
        
        //return scaled PW according to scale/100*(max-min) + min
        //return_val = float(scale)/100.0*(left_lifter_max - left_lifter_min) + left_lifter_min;
        return_val = float(scale)/100.0*(left_lifter_max - mid) + mid;
        
        //provide another layer of capping
        return_val = max(min(return_val, left_lifter_max),left_lifter_min);
        return return_val;
        
    }else if(servo == right_lifter){
        int mid;
        
        mid = (right_lifter_min+right_lifter_max)/2;
        
        //return scaled PW according to scale/100*(max-min) + min
        //return_val = float(scale)/100.0*(right_lifter_max - right_lifter_min) + right_lifter_min;
        return_val = float(scale)/100.0*(right_lifter_max - mid) + mid;
        
        //provide another layer of capping
        return_val = max(min(return_val, right_lifter_max),right_lifter_min);
        return return_val;
        
    }else{
        //should never get here
        return 1500;
    }
    
}

void ServoSignalScaler::initializeScoutBotValues(){



    //channel 1
    channel_min[0]      = 1120;
    channel_neutral[0]  = 1512;
    channel_max[0]      = 1950;
    
    //channel 2
    channel_min[1]      = 1180;
    channel_neutral[1]  = 1520;
    channel_max[1]      = 1870;
    
    //channel 3
    channel_min[2]      = 1090;
    channel_neutral[2]  = 1430;
    channel_max[2]      = 1790;
    
    //channel 4
    channel_min[3]      = 1090;
    channel_neutral[3]  = 1515;
    channel_max[3]      = 1950;
    
    //channel 5
    channel_min[4]      = 1490;
    channel_neutral[4]  = 1750;
    channel_max[4]      = 2030;
    
    //channel 6
    channel_min[5]      = 1490;
    channel_neutral[5]  = 1750;
    channel_max[5]      = 2030;
    
    //channel 7
    channel_min[6]      = 1000;
    channel_neutral[6]  = 1500;
    channel_max[6]      = 2000;
    
    //channel 8
    channel_min[7]      = 1000;
    channel_neutral[7]  = 1500;
    channel_max[7]      = 2000;
    
    //channel 9
    channel_min[8]      = 1000;
    channel_neutral[8]  = 1500;
    channel_max[8]      = 2000;
    
    
    left_motor_min      = 1000;
    left_motor_neutral  = 1500;
    left_motor_max      = 2000;
    
    right_motor_min     = 1000;
    right_motor_neutral = 1500;
    right_motor_max     = 2000;
    
    left_lifter_min     = 1000;
    left_lifter_max     = 2000;
    
    right_lifter_min    = 1000;
    right_lifter_max    = 2000;
    

}

void ServoSignalScaler::initializeDeadLiftValues(){



    //channel 1
    channel_min[0]      = 1120;
    channel_neutral[0]  = 1512;
    channel_max[0]      = 1950;
    
    //channel 2
    channel_min[1]      = 1180;
    channel_neutral[1]  = 1520;
    channel_max[1]      = 1870;
    
    //channel 3
    channel_min[2]      = 1160;
    channel_neutral[2]  = 1455;
    channel_max[2]      = 1840;
    
    //channel 4
    channel_min[3]      = 1090;
    channel_neutral[3]  = 1515;
    channel_max[3]      = 1950;
    
    //channel 5
    channel_min[4]      = 1490;
    channel_neutral[4]  = 1750;
    channel_max[4]      = 2030;
    
    //channel 6
    channel_min[5]      = 1490;
    channel_neutral[5]  = 1750;
    channel_max[5]      = 2030;
    
    //channel 7
    channel_min[6]      = 1000;
    channel_neutral[6]  = 1500;
    channel_max[6]      = 2000;
    
    //channel 8
    channel_min[7]      = 1000;
    channel_neutral[7]  = 1500;
    channel_max[7]      = 2000;
    
    //channel 9
    channel_min[8]      = 1000;
    channel_neutral[8]  = 1500;
    channel_max[8]      = 2000;
    
    
    left_motor_min      = 1000;
    left_motor_neutral  = 1500;
    left_motor_max      = 2000;
    right_motor_min     = 1000;
    right_motor_neutral = 1500;
    right_motor_max     = 2000;
    left_lifter_min     = 1295;
    left_lifter_max     = 1915;
    right_lifter_min    = 1195;
    right_lifter_max    = 1815;
    

}
