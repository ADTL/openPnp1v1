/*
 * simStepper.xc
 *
 *  Created on: 20 maj 2018
 *      Author: micke
 */

#include <xs1.h>
#include "stepper_server.h"
#define GAIN 1

int simStepper(unsigned total_steps  , unsigned minPulseTime , unsigned acc){
    acc = acc<<24;
    unsigned t=0 , dt , steps=0;
    unsigned half_way = total_steps;
    total_steps <<=1;
    unsigned max_speed_steps=0;
    long long v = (long long)1<<48;
    while(steps < half_way){
        steps++;
        dt = 0xFFFFFFFF / (unsigned)(v>>32);
        t +=dt;
        if(dt > minPulseTime)
            v += (long long)acc*(dt<<GAIN);
        else{
            dt=minPulseTime;
            max_speed_steps++;
        }
    }
    while(steps <= total_steps ){
        steps++;
        dt = 0xFFFFFFFF / (unsigned)(v>>32);
        t +=dt;
        if(max_speed_steps>0)
            max_speed_steps--;
        else{
            if(dt < MAX_PUSLE_TIME)
                v -= (long long)acc*(dt<<GAIN);
            else
                dt=MAX_PUSLE_TIME;
        }
    }
    return t;
}
