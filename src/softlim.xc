/*
 * softlim.xc
 *
 *  Created on: 20 maj 2018
 *      Author: micke
 */

#include <stdlib.h>
#include "stepper_server.h"

float softlim(char* data , struct move_t &s){
    float new_x = atoff(data);
    float dx = new_x- s.pos;
    if(s.pos + dx > s.softlim_max){
        dx = s.softlim_max - s.pos;
        s.pos = s.softlim_max;
        s.softlim_char = '+';
    }
    else if(s.pos + dx < s.softlim_min){
        dx = s.softlim_min-s.pos;
        s.pos = s.softlim_min;
        s.softlim_char = '-';
    }else{
        s.pos += dx;
        s.softlim_char = 0;
    }
    return dx;
}
