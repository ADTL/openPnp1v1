/*
 * wait.xc
 *
 *  Created on: 7 maj 2018
 *      Author: micke
 */

#include <xs1.h>

void wait(unsigned time){
    unsigned t;
    timer tmr;
    tmr :> t;
    tmr when timerafter(t + time):> void;
}
