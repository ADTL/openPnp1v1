/*
 * calcTotalTime.xc
 *
 *  Created on: 11 jun 2018
 *      Author: micke
 */

#include "math.h"
#include "stdio.h"
#include "stepper_server.h"
#include <stdlib.h>

extern unsigned sqrt_tbl(unsigned x , unsigned &a , unsigned &b);
//#define PRINT
void calcFixedPoint(struct move_t &s){
    const FLOAT_T numerator = (FLOAT_T)UINT32_MAX+1;
    int shift;
    //FLOAT_T mantissa = 2*FREXP(SQRT(s.acc) , &shift);
    FLOAT_T mantissa;
    if(s.sqrt_acc>1e-9){
        mantissa = 2*FREXP(s.sqrt_acc , &shift);
        shift-=1;
    }else{
        mantissa = 1;
        shift=0;
    }


    s.m.acc_exp = shift;
    s.m.acc_inv=ROUND(numerator/mantissa);
    FLOAT_T f;
    if(shift < 0)
        f = numerator/(1<<-shift);
    else
        f = numerator*(1<<shift);

    s.m.period = ROUND(s.time.period * f);
    s.m.t_tot = ROUND(s.time.total *f);
    s.m.dir = s.steps <0 ? 1 : 0 ;
#ifdef PRINT
    printf("acc_inv=%x | f=%f, tot=%f , %ull , shift=%d\n", s.m.acc_inv ,f , s.time.total , s.m.t_tot, shift);
#endif
}

void calcTimes(struct move_t &s , int phase , double &scale){
    //unsigned abs_steps = abs(s.steps);
    if(s.steps==0){
        s.time.acceleration=0;
        s.time.velocity=0;
        s.time.total=0;
        return;
    }
    FLOAT_T sqrt_steps = SQRT((FLOAT_T)s.m.acc_steps);
    FLOAT_T sqrt_steps_1 = SQRT((FLOAT_T)(s.m.acc_steps-1));
    //FLOAT_T acc_sqrt_inv=scale/SQRT(s.acc);
    FLOAT_T acc_inv=scale/s.sqrt_acc;
    FLOAT_T acceleration_time = (sqrt_steps + sqrt_steps_1)*acc_inv;
    if(phase == 0)
        s.time.period = (sqrt_steps - sqrt_steps_1)*acc_inv;
    else{
        s.time.period = (s.time.total - acceleration_time)/s.m.v_steps;
#ifdef PRINT
        printf("total_time = %g , acc_time = %g ", s.time.total , acceleration_time);
#endif
    }
    if( s.time.period < s.time.minPeriod){
#ifdef PRINT
        printf("per was %f , minPer=%f\n", s.time.period , s.time.minPeriod);
#endif
        s.time.period   = s.time.minPeriod;
    }
    s.time.acceleration = acceleration_time;
    s.time.velocity     = s.time.period * s.m.v_steps;
    s.time.total        = s.time.acceleration + s.time.velocity;
}

void calcSquares(struct move_t &s){
    s.sqrt_maxAcc = SQRT(s.maxAcc);
    s.sqrt_acc = SQRT(s.acc);
}

void calcAccSteps(struct move_t &s , double &scale){
    //unsigned acc_steps= FLOOR(scale*scale/(4*s.acc *s.time.period * s.time.period));
    unsigned acc_steps= FLOOR(scale*scale/(4*s.acc *s.time.period * s.time.period));
    unsigned abs_steps = abs(s.steps);
    unsigned max_steps = abs_steps>>1;
    s.m.acc_steps = acc_steps > max_steps ? max_steps : acc_steps;
    s.m.v_steps = abs_steps - 2*s.m.acc_steps;
}

void calcAllMovement(struct moveGroup_t &s){
#ifdef PRINT
    printf("scale = %f\n" , s.scale);
#endif


    //Update struct and then calculate the minimum travel time for each axis independently
    for(int i=0; i<3; i++){
        if(s.XYZ[i].steps !=0){
            s.XYZ[i].time.period = s.XYZ[i].time.minPeriod;
            s.XYZ[i].acc = s.XYZ[i].maxAcc;
            s.XYZ[i].sqrt_acc = s.XYZ[i].sqrt_maxAcc;
            calcAccSteps(s.XYZ[i] , s.scale);
            calcTimes(s.XYZ[i] , 0 , s.scale);
        }
    }
    //Number of moving axis
    int moving_axis = (s.XYZ[X].steps != 0 )+ (s.XYZ[Y].steps != 0 ) + (s.XYZ[Z].steps != 0 );

    if ( moving_axis > 1){
            //Find the axis that takes the longest time to complete.
            int axis=X;
            if(s.XYZ[Y].time.total > s.XYZ[X].time.total )
                axis=Y;
            if(s.XYZ[Z].time.total > s.XYZ[axis].time.total )
                axis=Z;
            // s.XYZ[axis].time.total is the shortest time possible, but it might violate the speed/Acc of other axis.
            FLOAT_T scaledown_acc=1;
            FLOAT_T scaleup_period=1;

            //Calculate fastest possible acceleration and period time based on all axis
            FLOAT_T step_scaling[3];
            for( int a=1; a<3; a++){
                int i = (axis+a)%3;
                if(s.XYZ[i].steps !=0){
                    step_scaling[i] = (FLOAT_T )s.XYZ[axis].steps / (FLOAT_T )s.XYZ[i].steps;
                    printf("stepscaling=%g\n",step_scaling[i]);
                     //Check for fastest possible acceleration over all axis
                    FLOAT_T scale_acc = s.XYZ[i].maxAcc /(s.XYZ[axis].acc/step_scaling[i]);
                    //Check for minimum period time over all axis
                    FLOAT_T scale_period = s.XYZ[i].time.minPeriod/(s.XYZ[axis].time.period*step_scaling[i]);
                    if(scale_acc < scaledown_acc){ //<<1
                        scaledown_acc = scale_acc;
                    }
                    if(scale_period > scaleup_period){ //>>1
                        scaleup_period = scale_period;
                    }
                }
            }
            printf("acc_down = %g period_up = %g\n" , scaledown_acc , scaleup_period );

                if(scaledown_acc < 1){
                    //Redo acceleration for axis, meaning slowest axis must go slower
                    s.XYZ[axis].acc *=scaledown_acc;
                    s.XYZ[axis].sqrt_acc =sqrt(s.XYZ[axis].acc);
                }
                if(scaleup_period > 1)
                    s.XYZ[axis].time.period *=scaleup_period;

                //Update struct with acceleration and period for the other axis bases on updated axis
                for( int a=1; a<3; a++){
                    int i = (axis+a)%3;
                    if(s.XYZ[i].steps !=0){
                        s.XYZ[i].acc = s.XYZ[axis].acc / step_scaling[i];
                        s.XYZ[i].sqrt_acc =SQRT(s.XYZ[i].acc);
                        s.XYZ[i].time.period = s.XYZ[axis].time.period * step_scaling[i];

                    }
                }

                //Recalculate all axis based on updated period and acceleration
                   for(int i=0; i<3; i++){
                       if(s.XYZ[i].steps !=0){
                           calcAccSteps(s.XYZ[i] , s.scale);
                           calcTimes(s.XYZ[i] , 0 , s.scale);
                       }
                   }

#ifdef PRINT
                printf("axis = %d, new acc = %g\n" , axis1 , s.XYZ[axis1].sqrt_acc);
#endif
#ifdef PRINT
                printf("t_tot = %g" , t_tot);
#endif
               // Correct for the rounding errors due to discrete steps based on total time by stretching acceleration and then calculate the optimal period
                for( int a=1; a<3; a++){
                    int i = (axis+a)%3;
                FLOAT_T correction = s.XYZ[axis].time.total /s.XYZ[i].time.total;
                printf("Correction = %g \n" , correction-1);
                s.XYZ[i].time.period *=correction;
                s.XYZ[i].sqrt_acc /= correction;
                calcTimes(s.XYZ[i] , 0 , s.scale);
            }

    } //if ( moving_axis > 1)

    //Translate to fixed point
    for(int i=0; i<3; i++){
        calcFixedPoint(s.XYZ[i]);
#ifdef PRINT
        printf("axis=%d , %u*2^%d , per=%u , t_tot_fixed=%llu \n" ,i ,s.XYZ[i].m.acc_inv , s.XYZ[i].m.acc_exp , (s.XYZ[i].m.period>>32) , s.XYZ[i].m.t_tot);
#endif
    }
}

