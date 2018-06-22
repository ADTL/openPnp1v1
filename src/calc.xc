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
    int shift[1];
    //FLOAT_T mantissa = 2*FREXP(SQRT(s.acc) , &shift);
    FLOAT_T mantissa;
    if(s.sqrt_acc>1e-9){
        mantissa = 2*FREXP(s.sqrt_acc , shift);
        shift[0]-=1;
    }else{
        mantissa = 1;
        shift[0]=0;
    }


    s.m.acc_exp = shift[0];
    s.m.acc_inv=ROUND(numerator/mantissa);
    FLOAT_T f;
    if(shift < 0)
        f = numerator/(1<<-shift[0]);
    else
        f = numerator*(1<<shift[0]);

    s.m.period = ROUND(s.time.period * f);
    printullongln(s.m.period);
    s.m.t_tot = ROUND(s.time.total *f);
    s.m.dir = s.steps <0 ? 1 : 0 ;
#ifdef PRINT
    printf("Acc=%g , Sqrt_acc=%g , mantissa=%f , shift =%d f=%g\n%Ttot_uint64=%llu\n", s.acc , s.sqrt_acc , mantissa , shift[0], f , s.m.t_tot);
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
    s.sqrt_maxAcc =  SQRT(s.maxAcc);
    s.sqrt_acc =     SQRT(s.acc);
    s.sqrt_HomeAcc = SQRT(s.HomeAcc);
}

void calcAccSteps(struct move_t &s , double &scale){
    //unsigned acc_steps= FLOOR(scale*scale/(4*s.acc *s.time.period * s.time.period));
    unsigned acc_steps= FLOOR(scale*scale/(4*s.acc *s.time.period * s.time.period));
    unsigned abs_steps = abs(s.steps);
    unsigned max_steps = abs_steps>>1;
    s.m.acc_steps = acc_steps > max_steps ? max_steps : acc_steps;
    s.m.v_steps = abs_steps - 2*s.m.acc_steps;
}

FLOAT_T calcFeedrate2Period(struct move_t &s , float feedrate){
    if(feedrate<=0){
        printf("\n!!!Error in calcFeedrate2Period | feedrate=%g \n\n" , feedrate);
        feedrate=1000;
    }
  //  printf("fin=%g ", feedrate);
    feedrate /=secPerMin; // = [mm/s]
    feedrate *=s.StepsPerMm; // [mm/s] * [steps/mm] = steps/s
    FLOAT_T period = 1000*PORT_CLK_FREQ_kHz/feedrate; // k*clockcyles/ms / (steps/s) = clockcyles/step
    if(period < s.time.minPeriod)
        period= s.time.minPeriod;
  //  printf("%g, per=%g\n" , feedrate , (FLOAT_T)period);
    return period;

}

void calcHomeMovement(struct moveGroup_t &s , int phase , enum OCaxes axes){

    int start=X;
    int stop=Z;
    if(axes<=Z){
        start=axes;
        stop=axes;
        for(int i=X; i<=Z; i++)
            s.XYZ[i].steps=0;
    }

    for(int i=start; i<=stop; i++){
        s.XYZ[i].steps = s.XYZ[i].softlim_max*s.XYZ[i].StepsPerMm;
        if( phase)
            s.XYZ[i].time.period = calcFeedrate2Period(s.XYZ[i] , s.XYZ[i].SlowHomeFeedrate);
        else
            s.XYZ[i].time.period = calcFeedrate2Period(s.XYZ[i] , s.XYZ[i].HomeFeedrate);
        s.XYZ[i].time.period = calcFeedrate2Period(s.XYZ[i] , s.XYZ[i].HomeFeedrate);
        s.XYZ[i].acc = s.XYZ[i].HomeAcc;
        s.XYZ[i].sqrt_acc = s.XYZ[i].sqrt_HomeAcc;
        calcAccSteps(s.XYZ[i] , s.scale);
        calcTimes(s.XYZ[i] , 0 , s.scale);
        calcFixedPoint(s.XYZ[i]);
    }
}

//#define PRINTcalcAllMovement
void calcAllMovement(struct moveGroup_t &s){
#ifdef PRINT
    printf("scale = %f\n" , s.scale);
#endif

    //Update struct and then calculate the minimum travel time for each axis independently
    for(int i=0; i<3; i++){
        if(s.XYZ[i].steps !=0){
            s.XYZ[i].time.period = calcFeedrate2Period(s.XYZ[i] , s.feedrate);
            s.XYZ[i].acc = s.XYZ[i].maxAcc;
            s.XYZ[i].sqrt_acc = s.XYZ[i].sqrt_maxAcc;
            calcAccSteps(s.XYZ[i] , s.scale);
            calcTimes(s.XYZ[i] , 0 , s.scale);
#ifdef PRINTcalcAllMovement
            printf("Total time axis %d=%g period=%g\n" , i ,s.XYZ[i].time.total ,s.XYZ[i].time.period );
#endif
        }
        else
            s.XYZ[i].time.total=0;

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
                    step_scaling[i] = (FLOAT_T )abs(s.XYZ[axis].steps) / (FLOAT_T) abs(s.XYZ[i].steps);
#ifdef PRINTcalcAllMovement
                    printf("axis=%d , stepscaling[%d]=%g s.XYZ[axis].steps=%d\n", axis, i ,step_scaling[i] , s.XYZ[axis].steps);
#endif
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
#ifdef PRINTcalcAllMovement
            printf("acc_down = %g period_up = %g\n" , scaledown_acc , scaleup_period );
#endif
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

#ifdef PRINTcalcAllMovement
                int axis1=(axis+1)%3;
                   printf("axis = %d, new acc = %g\n" , axis , s.XYZ[axis1].sqrt_acc);
#endif
               // Correct for the rounding errors due to discrete steps based on total time by stretching acceleration and then calculate the optimal period
                for( int a=1; a<3; a++){
                    int i = (axis+a)%3;
                FLOAT_T correction = s.XYZ[axis].time.total /s.XYZ[i].time.total;
#ifdef PRINTcalcAllMovement
                printf("Correction = %g \n" , correction-1);
#endif
                s.XYZ[i].time.period *=correction;
                s.XYZ[i].sqrt_acc /= correction;
                calcTimes(s.XYZ[i] , 0 , s.scale);
            }

    } //if ( moving_axis > 1)

    //Translate to fixed point
    for(int i=0; i<3; i++){
        if(s.XYZ[i].steps !=0)
            calcFixedPoint(s.XYZ[i]);
#ifdef PRINTcalcAllMovement
        printf("axis=%d , %u*2^%d , per=%u , t_tot_fixed=%llu \n\n" ,i ,s.XYZ[i].m.acc_inv , s.XYZ[i].m.acc_exp , (s.XYZ[i].m.period>>32) , s.XYZ[i].m.t_tot);
#endif
    }
}

