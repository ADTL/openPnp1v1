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
    if(s.acc>1e-9){
        mantissa = 2*FREXP(s.acc , &shift);
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
    s.m.dir = s.steps <0 ? 0 : 1 ;
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
    FLOAT_T acc_inv=scale/s.acc;
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

void calcAccSteps(struct move_t &s , double &scale){
    //unsigned acc_steps= FLOOR(scale*scale/(4*s.acc *s.time.period * s.time.period));
    unsigned acc_steps= FLOOR(scale*scale/(4*s.acc*s.acc *s.time.period * s.time.period));
    unsigned abs_steps = abs(s.steps);
    unsigned max_steps = abs_steps>>1;
    s.m.acc_steps = acc_steps > max_steps ? max_steps : acc_steps;
    s.m.v_steps = abs_steps - 2*s.m.acc_steps;
}

void calcAllMovement(struct moveGroup_t &s){
#ifdef PRINT
    printf("scale = %f\n" , s.scale);
#endif
    for(int i=0; i<3; i++){
        s.XYZ[i].time.period = s.XYZ[i].time.minPeriod;
        s.XYZ[i].acc = s.XYZ[i].maxAcc;
        calcAccSteps(s.XYZ[i] , s.scale);
        calcTimes(s.XYZ[i] , 0 , s.scale);
#ifdef PRINT
        printf("axis = %d, acc_steps = %d , v_steps = %d ,acc=%g, accTime=%g velTime=%g\n" , i , s.XYZ[i].m.acc_steps , s.XYZ[i].m.v_steps, s.XYZ[i].acc, s.XYZ[i].time.acceleration , s.XYZ[i].time.velocity);
#endif
    }
    if ( (s.XYZ[X].steps != 0 )+ (s.XYZ[Y].steps != 0 ) + (s.XYZ[Z].steps != 0 ) > 1){
            //FLOAT_T f1 ,f2;
            int axis=X;
            if(s.XYZ[Y].time.total > s.XYZ[X].time.total )
                axis=Y;
            if(s.XYZ[Z].time.total > s.XYZ[axis].time.total )
                axis=Z;
            unsigned axis1=(axis+1)%3;
            unsigned axis2=(axis+2)%3;

            s.XYZ[axis1].acc *= s.XYZ[axis1].time.total /s.XYZ[axis].time.total;
#ifdef PRINT
            printf("axis = %d, new acc = %g\n" , axis1 , s.XYZ[axis1].acc);
#endif
            FLOAT_T  t_tot = s.XYZ[axis].time.total;
#ifdef PRINT
            printf("t_tot = %g" , t_tot);
#endif
            calcTimes(s.XYZ[axis1] , 0 , s.scale);
            if(s.XYZ[axis1].m.v_steps !=0)
                s.XYZ[axis1].time.period = (t_tot - s.XYZ[axis1].time.acceleration)/  s.XYZ[axis1].m.v_steps;
            s.XYZ[axis1].time.total = t_tot;

            s.XYZ[axis2].acc *= s.XYZ[axis2].time.total /s.XYZ[axis].time.total;
#ifdef PRINT
            printf("axis = %d, new acc = %g\n" , axis2 , s.XYZ[axis2].acc);
#endif
            calcTimes(s.XYZ[axis2] , 0 , s.scale);
            if(s.XYZ[axis2].m.v_steps !=0)
                s.XYZ[axis2].time.period = (t_tot - s.XYZ[axis2].time.acceleration)/  s.XYZ[axis2].m.v_steps;
            s.XYZ[axis2].time.total = t_tot;

   }
    //Translate to fixed point
    for(int i=0; i<3; i++){
        calcFixedPoint(s.XYZ[i]);
#ifdef PRINT
        printf("axis=%d , %u*2^%d , per=%u , t_tot_fixed=%llu \n" ,i ,s.XYZ[i].m.acc_inv , s.XYZ[i].m.acc_exp , (s.XYZ[i].m.period>>32) , s.XYZ[i].m.t_tot);
#endif
    }
}

