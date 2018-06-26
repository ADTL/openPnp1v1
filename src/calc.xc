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
#include "assert.h"

static inline
int min(int a , int b){
    return a < b ? a : b;
}

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

    if((s.time.total *f) > UINT64_MAX){
        FREXP((s.time.total*f) , shift);
        int adjust = shift[0]-64;
      //  printf("\nOverflow:%d\n",adjust);
        f = LDEXP(f , -adjust);
        s.m.acc_inv >>= adjust;
        s.m.acc_exp -= adjust;
    }

    assert( (s.time.total *f) <= UINT64_MAX);
    s.m.period = ROUND(s.time.period * f);
    s.m.v_wrap = s.m.period>>(48+s.m.acc_exp);
    //printf("period = %d , wrap = %d" , (unsigned)(s.m.period>>(32+s.m.acc_exp)) , s.m.v_wrap );


    s.m.t_tot = ROUND(s.time.total *f);
   // printullongln(s.m.t_tot>>(32+s.m.acc_exp));
    s.m.dir = s.steps <0 ? 1 : 0 ;
#ifdef PRINT
    printf("Acc=%g , Sqrt_acc=%g , mantissa=%f , shift =%d f=%g\n%Ttot_uint64=%llu\n", s.acc , s.sqrt_acc , mantissa , shift[0], f , s.m.t_tot);
#endif
}

void calcTimes(struct move_t &s , double &scale){
    //unsigned abs_steps = abs(s.steps);
    if(s.steps==0){
        s.time.acceleration=0;
        s.time.velocity=0;
        s.time.total=0;
        return;
    }
    FLOAT_T sqrt_steps , sqrt_steps_1;
    if(s.m.acc_steps>0){
        sqrt_steps = SQRT((FLOAT_T)s.m.acc_steps);
        sqrt_steps_1 = SQRT((FLOAT_T)(s.m.acc_steps-1));
    }
    else{
        sqrt_steps=1;
        sqrt_steps_1=0;
    }

    //FLOAT_T acc_sqrt_inv=scale/SQRT(s.acc);
    FLOAT_T acc_inv=scale/s.sqrt_acc;
    FLOAT_T acceleration_time = (sqrt_steps + sqrt_steps_1)*acc_inv;
    s.time.acceleration = acceleration_time;
    s.time.velocity     = s.time.period * s.m.v_steps;
    s.time.total        = s.time.acceleration + s.time.velocity;
}

void calcSquares(struct move_t &s){
    assert(s.maxAcc>0);
    assert(s.acc>=0);
    assert(s.HomeAcc>0);
    s.sqrt_maxAcc =  SQRT(s.maxAcc);
    s.sqrt_acc =     SQRT(s.acc);
    s.sqrt_HomeAcc = SQRT(s.HomeAcc);
}

void calcAccSteps(struct move_t &s , double &scale){
    //unsigned acc_steps= FLOOR(scale*scale/(4*s.acc *s.time.period * s.time.period));
    assert(s.time.period>0);
    unsigned acc_steps= FLOOR(scale*scale/(4*s.acc *s.time.period * s.time.period));
    unsigned abs_steps = abs(s.steps);
    unsigned max_steps = abs_steps>>1;
    s.m.acc_steps = acc_steps > max_steps ? max_steps : acc_steps;
    s.m.v_steps = abs_steps - 2*s.m.acc_steps;
}

FLOAT_T calcFeedrate2Period(struct move_t &s , float feedrate){
    if(feedrate <=0)
        return MAXFLOAT;
  //  printf("fin=%g ", feedrate);
    feedrate /=secPerMin; // = [mm/s]
    feedrate *=s.StepsPerMm; // [mm/s] * [steps/mm] = steps/s
    FLOAT_T period = (1000*PORT_CLK_FREQ_kHz)/feedrate; // k*clockcyles/ms / (steps/s) = clockcyles/step
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
        calcTimes(s.XYZ[i] , s.scale);
        calcFixedPoint(s.XYZ[i]);
    }
}

//#define PRINTcalcAllMovement
void calcAllMovement(struct moveGroup_t &s){
#ifdef PRINT
    printf("scale = %f\n" , s.scale);
#endif

    //Calculate maximum acceleration and minimum period
    if(s.feedrate<60)
        s.feedrate=60;
    long long hyp2=0;
    FLOAT_T accMax=0 , periodMin=MAXFLOAT;
    FLOAT_T scaledown[Z+1];
    for(int i=X; i<=Z; i++)
        hyp2+=(long long) s.XYZ[i].steps*s.XYZ[i].steps;
    if(hyp2==0)
        return;
    FLOAT_T hyp=SQRT(hyp2);
    int moving_axis=0;
    {
        short axis_acc;
        short axis_period=-1;
        for(int i=X; i<=Z; i++){
            if(s.XYZ[i].steps !=0){
                moving_axis++;
                scaledown[i] = (FLOAT_T)abs(s.XYZ[i].steps)/hyp; // <1
                FLOAT_T acc = s.XYZ[i].maxAcc * scaledown[i];
                if(accMax < acc){
                    accMax = acc;
                    axis_acc =i;
                }

                FLOAT_T period =  calcFeedrate2Period( s.XYZ[i] , s.feedrate* scaledown[i]);//  s.XYZ[i].time.minPeriod / scaledown[i];
                //printf("%g <= %g*%g hyp=%g\n" , period , s.feedrate , scaledown[i] , hyp);
                if( periodMin > period){
                    periodMin = period;
                    axis_period = i;
                }

            }
        }
        if(moving_axis==0)
            return;
            //printint(axis_acc);
        accMax =    s.XYZ[axis_acc].maxAcc;
        periodMin = calcFeedrate2Period( s.XYZ[axis_period] , s.feedrate);
        //calcAccSteps(s.XYZ[i] , s.scale);

    }

    //Calculate stepper moves
    FLOAT_T total_t=0;
    for(int i=X; i<=Z; i++){
        if(s.XYZ[i].steps !=0){
            s.XYZ[i].time.period = periodMin/scaledown[i];
            s.XYZ[i].acc = accMax * scaledown[i];
            s.XYZ[i].sqrt_acc = SQRT(s.XYZ[i].acc);
           // printf("%d | Pre-Calculated period = %g\n" , i , s.XYZ[i].time.period);
            calcAccSteps(s.XYZ[i] , s.scale);
            calcTimes(s.XYZ[i] , s.scale);
            if(s.XYZ[i].time.total > total_t)
                total_t = s.XYZ[i].time.total;
        }
        //printf("%d | Calculated period = %g\n" , i , s.XYZ[i].time.period);
    }



    // Correct for the rounding errors due to discrete steps based on total time by stretching acceleration and then calculate the optimal period
    FLOAT_T correction , correction_max;
    int iterations=0;
    do{
        correction_max=0;
        for(int i=X; i<=Z; i++){
            if(s.XYZ[i].steps !=0){
                assert(s.XYZ[i].time.total>0);
                correction = total_t /s.XYZ[i].time.total;
                if(correction > correction_max)
                    correction_max = correction;
                assert(correction < 2);
                assert(correction > 0.5);
//#define PRINTcorrection
#ifdef PRINTcorrection
                printf("Correction = %.12g \n" , correction-1);
#endif
                s.XYZ[i].time.period *=correction;
                s.XYZ[i].sqrt_acc /= correction;
                calcTimes(s.XYZ[i] , s.scale);
            }
        }
        iterations++;
    }while(iterations<10 && abs(correction_max-1) > 1E-20);
//printintln(iterations);

    //Translate to fixed point
    for(int i=0; i<3; i++){
        if(s.XYZ[i].steps !=0){
           // printf("%d | Final period = %g\n" , i , s.XYZ[i].time.period);
            calcFixedPoint(s.XYZ[i]);
#ifdef PRINTcalcAllMovement
        printf("axis=%d , %u*2^%d , per=%u , t_tot_fixed=%llu \n\n" ,i ,s.XYZ[i].m.acc_inv , s.XYZ[i].m.acc_exp , (s.XYZ[i].m.period>>32) , s.XYZ[i].m.t_tot);
#endif
        }
    }
}

