/*
 * moveStepper.xc
 *
 *  Created on: 3 jun 2018
 *      Author: micke
 */

#include "xs1.h"
#include "stepper_server.h"
#include <stdlib.h>
#include <math.h>
#include "print.h"

#define MSP 1 //Most significant part of uint32

static unsigned sqrt_tb[SQRT_TB_LEN+1];

#define EXTRACT_UINT32_HI(val) (val ,unsigned[])[1]
#define EXTRACT_UINT32_LO(val) (val ,unsigned[])[0]

#pragma unsafe arrays
static inline
void calc(unsigned * unsafe vec, int start , int stop , double scale){
    unsafe{
        for(int i=start; i<stop ; i++){
            vec[i]=round(scale*sqrt(i));
        }
    }
}

void init_sqrt_tbl(double &scale){
    unsafe{
        unsigned* unsafe ptr = sqrt_tb;
        for(int i=0; i<=SQRT_TB_LEN ; i++)
            ptr[i]=round(scale*sqrt(i));
    }
}

    /*    int delta=SQRT_TB_LEN/5;
    unsafe{
        unsigned* unsafe ptr = sqrt_tb;
        par(int i=0; i<5 ; i++)
            calc(ptr ,delta*i , (i+1)*delta ,   scale);
    calc(ptr, 5*delta , SQRT_TB_LEN+1,scale);
    }
   // for(int i=0; i< SQRT_TB_LEN  ; i++)
   //     printintln(sqrt_tb[i]);
*/

#pragma unsafe arrays
static inline
unsigned sqrt_tbl(unsigned x , unsigned &a , unsigned &b){
    unsigned mod , i , shift;
    if((x >= (SQRT_TB_LEN*4))){
        i = x>>4;
        mod = x & 0b1111;
        shift=2;
    }
    else if(x >= SQRT_TB_LEN){
        i = x>>2; //  /4
        mod = x & 0b11;
        shift=1;
    }
    else
        return sqrt_tb[x];
    if(mod==0 || (b==0)){
        unsigned v0 = sqrt_tb[i];
        unsigned v1 = sqrt_tb[i+1];
        a = v0<<shift;
        b = v1-v0;
    }
    return (a + ((b*mod)>>shift));
}

/*static inline
unsigned calc_Tlong(move_data_t &data , unsigned long long t , unsigned shift){
return data->acc_exp < 0 ? ((unsigned)(t>>32))<<-shift : ((unsigned)(t>>32))>>shift;
}
*/
static inline
void sendPulse(unsigned t , unsigned PulseTime , out port p , unsigned p_val){
    p @ t<: p_val;
    p @ t + PulseTime<: 0;
}

static inline
int sendPulseLim(unsigned t , unsigned PulseTime , out port p , unsigned p_val , in port p_lim , unsigned cont){
    //printuintln(t);
    p @ t<: p_val;
    int status=0;
    int lim;
    p_lim :> lim;
    if((lim&3) != cont)
        status=1;
    p @ t + PulseTime<: 0;
    return status;
}



unsafe void moveStepper(streaming chanend c , out port p_step , unsigned p_val , move_data_t * unsafe data , const unsigned minPulseTime){
    //set_core_high_priority_on();
    unsigned a,b=0;
    unsigned long long t;
    unsigned shift = 32+data->acc_exp;

    if((data->v_steps==1) && (data->acc_steps==0) ){
        unsigned t_long;
        if(data->acc_exp<0){
            t_long =data->t_tot>>(shift+1);
        }else{
            t_long = (data->t_tot ,unsigned[])[MSP] >>(data->acc_exp+1);
        }
        //printullongln(data->t_tot>>32);
        //printint(t_long);
        unsigned wrap =2+(t_long>>PORT_TIMER_BITS); // 2 is a magic number
        soutct(c , READY);
        while(wrap!=0){
            p_step @ minPulseTime <: 0;
            wrap--;
        }
        sendPulse(t_long , minPulseTime , p_step , p_val);
        return;
    }
    unsigned v_wrap = (data->period ,unsigned[])[MSP];
    int s = PORT_TIMER_BITS+data->acc_exp;
    if(s>0)
        v_wrap >>=s;
    else{
        printchar('*');
        printintln(s);
        v_wrap <<=-s;
    }
    soutct(c , READY);
    sendPulse(0, minPulseTime , p_step , p_val);


    if(data->acc_exp<0){
         int steps=1;
         unsigned t_old=0;
         b=0;
         { // var guard
 #pragma unsafe arrays
             for(; steps < data->acc_steps ; steps++){
                 t= ((unsigned long long) sqrt_tbl(steps , a ,b) * data->acc_inv);
                 unsigned t_long =(t>>shift);
                 unsigned wrap = (t_long-t_old)>>PORT_TIMER_BITS;
                 while(wrap!=0){
                     p_step @ t_long + minPulseTime<: 0;
                     wrap--;
                 }
                 sendPulse(t_long , minPulseTime , p_step , p_val);
                 t_old = t_long;
             }
         } // var guard
      for(int v_steps = data->v_steps ; v_steps!=0 ; v_steps--){
             t += data->period;
             t_old = (t>>shift);
            while(v_wrap!=0){
                 p_step @ t_old + minPulseTime<: 0;
                 v_wrap--;
             }
             sendPulse(t_old , minPulseTime , p_step , p_val);

         }
         steps--;
         while(steps >=0){
             b=0;
             t= data->t_tot-((unsigned long long) sqrt_tbl(steps , a ,b) * data->acc_inv);
             unsigned t_long =(t>>shift);
             unsigned wrap = (t_long-t_old)>>PORT_TIMER_BITS;
             while(wrap!=0){
                 p_step @ t_long + minPulseTime<: 0;
                 wrap--;
             }
            sendPulse(t_long , minPulseTime , p_step , p_val);
            t_old = t_long;
            steps--;
        }
    }
    else
    { //data->acc_exp>=0
        int steps=1;
        unsigned t_old=0;
        b=0;
        { // var guard
#pragma unsafe arrays
            for(; steps < data->acc_steps ; steps++){
                t= ((unsigned long long) sqrt_tbl(steps , a ,b) * data->acc_inv);
                unsigned t_long =(t , unsigned[])[MSP]>>data->acc_exp;
                unsigned wrap = (t_long-t_old)>>PORT_TIMER_BITS;
                while(wrap!=0){
                    p_step @ t_long + minPulseTime<: 0;
                    wrap--;
                }
                sendPulse(t_long , minPulseTime , p_step , p_val);
                t_old = t_long;
            }
        } // var guard
     for(int v_steps = data->v_steps ; v_steps!=0 ; v_steps--){
            t += data->period;
            t_old = (t , unsigned[])[MSP]>>data->acc_exp;
           while(v_wrap!=0){
                p_step @ t_old + minPulseTime<: 0;
                v_wrap--;
            }
            sendPulse(t_old , minPulseTime , p_step , p_val);
        }
        steps--;
        while(steps >=0){
            b=0;
            t= data->t_tot-((unsigned long long) sqrt_tbl(steps , a ,b) * data->acc_inv);
            unsigned t_long =(t , unsigned[])[MSP]>>data->acc_exp;
            unsigned wrap = (t_long-t_old)>>PORT_TIMER_BITS;
            while(wrap!=0){
                p_step @ t_long + minPulseTime<: 0;
                wrap--;
            }
            sendPulse(t_long , minPulseTime , p_step , p_val);
           t_old = t_long;
           steps--;
       }
    }
    sync(p_step);
}

unsafe unsigned moveStepperLim(streaming chanend c , out port p_step , unsigned p_val , move_data_t * unsafe data ,
        const unsigned minPulseTime , in port p_lim , unsigned cont){
    //set_core_high_priority_on();
    unsigned a,b=0;
    unsigned long long t;
    unsigned shift = INT32_BITS+data->acc_exp;

    if((data->v_steps==1) && (data->acc_steps==0) ){
        unsigned t_long;
        if(data->acc_exp<0){
            t_long =data->t_tot>>(shift+1);
        }else{
            t_long = (data->t_tot ,unsigned[])[MSP] >>(data->acc_exp+1);
        }
        //printullongln(data->t_tot>>32);
        //printint(t_long);
        unsigned wrap =2+(t_long>>PORT_TIMER_BITS); // 2 is a magic number
        soutct(c , READY);
        while(wrap!=0){
            p_step @ minPulseTime <: 0;
            wrap--;
        }
        if(sendPulseLim(t_long , minPulseTime , p_step , p_val , p_lim , cont))
            return 1;
        return 0;
    }
    unsigned v_wrap = (data->period ,unsigned[])[MSP];
    int s = PORT_TIMER_BITS+data->acc_exp;
    if(s>0)
        v_wrap >>=s;
    else{
        printchar('*');
        printintln(s);
        v_wrap <<=-s;
    }

    soutct(c , READY);
    if(sendPulseLim(0, minPulseTime , p_step , p_val , p_lim , cont))
        return 1;

    if(data->acc_exp<0){
         int steps=1;
         unsigned t_old=0;
         b=0;
         { // var guard
 #pragma unsafe arrays
             for(; steps < data->acc_steps ; steps++){
                 t= ((unsigned long long) sqrt_tbl(steps , a ,b) * data->acc_inv);
                 unsigned t_long =(t>>shift);
                 unsigned wrap = (t_long-t_old)>>PORT_TIMER_BITS;
                 while(wrap!=0){
                     p_step @ t_long + minPulseTime<: 0;
                     wrap--;
                 }
                 if(sendPulseLim(t_long , minPulseTime , p_step , p_val , p_lim , cont))
                     return 1;
                 t_old = t_long;
             }
         } // var guard
      for(int v_steps = data->v_steps ; v_steps!=0 ; v_steps--){
             t += data->period;
             t_old = (t>>shift);
            while(v_wrap!=0){
                 p_step @ t_old + minPulseTime<: 0;
                 v_wrap--;
             }
             if(sendPulseLim(t_old , minPulseTime , p_step , p_val , p_lim , cont))
                 return 1;
         }
         steps--;
         while(steps >=0){
             b=0;
             t= data->t_tot-((unsigned long long) sqrt_tbl(steps , a ,b) * data->acc_inv);
             unsigned t_long =(t>>shift);
             unsigned wrap = (t_long-t_old)>>PORT_TIMER_BITS;
             while(wrap!=0){
                 p_step @ t_long + minPulseTime<: 0;
                 wrap--;
             }
             if(sendPulseLim(t_long , minPulseTime , p_step , p_val , p_lim , cont))
                 return 1;
            t_old = t_long;
            steps--;
        }
    }
    else
    { //data->acc_exp>=0
        int steps=1;
        unsigned t_old=0;
        b=0;
        { // var guard
#pragma unsafe arrays
            for(; steps < data->acc_steps ; steps++){
                t= ((unsigned long long) sqrt_tbl(steps , a ,b) * data->acc_inv);
                unsigned t_long =(t,unsigned[])[MSP]>>data->acc_exp;
                unsigned wrap = (t_long-t_old)>>PORT_TIMER_BITS;
                while(wrap!=0){
                    p_step @ t_long + minPulseTime<: 0;
                    wrap--;
                }
                if(sendPulseLim(t_long , minPulseTime , p_step , p_val , p_lim , cont))
                    return 1;
                t_old = t_long;
            }
        } // var guard
     for(int v_steps = data->v_steps ; v_steps!=0 ; v_steps--){
            t += data->period;
            t_old = (t,unsigned[])[MSP]>>data->acc_exp;
           while(v_wrap!=0){
                p_step @ t_old + minPulseTime<: 0;
                v_wrap--;
            }
            if(sendPulseLim(t_old , minPulseTime , p_step , p_val , p_lim , cont))
                return 1;
        }
        steps--;
        while(steps >=0){
            b=0;
            t= data->t_tot-((unsigned long long) sqrt_tbl(steps , a ,b) * data->acc_inv);
            unsigned t_long =(t,unsigned[])[MSP]>>data->acc_exp;
            unsigned wrap = (t_long-t_old)>>PORT_TIMER_BITS;
            while(wrap!=0){
                p_step @ t_long + minPulseTime<: 0;
                wrap--;
            }
            if(sendPulseLim(t_long , minPulseTime , p_step , p_val , p_lim , cont))
                return 1;
           t_old = t_long;
           steps--;
       }
    }
    sync(p_step);
    return 0;
}
