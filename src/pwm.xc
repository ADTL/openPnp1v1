/*
 * pwm.xc
 *
 *  Created on: 6 maj 2018
 *      Author: micke
 */
#include <xs1.h>
#include "pwm.h"
#include "ctype.h"

const char LEDtable[]={0b1000 , 0b100 , 0b10 };
const int lookup12V[8]={0x20    ,0x1   ,0x8  , 0x4 , 0x20    ,0x10   ,0x80  , 0x40 };
const int lookup24V[8]={0x202   ,0x101  ,0x808 , 0x404, 0x2020   ,0x1010  ,0x8080 , 0x4040};
const int CYCLETIME=(1e8/25e3);

extern void wait(unsigned time);

int test(unsigned pwm){
    return ((pwm <= 100) && (pwm>0)) ? 1 : 0;
}

enum Q{StartLo , StartHi , StopLo, StopHi  };

void pwm_static(out port p_lo , out port p_hi , streaming chanend c){
    unsigned t=100 , dt=50;
    unsigned val=0;
    while(1) {
        select{
        case c:> val:
            break;
        default:
            p_lo @ t <: val;
            p_hi @ t <: val>>4;
            t +=dt;
            p_lo @ t <: val>>8;
            p_hi @ t <: val>>12;
            t +=dt;
            break;
        }
    }
}


void pwm_RTloop(out port p , streaming chanend c){
    unsigned t=100 , pwm=0 , run=0;
    while(1) {
        select{
        case c:> pwm:
            run = test(pwm);
            break;
        default:
            if(run){
                p @ t <: 1;
                p @ t+pwm <: 0;
                t +=100;
            }
            else{
                if(pwm == 0)
                    p <:0;
                else
                    p <:1;
                c:> pwm;
                run = test(pwm);
            }
            break;
        }
    }
}

#define BITFIELD_PUMP (ena &1)
const char PUMP=1;

#define MAX_PUMP_POWER 50
#define MS 1e5

#define DUMP_OFF(c , nozzle) Qstate &= ~lookup24V[nozzle];  c <: Qstate;   // 3 port valve off
#define DUMP_ON(c , nozzle)  Qstate |= lookup24V[nozzle];   c <: Qstate;  // 3 port valve on
#define VALVE_ON(c , nozzle) Qstate |=  lookup12V[nozzle+2];c <: Qstate; // 2 port valve on
#define VALVE_OFF(c , nozzle) Qstate &= ~lookup12V[nozzle+2];c <: Qstate; // 2 port valve off

[[combinable]]
void pwm_supervisor(server pwm_if pwm , streaming chanend c_pump , streaming chanend c_LED , streaming chanend c_Q , out port p_ena ){
    set_core_high_priority_off();
    unsigned ena=0;
    unsigned Qstate=0;
    p_ena<:0;
    while(1){
        select{
        case pwm.start_pump():
            if(!BITFIELD_PUMP){
                ena |=PUMP;
                p_ena <: ena;
                for(int k = 1 ; k<=MAX_PUMP_POWER ; k++){
                    c_pump <:k;
                    wait(WAIT);
                }
            }
            break;
        case pwm.stop_pump():
            if(BITFIELD_PUMP){
                for(int k = MAX_PUMP_POWER ; k>=0 ; k--){
                    c_pump <:k;
                    wait(WAIT);
                }
                ena &=~PUMP;
            }
            break;
        case pwm.vaccuumOn(int nozzle):
            DUMP_OFF( c_Q , nozzle)
            wait(50*MS);                     // deadtime
            VALVE_ON(c_Q , nozzle)
            break;
        case pwm.vaccuumOff(int nozzle):
            VALVE_OFF(c_Q  , nozzle)
            wait(50*MS);
            DUMP_ON( c_Q , nozzle)
            break;
        case pwm.actuatorsOff():
            DUMP_OFF(c_Q , 0);
            DUMP_OFF(c_Q , 1);
            VALVE_OFF(c_Q  , 0);
            VALVE_OFF(c_Q  , 1);
            break;
        case pwm.LEDintensity(unsigned DC):
              c_LED <: DC;
            break;
        case pwm.LEDon(int led):
            ena |= LEDtable[led];
            p_ena <: ena;
            break;
        case pwm.LEDoff(int led):
            ena &= ~LEDtable[led];
            p_ena <: ena;
            break;
        }
    }
}




void pwm_server(server pwm_if pwm , pwm_t &pwm_r){
    streaming chan c_pump , c_LED , c_Q;
    set_clock_ref(pwm_r.clk);
    set_clock_div(pwm_r.clk , 20);
    configure_out_port_no_ready(pwm_r.PUMP , pwm_r.clk ,0);
    configure_out_port_no_ready(pwm_r.LED , pwm_r.clk , 0);
    configure_out_port_no_ready(pwm_r.Qlo , pwm_r.clk , 0);
    configure_out_port_no_ready(pwm_r.Qhi , pwm_r.clk , 0);

    start_clock(pwm_r.clk);
    par{
        pwm_RTloop(pwm_r.PUMP , c_pump);
        pwm_RTloop(pwm_r.LED , c_LED);
        pwm_static(pwm_r.Qlo , pwm_r.Qhi , c_Q );
        [[combine]]pwm_supervisor(pwm , c_pump , c_LED , c_Q , pwm_r.ena );
    }
}



/*
void pwm_RTloopOld(out port p_pump , out port p_LED , out port p_Qlo , out port p_Qhi , clock clk , streaming chanend c_pump , streaming chanend c_LED , streaming chanend c_Q){
            configure_out_port(p_pump , clk , 0);
            configure_out_port(p_LED , clk , 0);
            configure_out_port(p_Qlo , clk , 0);
            configure_out_port(p_Qhi , clk , 0);
            start_clock(clk);
            unsigned t=100;
            unsigned t_pump=CYCLETIME/2, t_LED=CYCLETIME/2;
            //unsigned state=0;
            Q_t Q;
            struct start_t LED={0,0};
            struct start_t pump={0,0};
            struct start_t Qlo={0,0};
            struct start_t Qhi={0,0};
            // init struct
            while(1){
                select{
                default:
                    p_LED @ t <: LED.start;
                    p_pump@ t <: pump.start;
                    p_Qlo @ t <: Qlo.start;
                    p_Qhi @ t <: Qhi.start;

                    p_LED @  t+t_LED <: LED.stop;
                    p_pump@  t+t_pump <: pump.stop;
                    p_Qlo @ (t+(CYCLETIME/2)) <: Qlo.stop;
                    p_Qhi @ (t+(CYCLETIME/2)) <: Qhi.stop;

                    t+= CYCLETIME;
                    break;
                case c_pump:> t_pump:
                     c_pump:> pump;
                    break;
                case c_LED:> t_LED:
                    c_LED:>  LED;
                    break;
                case c_Q:> Qlo:
                     c_Q:> Qhi;
                    break;
                }
            }
        }
*/
