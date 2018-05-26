/*
 * stepper_server.xc
 *
 *  Created on: 7 maj 2018
 *      Author: micke
 */

#include "xs1.h"
#include <platform.h>
#include <stdio.h>
#include "safestring.h"
#include <stdlib.h>
#include <string.h>
#include "print.h"
#include "stepper_server.h"
#include "math.h"

#define STEP_VAL 3
#define SOFTLIM

unsigned char error_string[]="error:";
unsigned error_len = sizeof(error_string);
unsigned char report_string[]="report: ";
unsigned report_len = sizeof(error_string);
unsigned char ok_string[]= {'o' , 'k', NEWLINE };
unsigned ok_len= sizeof(ok_string);

const float t_kXY =(const float) (5E7/(XYticks_per_mm/60));
const float t_kZ = (const float) (5E7/(Zticks_per_mm/60));
const int HOME_XTICKS = XYticks_per_mm * (xlim+20);
const int HOME_YTICKS = XYticks_per_mm * (ylim+20);
const int HOME_ZTICKS = Zticks_per_mm * (zlim+5);

void sendAMISerror( client interface error_if error_data , char error[3] , enum spi_slave stepper){
    if((error[0] & error[1] & error[2]) == 0xFF)
        return;

    if((error[0] | error[1] | error[2]) > 0){
        if((error[0]>>2)&1)
            error_data.OpenCoil(stepper , X);
        if((error[0]>>3)&1)
            error_data.OpenCoil(stepper , Y);
        if((error[0]>>5)&1)
            error_data.ChargePumpFailure(stepper);
        if((error[0]>>6)&1)
            error_data.ThermalWarning(stepper);
        if((error[1]>>3)&1)
            error_data.OverCurrent(stepper , X , OCbottom , OCneg);
        if((error[1]>>4)&1)
            error_data.OverCurrent(stepper , X , OCtop ,    OCneg);
        if((error[1]>>5)&1)
            error_data.OverCurrent(stepper , X , OCbottom , OCpos);
         if((error[1]>>6)&1)
            error_data.OverCurrent(stepper , X , OCtop ,    OCpos);
        if((error[2]>>2)&1)
            error_data.ThermalShutdown(stepper);
        if((error[2]>>3)&1)
            error_data.OverCurrent(stepper , Y , OCbottom , OCneg);
        if((error[2]>>4)&1)
            error_data.OverCurrent(stepper , Y , OCtop ,    OCneg);
        if((error[2]>>5)&1)
            error_data.OverCurrent(stepper , Y , OCbottom , OCpos);
        if((error[2]>>6)&1)
            error_data.OverCurrent(stepper , Y , OCtop ,    OCpos);
    }
}





void supervisor(client spi_if spi, client interface error_if error_data , in port p){
    unsigned val;
    enum error_t status=STOP;
    while(1){
        select{
            case error_data.notification():
                status = error_data.getStepperStatus();
                    break;
            case p when pinsneq(val):> val:
                if(status){
                    char error[3];
                    if((val & 1) ==0){
                        error_data.XYstepperFailure();
                        wait(1e5); // Wait 1 ms
                    }
                    if( (val & 2)==0){
                        error_data.STOP();
                        wait(1e5); // Wait 1 ms
                    }
                    if( (val & 4)>0){
                        spi.readError(error , spiZ);
                        sendAMISerror(error_data ,error , stepperZ );
                    }
                    if( (val & 8)>0){
                        spi.readError(error , spiC0);
                        sendAMISerror(error_data , error , stepperC0);
                        spi.readError(error , spiC1);
                        sendAMISerror(error_data , error , stepperC1);
                    }
                    p:>val;
                }
                break;
        }
    }
}


int move_lim(out port p_step , in port p_lim, int cont, int p_val ,unsigned total_steps  , unsigned minPulseTime , unsigned acc){
    if(minPulseTime< MIN_PUSLE_TIME)
        minPulseTime=MIN_PUSLE_TIME;
    unsigned t=0 , dt , steps=0 , lim;
    unsigned half_way = total_steps;
    total_steps <<=1;
    unsigned max_speed_steps=0;
    long long v = (long long)1<<48;
    while(steps < half_way){
        p_lim :> lim;
        if((lim&3) != cont)
            return -1;
        steps++;
        p_step @ t <: p_val * (steps&1);
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
        p_lim :> lim;
        if((lim&3) != cont)
            return -1;
        steps++;
        p_step @ t <: p_val * (steps&1);
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
    p_step @ t <:0;
    return 0;
}


unsigned moveStepper(out port p_step , unsigned total_steps  , unsigned minPulseTime , unsigned acc){
    unsigned t=0 , dt , steps=0;
    unsigned half_way = total_steps;
    total_steps <<=1;
    unsigned max_speed_steps=0;
    unsigned p_val=0;
    unsigned long long v = (long long)1<<48;
    while(steps < half_way){
        p_val = ~p_val;
        p_step @ t <: p_val;
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
    while(steps < total_steps ){
        p_val = ~p_val;
        p_step @ t <: p_val;
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
    p_step @ t <:0;
    return t;
}


void stepperX_server(streaming chanend c , stepper_t &p){
    //Min pulse 2.5us = 250 cycles
    unsigned t_min , cmd;
    int steps , pos;
    int homeDir=1;
    const int p_val=1;
    const int acc_home=75<<24;
    int acc;
    while(1){
          c :> cmd;
          switch(cmd){
          case HOME:
              p.dir <: homeDir;
              move_lim(p.step , p.endstop , 0 , p_val , HOME_XTICKS ,HOME_XY_T ,  acc_home);
              p.dir <: !homeDir;
              move_lim(p.step , p.endstop , 1 , p_val, (5*XYticks_per_mm)  , 50000 , acc_home>>2);

              moveStepper(p.step , XY_STEPPING , 10000 , acc_home>>2);
              pos=0;
              soutct(c , 0);
              break;
          case MOVE:
              c:> steps;
              c:> t_min;
              c:> acc;
              if(steps<0){
                  p.dir <:1;
                  steps = -steps;
              }else
                  p.dir <:0;
              moveStepper(p.step , steps  , t_min , acc);
              soutct(c , 0);
              break;
          case SIM:
              c:> steps;
              c:> t_min;
              c:> acc;
              c <: simStepper(  steps  , t_min , acc);
              break;
          default:
              break;
         }
      }
}

void stepperY_server(streaming chanend c , stepper_t &p){
    set_core_high_priority_off();
    unsigned t_min , cmd;
    int steps , pos;
    const int p_val=3;
    const int acc_home=50<<24;
    int acc;
    const int fineAdjustRange = (4*XYticks_per_mm);
    while(1){
          c :> cmd;
          switch(cmd){
          case HOME:
              p.dir <: 2;
              move_lim(p.step , p.endstop , 0 , p_val ,  HOME_YTICKS ,HOME_XY_T  , acc_home);
              move_lim(p.step , p.endstop , 2 , 1 , fineAdjustRange , t_min , acc_home>>2);
              move_lim(p.step , p.endstop , 1 , 2 , fineAdjustRange , t_min , acc_home>>2);
              p.dir <: 1;
              move_lim(p.step , p.endstop , 3 , p_val , fineAdjustRange , t_min , acc_home>>2);
              move_lim(p.step , p.endstop , 2 , 1 , fineAdjustRange , t_min , acc_home>>2);
              move_lim(p.step , p.endstop , 1 , 2 , fineAdjustRange , t_min , acc_home>>2);

              moveStepper(p.step , XY_STEPPING , 10000 , acc_home);
              pos=0;
              soutct(c , 0);
              break;
          case MOVE:
              c:> steps;
              c:> t_min;
              c:> acc;
              if(steps<0){
                  p.dir <:2;
                  steps = -steps;
              }else
                  p.dir <:1;
              moveStepper(p.step , steps  , t_min , acc);
              soutct(c , 0);
              break;
          case SIM:
              c:> steps;
              c:> t_min;
              c:> acc;
              c <: simStepper(  steps  , t_min , acc);
              break;
          default:
              break;
         }
      }
}


void stepperZ_server(streaming chanend c , client spi_if spi , stepper_t &p){
    spi.setCurrent(CURRENT_Z>>1 , spiZ);
    spi.setMicroStepping(Z_STEPPING, spiZ);
    unsigned endstop , t_min , cmd;
    int steps;
    const int p_val=1;
    int acc;
    int hicurrent=0;
    unsigned offset_steps=0;
    unsigned offset_dir;
    while(1){
        c :> cmd;
        switch(cmd){
        case HOME:
            if(hicurrent == 0)
                spi.setCurrent(CURRENT_Z , spiZ);
            p.endstop :> endstop;
            p.dir <: !endstop;
            move_lim(p.step , p.endstop , endstop ,p_val, HOME_ZTICKS , 2500 , 75<<24);

            p.dir <: endstop;
            move_lim(p.step , p.endstop , !endstop , p_val, Zticks_per_mm , 25000 , 75<<24);

            p.dir <: offset_dir;
            moveStepper(p.step , offset_steps , 2500 , 50<<24);
            if(hicurrent == 0)
                spi.setCurrent(CURRENT_Z>>1 , spiZ);
            soutct(c , 0);
            break;
        case MOVE:
            c:> steps;
            c:> t_min;
            c:> acc;
            if(steps<0){
                p.dir <:0;
                steps = -steps;
            }else
                p.dir <:1;
            if(hicurrent == 0){
                spi.setCurrent(CURRENT_Z , spiZ);
                moveStepper(p.step , steps  , t_min , acc);
                spi.setCurrent(CURRENT_Z>>1 , spiZ);
            }else
                moveStepper(p.step , steps  , t_min , acc);
            soutct(c , 0);
            break;
        case OFFSET:
            int offset;
            c:> offset;
            if(offset<0){
                offset_dir=0;
                offset_steps = -offset;
            }else{
                offset_dir=1;
                offset_steps = offset;
            }
            break;
        case HICURRENT:
            c:> hicurrent;
            if(hicurrent)
                spi.setCurrent((CURRENT_Z*4)/3 , spiZ);
            else
                spi.setCurrent(CURRENT_Z>>1 , spiZ);

            break;
        default:
            break;
       }
    }
}

void stepperC_server(streaming chanend c, client spi_if spi , stepper_t &p_C0 , stepper_t &p_C1){
    spi.setCurrent(CURRENT_C>>1 , spiC0);
    spi.setCurrent(CURRENT_C>>1 , spiC1);
    spi.setMicroStepping(C_STEPPING, spiC0);
    spi.setMicroStepping(C_STEPPING, spiC1);

    unsigned t_min , head;
    int steps;
    while(1){
        c :> head;
        c:> steps;
        c:> t_min;
        if(head==2){
            spi.setCurrent(CURRENT_C , spiC0);
            int cont=1 , p_val=1;
            unsigned t=0;
            while(cont){
                select{
                case c:> cont:
                    break;
                default:
                    p_C0.step @ t <: p_val;
                    p_val = ! p_val;
                    t +=2000;
                    break;
                }
            }
            p_C0.step @ t <: 0;
            spi.setCurrent(CURRENT_C>>1 , spiC1);

        }else{
        if(steps<0){
            p_C0.dir <:0;
            p_C1.dir <:0;
            steps = -steps;
        }else{
            p_C0.dir <:1;
            p_C1.dir <:1;
        }
        if(head){
            spi.setCurrent(CURRENT_C , spiC0);
            moveStepper(p_C0.step , steps  , t_min , 200<<24);
            spi.setCurrent(CURRENT_C>>1 , spiC0);
        }else{
            spi.setCurrent(CURRENT_C , spiC1);
            moveStepper(p_C1.step , steps  , t_min , 200<<24);
            spi.setCurrent(CURRENT_C>>1 , spiC1);
        }
        soutct(c , 0);
        }
    }//while
}


void Move(streaming chanend c , int steps , unsigned t_min){
    c <: MOVE;
    c <: steps;
    c <: t_min;
}


int checkpos(int pos){
    if((pos>0) && (pos<255))
        return 1;
    else
        return 0;
}

int setAcc(char data[] , float &acc , float &offset){
    int pos = safestrchr(data, 'A');
    int offset_pos = safestrchr(data, 'O');
    const float scale = 1<<24;
    if(checkpos(pos>0)){
        acc = atoff(&data[pos+1])*scale; // feedrate mm/min
        if(acc > 0xFFFFFFFF)
            acc = 0xFFFFFFFF;
        else if(acc <1)
            acc = 1;
    }
    if(offset_pos>0){
        offset=atoff(&data[offset_pos+1]);
        return 1;
    }
    else
        return 0;
}



void sendError(client interface usb_cdc_interface cdc , char sign , char axes , char data[]){
    unsigned len = sprintf(data , "%s Soft limiter reached on axes %c%c\n"  , error_string ,sign , axes );
    //printf(data);
    cdc.write(data, len);
}


void stepper2string(int stepper , char str[8]){
    switch(stepper){
    case spiZ:
        safestrcpy(str , "Z" );
        return;
        break;
    case spiC0:
        safestrcpy(str , "C0");
        return;
        break;
    case spiC1:
        safestrcpy(str , "C1");
        return;
        break;
    default:
        break;
    }
    safestrcpy(str , "UNKNOWN");
}

void setEna(int val ,int &ena ,  server interface error_if error , out port p){
    ena=val;
    p <: ena;
    error.notification();
}

void g_code(client interface usb_cdc_interface cdc, client spi_if spi , client i2c_master_if i2c  , client pwm_if pwm , streaming chanend c_X , streaming chanend c_Y  , streaming chanend c_Z , streaming chanend c_C , server interface error_if error ,out port p_ena , clock clk){
float AccMaxX=10 * (1<<24);
float AccMaxY=10 * (1<<24);
float AccMaxZ=50 * (1<<24);
unsigned char data[512]={0};
unsigned length;
char stepper_moving[5]={0};
float x=0 , y=0 , z=0 , c[2]={0,0};
signed char softlim_v[3]={0};
unsigned Cstepper=0;
const char s_driver[]="in stepper driver";
const float nozzleZoneY=380; //Offset noozle 2
enum error_t error_state=OK;
int ena=0;
while(1){
        select{
                case error.STOP():
                    setEna(0 , ena , error , p_ena);
                    spi.MotorEnable(0, spiZ);
                    spi.MotorEnable(0, spiC0);
                    spi.MotorEnable(0, spiC1);
                    pwm.stop_pump();
                    pwm.actuatorsOff();
                    error_state=STOP;
                    break;
                case error.XYstepperFailure():
                    setEna(0 , ena , error , p_ena);
                    break;
                case error.getStepperStatus() -> int status:
                        status = ena;
                    break;
                case error.ChargePumpFailure(int stepper):
                    char str[8]; stepper2string(stepper , str);
                    unsigned len = sprintf(data , "%s Charge pump failure %s %s!\n"  , error_string , s_driver , str );
                    cdc.write(data , len);
                    break;
                case error.ThermalWarning(int stepper):
                    char str[8]; stepper2string(stepper , str);
                    unsigned len = sprintf(data , "%s Thermal Warning %s %s \n"  , report_string , s_driver , str );
                    cdc.write(data , len);
                   break;
                case error.ThermalShutdown(int stepper):
                    char str[8]; stepper2string(stepper , str);
                    unsigned len = sprintf(data , "%s Thermal Shutdown  %s %s\n"  , error_string, s_driver , str );
                    cdc.write(data , len);
                    break;
                case error.OverCurrent(int stepper , enum OCaxes axes , enum OCpos pos , enum OCpolarity polarity):
                    char axes_c , polarity_c , pos_c[7];
                    switch(axes){
                    case X:
                        axes_c='X';
                        break;
                    case Y:
                        axes_c='Y';
                        break;
                    default:
                        break;
                    }
                    switch(polarity){
                    case OCpos:
                        polarity_c='-';
                        break;
                    case OCneg:
                        polarity_c='+';
                        break;
                    default:
                        break;
                     }

                    switch(pos){
                    case OCtop:
                        sprintf(pos_c , "Top");
                        break;
                    case OCbottom:
                        sprintf(pos_c , "Bottom");
                        break;
                    default:
                        break;
                    }
                    char str[8]; stepper2string(stepper , str);
                    unsigned len = sprintf(data , "%s Over-current %s %s %s %c%c\n"  , error_string, s_driver , str , polarity_c, pos_c );
                    cdc.write(data , len);
                    break;

                    case error.OpenCoil(int stepper , enum OCaxes axes):
                    char str[8]; stepper2string(stepper , str);
                    char polarity_c;
                    unsigned len = sprintf(data , "%s Over-current %s %s %s\n"  , error_string, s_driver , str);
                    cdc.write(data , len);

                     break;

        case cdc.data_ready():
            length = cdc.available_bytes();
            length = cdc.read(data , length);
            unsigned gcode;
            //printstrln(data);
            char *unsafe end;
            gcode = atol(&data[1]);
            switch(data[0]){
            case 'G':  //Gcode
                switch(gcode){
                case 0: //standard Gcode move
                    float accX ,accY , accZ , dx , dy , dz , f , t_minXY ,t_minZ;
                    float new_x , new_y , new_z;
                    unsigned t_X , t_Y , t_Z;
                    int stepsX=0, stepsY=0 , stepsZ=0;
                    int xpos = safestrchr(data, 'X');
                    int ypos = safestrchr(data, 'Y');
                    int zpos = safestrchr(data, 'Z');
                    int fpos = safestrchr(data, 'F');
                    int epos = safestrchr(data, 'A'); // A or E ??
                    if(checkpos(fpos))
                        f = atof(&data[fpos+1]); // feedrate mm/min
                    else
                        f = 1000;
                    t_minXY = t_kXY/f; // ticks per second
                    t_minZ =  t_kZ/f;
                    new_x = checkpos(xpos) ? atoff(&data[xpos+1]) : x;
                    new_y = checkpos(ypos) ? atoff(&data[ypos+1]) : y;
                    new_z = checkpos(zpos) ? atoff(&data[zpos+1]) : z;
#ifdef NOOZLEAWARE
                    if(new_y > 395)
                        if( (fabsf(new_x-80.1)>0.1) || (fabsf(new_z)< 13) ){
                            unsigned len = sprintf(data , "%s Soft limiter to protect nozzle is reached \n"  , error_string);
                            //printf(data);
                            cdc.write(data, len);
                            break;
                     }
#endif
                    if(checkpos(xpos)){
                        new_x=atoff(&data[xpos+1]);
                        {dx , softlim_v[stepperX]} =  softlim(new_x , xlim , 0  , x);
                        //default
                        accX =  AccMaxX;
                        //printuint(accX);
                        t_X = roundf(t_minXY);
                        stepsX = roundf(dx*XYticks_per_mm);
                    }

                    if(checkpos(ypos)){
                        {dy , softlim_v[stepperY]} =  softlim(new_y , ylim  , 0 , y);
                         //default
                        accY =  AccMaxY;
                        t_Y = roundf(t_minXY);
                        stepsY = roundf(dy*XYticks_per_mm);
                    }

                    if(checkpos(zpos)){
                        {dz , softlim_v[stepperZ]} =  softlim(new_z , zlim , -zlim , z);
                        stepsZ = roundf(dz*Zticks_per_mm);
                        t_Z = roundf(t_minZ);
                        accZ = roundf(AccMaxZ);
                    }


                    if( ((stepsX!=0) + (stepsY!=0) + (stepsZ!=0)) >= 2){ // at least two axis moving
                        float hyp = sqrt(dx*dx + dy*dy + dz*dz);

                        if(stepsX!=0){ //avoid div with zero
                            t_X = roundf(t_minXY*hyp/fabsf(dx)); // decrease velocity -> increase time
                            if(t_X > UINT16_MAX)
                                t_X = UINT16_MAX;
                            accX =  AccMaxY*fabsf(dx/dy); // dx/H*H/dy = dx/dy
                        }

                        if(stepsY!=0){
                            t_Y = roundf(t_minXY*hyp/fabsf(dy));
                            if(t_Y > UINT16_MAX)
                                t_Y = UINT16_MAX;  // accY = dy/H*H/dy = 1

                        }

                        if(stepsZ!=0){
                            t_Z = roundf(t_minZ*hyp/fabsf(dz));
                            if(t_Z > UINT16_MAX)
                                t_Z = UINT16_MAX;
                            accZ =  AccMaxY*fabsf(dz/dy);   // dz/H*H/dy = dz/dy
                        }
                        if(accX >  AccMaxX){
                            float dec = AccMaxX/accX;
                            accX =  AccMaxX;
                            accY = accY * dec;
                            accZ = accZ * dec;
                        }
                    }

                    int aX = roundf(accX);
                    int aY = roundf(accY);
                    int aZ = roundf(accZ);

                    //send move command

                    if((stepsX!=0)){
                        c_X <: MOVE;
                        c_X <: stepsX;//+ticks_per_mm_s32;
                        c_X <: t_X;
                        c_X <: aX;
                        stepper_moving[stepperX]++;
                    }

                    if((stepsY!=0)){
                        int acc = roundf(accY);
                        //printint(steps); printchar(','); printint(t_min);
                        c_Y <: MOVE;
                        c_Y <: stepsY;//+ticks_per_mm_s32;
                        c_Y <: t_Y;
                        c_Y <: aY;
                        stepper_moving[stepperY]++;
                    }
                    if((stepsZ!=0)){
                        if((y > nozzleZoneY) && (z==0))
                            c_Z <: HOME;
                        else{
                            c_Z <: MOVE;
                            c_Z <: stepsZ;
                            c_Z <: t_Z;
                            c_Z <: aZ;
                        }
                        stepper_moving[stepperZ]++;
                    }
                    if(checkpos(epos)){
                        float deg = atof(&data[epos+1]) - c[Cstepper];
                        c[Cstepper] +=deg;
                        int steps = roundf( deg * Cticks_per_deg);
                        //unsigned t = roundf((5E7/Cticks_per_deg*60) / f); // ticks per second
                        c_C <: Cstepper; //stepper
                        c_C <: steps;
                        c_C <: 1000;
                        stepper_moving[stepperC0 + Cstepper]++;
                    }
                    start_clock(clk);
                    cdc.write(ok_string, ok_len);
                    break;
                case 4: //wait ms
                    int ppos = safestrchr(data, 'P');
                    if(checkpos(ppos)){
                        unsigned delay = atoi(&data[ppos+1]); // feedrate mm/min
                        if(delay > (UINT32_MAX/1e5))
                            delay = UINT32_MAX;
                        wait(100000*delay);
                        cdc.write(ok_string, ok_len);
                        break;
                    }
                    unsigned len= sprintf(data , "%s Unknown timedelay!" , error_string);
                    cdc.write(data ,  len);
                    break;
                case 21: //set mm;
                    cdc.write(ok_string, ok_len);
                    break;
                case 28: // home all axes
                    c_Z<: HOME;
                    start_clock(clk);
                    sinct(c_Z);
                    c_Y<: HOME;
                    wait(7e7); //Delay X home with 0.7 s
                    c_X<: HOME;
                    x=0; y=0; z=0;
                    memset(softlim_v , 0 , sizeof(softlim_v)); // reset all softlim errors
                    sinct(c_X); sinct(c_Y);
                    stop_clock(clk);
                    cdc.write(ok_string, ok_len);
                    break;
                case 90: //sSet absolute positioning mode;
                    cdc.write(ok_string, ok_len);
                    break;

                default:
                    //unknown command
                    break;
                }
            break;
            case 'M':
                switch(gcode){
                case 82:
                    cdc.write(ok_string, ok_len); // Set absolute mode for extruder
                break;
                case 400: // Wait for move to complete
                    if(stepper_moving[stepperX]>0){
                        sinct(c_X);
                        stepper_moving[stepperX]=0;
                        if(softlim_v[stepperX] !=0){
                            sendError(cdc , softlim_v[stepperX] , 'X' , data);
                            break;
                        }
                    }
                    if(stepper_moving[stepperY]>0){
                        sinct(c_Y);
                        stepper_moving[stepperY]=0;
                        if(softlim_v[stepperY] !=0){
                            sendError(cdc , softlim_v[stepperY] , 'Y', data);
                            break;
                        }
                    }
                    if(stepper_moving[stepperZ]>0){
                        sinct(c_Z);
                         stepper_moving[stepperZ]=0;
                        if(softlim_v[stepperZ] !=0){
                            sendError(cdc , softlim_v[stepperZ] , 'Z' , data);
                            break;
                        }
                    }
                    if(stepper_moving[stepperC0]>0){
                        sinct(c_C);
                        stepper_moving[stepperC0]=0;
                    }
                    if(stepper_moving[stepperC1]>0){
                        sinct(c_C);
                        stepper_moving[stepperC1]=0;
                    }
                    stop_clock(clk);
                    cdc.write(ok_string, ok_len);

                    break;
                case 800: //pump on
                    pwm.start_pump();
                    cdc.write(ok_string, ok_len);
                    break;
                case 801: //pump off
                    pwm.stop_pump();
                    pwm.actuatorsOff();
                    cdc.write(ok_string, ok_len);
                    break;
                case 802: //
                    pwm.vaccuumOn(0);
                    cdc.write(ok_string, ok_len);
                    break;
                case 803: //
                    pwm.vaccuumOff(0);
                    cdc.write(ok_string, ok_len);
                    break;
                case 804: //
                    pwm.vaccuumOn(1);
                    cdc.write(ok_string, ok_len);
                    break;
                case 805: //
                    pwm.vaccuumOff(1);
                    cdc.write(ok_string, ok_len);
                    break;
                case 806: // LED intensity
                    int ipos = safestrchr(data, 'I');
                    if(checkpos(ipos)){
                        int i = atoi(&data[ipos+1]);
                        pwm.LEDintensity( i);
                    }
                    cdc.write(ok_string, ok_len);
                    break;
                case 807: // LED i on
                    char i = data[6];
                    if( (i >= '0') && (i <= '2'))
                        pwm.LEDon(i-48);
                    cdc.write(ok_string, ok_len);
                    break;
                case 808: //LED i off
                    char i = data[6];
                    if( (i >= '0') && (i <= '2'))
                        pwm.LEDoff(i-48);
                    cdc.write(ok_string, ok_len);
                    break;
                case 810: // enable steppers
                    spi.MotorEnable(1, spiZ);
                    spi.MotorEnable(1, spiC0);
                    spi.MotorEnable(1, spiC1);
                    setEna(3 , ena , error , p_ena);
                    error_state = OK;
                    cdc.write(ok_string, ok_len);
                    break;
                case 811: // disable all
                    spi.MotorEnable(0, spiZ);
                    spi.MotorEnable(0, spiC0);
                    spi.MotorEnable(0, spiC1);
                    setEna(0 , ena , error , p_ena);
                    pwm.stop_pump();
                    pwm.actuatorsOff();
                    pwm.LEDoff(0);
                    pwm.LEDoff(1);
                    pwm.LEDoff(2);
                    error_state = STOP;
                    cdc.write(ok_string, ok_len);
                    break;
                case 812: //ACTUATOR_READ_COMMAND
                    unsigned addr;
                    if(data[6] == '1')
                        addr=ADDR1;
                    else if(data[6] == '2')
                        addr=ADDR2;
                    else{
                        unsigned len= sprintf(data , "%s Unknown command!" , error_string);
                        cdc.write(data, len);
                        break;
                    }
                    uint8_t data_i2c[2]={0};
                    uint8_t ADCpointer[1]={0};
                    size_t num_bytes_sent;
                    i2c.write(addr , ADCpointer, 1, num_bytes_sent , 0);
                    i2c.read(addr ,  data_i2c , 2, 1);
                    unsigned val = ((0xF & data_i2c[0])<<4) | (data_i2c[1]>>4 );
                    unsigned len= sprintf(data , "read:%d\n" , val);
                    //printf(data);
                    cdc.write(data, len);
                    cdc.write(ok_string, ok_len);
                    break;
                case 813: // set max accX
                    float _;
                    setAcc(data , AccMaxX , _);
                    cdc.write(ok_string, ok_len);
                    break;
                case 814: // set max accY
                    float _;
                    setAcc(data , AccMaxY , _);
                    cdc.write(ok_string, ok_len);
                    break;
                case 815: // set max accZ
                    float offset;
                    if(setAcc(data , AccMaxZ , offset)){
                     unsigned steps = roundf((offset*Zticks_per_mm));
                     c_Z <: OFFSET;
                     c_Z <: steps;
                    }
                    cdc.write(ok_string, ok_len);
                    break;
                default:
                    break;
                } // Case 'M'
                break;
                case 'T':
                    Cstepper=data[1]-48;
                    if(Cstepper>1)
                        Cstepper=1;
                    cdc.write(ok_string, ok_len);
                break;
                default:
                    break;
            }
            break;
        }
    }
}

void planner(client interface usb_cdc_interface cdc_data ,
             client pwm_if pwm,
             client spi_if spi_data[SPI_INTERFACES],
             steppers_t &p , port p_SCL , port p_SDA ){

    streaming chan c_X ,  c_Y ,c_C , c_Z;
    interface error_if error;
    i2c_master_if i2c[1];
    set_clock_ref(p.clk);
    configure_out_port_no_ready(p.X.step , p.clk , 0);
    configure_out_port_no_ready(p.Y.step , p.clk , 0);
    configure_out_port_no_ready(p.Z.step , p.clk , 0);
    configure_out_port_no_ready(p.C0.step , p.clk , 0);
    configure_out_port_no_ready(p.C1.step , p.clk , 0);

    par{

        g_code(cdc_data , spi_data[3] , i2c[0]  , pwm , c_X , c_Y ,c_Z , c_C , error , p.Enable , p.clk);
        supervisor(spi_data[0] , error , p.error);
        stepperX_server(c_X , p.X);
        stepperY_server(c_Y , p.Y);
        stepperZ_server(c_Z , spi_data[1] , p.Z);
        stepperC_server(c_C , spi_data[2] , p.C0 , p.C1);
        i2c_master(i2c , 1 , p_SCL , p_SDA , 100);

    }
}
