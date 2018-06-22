/*
 * gcode.xc
 *
 *  Created on: 3 jun 2018
 *      Author: micke
 */
#include <xclib.h>
#include <stdbool.h>
#include <safestring.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "xud_cdc.h"
#include "stepper_server.h"
#include "i2c.h"
#include "pwm.h"
#include "calc.h"

unsigned char error_string[]="error:";
unsigned error_len = sizeof(error_string);
unsigned char report_string[]="report: ";
unsigned report_len = sizeof(error_string);
unsigned char ok_string[]= {'o' , 'k', NEWLINE };
unsigned ok_len= sizeof(ok_string);

const float t_kXY = (const float) (PORT_CLK_FREQ_kHz/(XYticks_per_mm/60));
const float t_kZ = (const float) (PORT_CLK_FREQ_kHz/(Zticks_per_mm/60));



void sendError(client interface usb_cdc_interface cdc , char sign , char axes , char data[]){
    unsigned len = sprintf(data , "%s Soft limiter reached on axes %c%c\n"  , error_string ,sign , axes );
    //printf(data);
    cdc.write(data, len);
}

static inline
int checkpos(int pos){
    if((pos>0) && (pos<255))
        return 1;
    else
        return 0;
}

int setAcc(char data[] , FLOAT_T &acc , FLOAT_T &sqrt_acc, float &offset){
    int pos = safestrchr(data, 'A');
    int offset_pos = safestrchr(data, 'O');
    if(checkpos(pos>0)){
        acc = atof(&data[pos+1]); // feedrate mm/min
        if(acc <=0)
            acc = 1e-3;
        else if(acc > 1e3)
            acc = 1e3;
    }
    sqrt_acc = SQRT(acc);
    if(offset_pos>0){
        offset=atoff(&data[offset_pos+1]);
        return 1;
    }
    else
        return 0;
}

int waitForCompletion(streaming chanend c , struct move_t &m){
    for(int k=m.moving; m.moving!=0 ; m.moving--)
            schkct(c , DONE);
    if( m.softlim !=0)
            return 1;
    return 0;
}

void waitForCompletionC(streaming chanend c , struct moveC_t &m){
    for(int i=0 ; i<2 ; i++){
        if(m.moving[i]>0){
            for(int k=0; k<m.moving[i] ; k++)
                sinct(c);
            m.moving[i]=0;
        }
    }
}

void MoveC(streaming chanend c , struct moveC_t &s){
    if(s.steps!=0){
        c <: ROTATE;
        c <: s.m;
        s.moving[s.activeStepper]++;
    }
}

void Move(streaming chanend c , struct move_t &s){
    if(s.steps!=0){
        c <: s.m;
        s.moving++;
    }
}


void moveSteppers(streaming chanend c[] , struct moveGroup_t &s , clock clk){
    int HomeZ=0;
    char token[3]={READY,READY,READY};

    if(s.XYZ[Z].steps==0)
        token[Z]=DONE;
    else if((s.XYZ[Y].pos>=390) && (s.XYZ[Z].pos==0) && (s.XYZ[Z].steps>0)){
        calcHomeMovement(s , 0 , Z);
        soutct(c[Z] , HOME);
        HomeZ=1;
    }else
       soutct(c[Z] , MOVE);

    for(int i=X ; i<=Y ; i++)
        if(s.XYZ[i].steps!=0)
            soutct(c[i] , MOVE);
        else
            token[i]=DONE;

    for(int i=X ; i<=Z ; i++)
        if(s.XYZ[i].steps!=0)
            schkct(c[i] , READY);
    start_clock(clk);

    while(!((token[X]==DONE) && (token[Y]==DONE) && (token[Z]==DONE))){
        select{
        case sinct_byref(c[X] , token[X]):
                                break;
        case sinct_byref(c[Y] , token[Y]):
                                break;
        case sinct_byref(c[Z] , token[Z]):
                            if(HomeZ){
                                if(token[Z]==DONE){
                                    HomeZ=0;
                                    s.XYZ[Z].steps= ROUND2INT(s.XYZ[Z].offset*s.XYZ[Z].StepsPerMm);
                                    calcAccSteps(s.XYZ[Z] , s.scale);
                                    calcTimes(s.XYZ[Z] , 0 , s.scale);
                                    calcFixedPoint(s.XYZ[Z]);
                                    soutct(c[Z] , MOVE);
                                    token[Z]=READY;
                                }
                            }
        break;

        }
    }

    stop_clock(clk);
}


void home(streaming chanend c[4] , struct moveGroup_t &s , clock clk , enum OCaxes axes){
    switch(axes){
    case X|Y|Z:
    calcHomeMovement(s , 0 , axes);
    soutct(c[axes] , HOME);
    schkct(c[axes] , READY);
    start_clock(clk);
    while( sinct(c[axes]) != DONE);
        stop_clock(clk);
    s.XYZ[axes].pos=0;
    s.XYZ[axes].softlim=0;
    s.XYZ[axes].steps= ROUND2INT(s.XYZ[axes].offset*s.XYZ[axes].StepsPerMm);
    s.XYZ[(axes+1)%3].steps=0;
    s.XYZ[(axes+2)%3].steps=0;
    calcAllMovement(s);
    moveSteppers(c , s , clk);

    break;
    case ALL:
        for(int phase=0; phase <2 ; phase++){
            //Home Z
            calcHomeMovement(s , phase , axes);
            soutct(c[Z] , HOME);
            schkct(c[Z] , READY);
            start_clock(clk);
            while( sinct(c[Z]) != DONE);

            //HomeY
            soutct(c[Y] , HOME);
            timer tmr; int t;
            tmr :> t;
            unsigned char Ytoken=0xFF;
            int timeout=0;
            while(!((Ytoken== DONE) || timeout)){
                select{
                case tmr when timerafter(t + 7e7):> t: //Delay X axis homing for up to 0.7s ...
                        timeout=1;
                        break;
                case sinct_byref(c[Y] , Ytoken):         //if Y has not finished homing yet
                        break;
                }
            }
            soutct(c[X] , HOME);
            while( sinct(c[X]) != DONE);
            while(Ytoken!= DONE){
                sinct_byref(c[Y] , Ytoken);
            }
            stop_clock(clk);
        }
        s.feedrate = 600;
        for(int i=0; i<3 ;i++){
            s.XYZ[i].pos=0;
            s.XYZ[i].softlim=0;
            s.XYZ[i].steps= ROUND2INT(s.XYZ[i].offset*s.XYZ[i].StepsPerMm);
        }

        calcAllMovement(s);
        moveSteppers(c , s , clk);
        break;
    }
}




//struct move_t &s , float t_min , float ds , float hyp , float scale , float acc){


static inline
void setEna(int val ,int &ena ,  server interface error_if error , out port p){
    ena=val;
    p <: ena;
    error.notification();
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



void g_code(struct moveGroup_t &s , client interface usb_cdc_interface cdc, client spi_if spi , client i2c_master_if i2c  , client pwm_if pwm , streaming chanend c[4], server interface error_if error ,out port p_ena , clock clk){

unsigned char data[512]={0};
unsigned length;
const char s_driver[]="in stepper driver";
enum error_t error_state=OK;
int ena=0;

s.scale = ((double)INT32_MAX+1)/(sqrt(SQRT_TB_LEN));
init_sqrt_tbl(s.scale);

// X
s.XYZ[X].maxAcc = 750; //Relative to ticks / s^2
s.XYZ[X].time.minPeriod = PORT_CLK_FREQ_kHz / DM860_MAX_FREQ_kHz;
s.XYZ[X].StepsPerMm = XYticks_per_mm;
s.XYZ[X].softlim_min = 0;
s.XYZ[X].softlim_max = XLIM;
// Y
s.XYZ[Y].maxAcc = 250;
s.XYZ[Y].time.minPeriod = PORT_CLK_FREQ_kHz / DM860_MAX_FREQ_kHz;
s.XYZ[Y].StepsPerMm = XYticks_per_mm;
s.XYZ[Y].softlim_min = 0;
s.XYZ[Y].softlim_max = YLIM;
// Z
s.XYZ[Z].maxAcc = 500;
s.XYZ[Z].time.minPeriod = PORT_CLK_FREQ_kHz / AMIS_MAX_FREQ_kHz;
s.XYZ[Z].StepsPerMm = Zticks_per_mm;
s.XYZ[Z].softlim_min = -ZLIM;
s.XYZ[Z].softlim_max = ZLIM;

s.C.maxAcc =1;
s.C.minPeriodTime = PORT_CLK_FREQ_kHz / AMIS_MAX_FREQ_kHz;

s.C.activeStepper =0;

for(int i=X; i<=Y; i++){
    s.XYZ[i].HomeAcc = 3;
    s.XYZ[i].HomeFeedrate = 500; //[mm/min]
    s.XYZ[i].SlowHomeFeedrate = 30; //[mm/min]
    calcSquares(s.XYZ[i]);
}
s.XYZ[Z].HomeAcc = 100;
s.XYZ[Z].HomeFeedrate = 10000; //[mm/min]
s.XYZ[Z].SlowHomeFeedrate = 30; //[mm/min]
calcSquares(s.XYZ[Z]);


//#define TESTI
//For testing with DSO
#ifdef TESTI

//s.XYZ[Y].steps=15000;
//s.XYZ[Z].steps=5000;

spi.MotorEnable(1, spiZ);
spi.MotorEnable(0, spiC0);
spi.MotorEnable(0, spiC1);
setEna(3 , ena , error , p_ena);
home(c , s , clk , ALL);

s.feedrate=80000;
int dir=1;
for(int i=0; i<6 ; i++){
    s.XYZ[X].steps=50*s.XYZ[X].StepsPerMm*dir;
    s.XYZ[Y].steps=50*s.XYZ[Y].StepsPerMm*dir;
    s.XYZ[Z].steps=25*s.XYZ[Z].StepsPerMm*dir;
    dir *=-1;
    calcAllMovement(s);
    moveSteppers(c , s , clk);
}
setEna(0 , ena , error , p_ena);
spi.MotorEnable(0, spiZ);
spi.MotorEnable(0, spiC0);
spi.MotorEnable(0, spiC1);
while(1);
#endif


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
                    unsigned len = sprintf(data , "%s Over-current %s %s %c%s\n"  , error_string, s_driver , str , polarity_c, pos_c );
                    cdc.write(data , len);
                    break;

                    case error.OpenCoil(int stepper , enum OCaxes axes):
                    char str[8]; stepper2string(stepper , str);
                    char polarity_c;
                    unsigned len = sprintf(data , "%s Over-current %s %s \n"  , error_string, s_driver , str);
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
                     float t_minXY ,t_minZ;
                     const char pos_char[]="XYZ";

                    int epos = safestrchr(data, 'A'); // A or E ??
                    int fpos = safestrchr(data, 'F');
                    if(checkpos(fpos))
                        s.feedrate = atoff(&data[fpos+1]); // feedrate mm/min
                    else
                        s.feedrate = 1000;

                    for(int axis=X; axis<=Z; axis++){
                        int pos = safestrchr(data, pos_char[axis]);
                        if(checkpos(pos)){
                            float ds;
                            ds = softlim(&data[pos+1] , s.XYZ[axis]);
                            s.XYZ[axis].steps = ROUND2INT(ds*s.XYZ[axis].StepsPerMm);
                        }
                        else{
                            s.XYZ[axis].steps=0;
                        }

                    }

#ifdef NOOZLEAWARE
                    if(new_y > 395)
                        if( (fabsf(new_x-80.1)>0.1) || (fabsf(new_z)< 13) ){
                            unsigned len = sprintf(data , "%s Soft limiter to protect nozzle is reached \n"  , error_string);
                            //printf(data);
                            cdc.write(data, len);
                            break;
                     }
#endif


                    calcAllMovement(s);
                    moveSteppers(c , s , clk);

                   // printf("Acc_f=%g Ttot_f=%g\n" , s.XYZ[Z].acc , s.XYZ[Z].time.total);

                    if(checkpos(epos)){
                        float ddeg = atof(&data[epos+1]) - s.C.pos[s.C.activeStepper];
                        s.C.pos[s.C.activeStepper] += ddeg;
                        s.C.steps = ROUND( ddeg * Cticks_per_deg);
                        s.C.m.period = 1000ull<<32;
                    }
                    else
                        s.C.steps=0;

                    //send move command
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
                    home(c , s , clk , ALL);
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
                    const char str[3]="XYZ";
                    for(int i=0; i<3; i++){
                        if( waitForCompletion( c[i] , s.XYZ[i]))
                            sendError(cdc ,  s.XYZ[i].softlim , str[i] , data);
                    }
                    waitForCompletionC( c[C] , s.C);

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
                    setAcc(data , s.XYZ[X].maxAcc , s.XYZ[X].sqrt_maxAcc , s.XYZ[X].offset);
                    //printf("s.XYZ[X].maxAcc = %f ", s.XYZ[X].maxAcc );
                    cdc.write(ok_string, ok_len);
                    break;
                case 814: // set max accY
                    setAcc(data , s.XYZ[Y].maxAcc , s.XYZ[Y].sqrt_maxAcc, s.XYZ[Y].offset);
                    cdc.write(ok_string, ok_len);
                    break;
                case 815: // set max accZ
                    setAcc(data , s.XYZ[Z].maxAcc , s.XYZ[Z].sqrt_maxAcc, s.XYZ[Z].offset);
                    cdc.write(ok_string, ok_len);
                    break;
                default:
                    break;
                } // Case 'M'
                break;
                case 'T':
                    s.C.activeStepper=data[1]-48;
                    if(s.C.activeStepper>1)
                        s.C.activeStepper=1;
                    cdc.write(ok_string, ok_len);
                break;
                default:
                    break;
            }
            break;
        }
    }
}
