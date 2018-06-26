/*
 * stepper_server.h
 *
 *  Created on: 7 maj 2018
 *      Author: micke
 */


#ifndef STEPPER_SERVER_H_
#define STEPPER_SERVER_H_

#include "spi.h"
#include "i2c.h"
#include "pwm.h"
#include "xud_cdc.h"
#include <math.h>

#define USE_DOUBLE
//#define DEBUG_TIME

#ifdef USE_DOUBLE
    #define FLOAT_T double
    #define SQRT sqrt
    #define FLOOR floor
    #define ROUND round
    #define ROUND2INT lround
    #define FREXP frexp
    #define NAN nan
    #define LDEXP ldexp
#else
    #define FLOAT_T float
    #define SQRT sqrtf
    #define FLOOR floorf
    #define ROUND roundf
    #define ROUND2INT lroundf
    #define FREXP frexpf
    #define NAN nanf
#define LDEXP ldexpf
#endif

#define SQRT_TB_LEN 32768
#define PORT_TIMER_BITS 16
#define INT32_BITS 32
#define MAX_PUSLE_TIME 0xFFFF

#define MICROSTEPPING 32

#define CURRENT_Z 870
#define CURRENT_C 485

#define MIN_PUSLE_TIME 200

#define NEWLINE 10

#define ADDR1 0b1010001
#define ADDR2 0b1010010
#define STOPBIT 1

#define Z_STEPPING 64
#define C_STEPPING 128

#define XY_STEPPING 64
#define HOME_XY_T 10000


#define PORT_CLK_FREQ_kHz 5E5
#define DM860_MAX_FREQ_kHz 200
#define AMIS_MAX_FREQ_kHz 250

#define XYticks_per_mm ((200 *XY_STEPPING) / 90)
#define Zticks_per_mm ((Z_STEPPING*200)/ 32)
#define Cticks_per_deg ((C_STEPPING * 200)/360)
#define secPerMin 60
#define XLIM 620 //Soft lim in mm
#define YLIM 440 //Soft lim
#define ZLIM 33.5 //Soft lim
#define Z_OFFSET_UM 3500 // DISTANCE FROM optical detection to mid

enum CMD{HOME , MOVE , MOVE_LIM , ROTATE};
enum {READY , DONE };
enum OCaxes{X,Y,Z,C,ALL};
enum OCpos{OCtop , OCbottom};
enum OCpolarity{OCpos , OCneg};
enum error_t{OK , STOP};



interface error_if {
   void STOP();
   void XYstepperFailure();
   void ChargePumpFailure(int stepper);
   void ThermalWarning(int stepper);
   void OverCurrent(int stepper , enum OCaxes axes , enum OCpos pos , enum OCpolarity polarity);
   void ThermalShutdown(int stepper);
   void OpenCoil(int stepper , enum OCaxes axes);
   [[notification]] slave void notification();
   [[clears_notification]] int getStepperStatus();
};

typedef struct{
    out port step;
    out port dir;
    in port ?endstop;
}stepper_t;


typedef struct{
    unsigned long long period;
    unsigned long long t_tot;
    int acc_steps;
    unsigned v_steps;
    unsigned acc_inv;
    int acc_exp;
    unsigned v_wrap;
    unsigned dir;
    unsigned axis;
//  unsigned _padding;
#ifdef DEBUG_TIME
    unsigned long long t1;
    unsigned long long t2;
    unsigned long long t3;
#endif
}move_data_t;

struct tone_t{
    float t;
    char len;
    char dir;
    char axis;
};


struct keys_t{
    float C;
    float Db;
    float D;
    float Eb;
    float E;
    float F;
    float Gb;
    float G;
    float Ab;
    float A;
    float Bb;
    float B;
};

typedef struct{
    FLOAT_T period;
    FLOAT_T minPeriod;
    FLOAT_T acceleration;
    FLOAT_T velocity;
    FLOAT_T total;
}timeTypes_t;

struct move_t{
    int steps;
    float pos;
    move_data_t m;
    timeTypes_t time;
    FLOAT_T acc;
    FLOAT_T sqrt_acc;
    FLOAT_T maxAcc;
    FLOAT_T sqrt_maxAcc;
    FLOAT_T HomeAcc;
    FLOAT_T sqrt_HomeAcc;
    FLOAT_T StepsPerMm;
    float HomeFeedrate;
    float SlowHomeFeedrate;
    float softlim_min;
    float softlim_max;
    int moving;
    float offset; // offset after home in mm
    int softlim;
    char softlim_char;
};

struct moveC_t{
    int steps;
    float pos[2];
    move_data_t m;
    unsigned minPeriodTime;
    FLOAT_T acc;
    FLOAT_T acc_sqrt;
    FLOAT_T maxAcc;
    FLOAT_T maxAcc_sqrt;
    unsigned time;
    short moving[2];
    unsigned activeStepper;

};


struct moveGroup_t{
    struct move_t XYZ[3];
    struct moveC_t C;
    double scale;
    float feedrate;
};


typedef struct{
    stepper_t X;
    stepper_t Y;
    stepper_t Z;
    stepper_t C0;
    stepper_t C1;
    out port Enable;
    in port error;
    clock clk;
}steppers_t;


unsafe void stepper_server(move_data_t * unsafe data , streaming chanend c , stepper_t &p ,  stepper_t &?p2 , int axis , client spi_if ?spi , const unsigned minPulseTime);

unsafe void moveStepper(streaming chanend c , out port p_step , unsigned p_val , move_data_t * unsafe data , const unsigned minPulseTime);

unsafe unsigned moveStepperLim(streaming chanend c , out port p_step , unsigned p_val , move_data_t * unsafe data ,
        const unsigned minPulseTime , in port p_lim , unsigned cont);


void sendAMISerror( client interface error_if error_data , char error[3] , enum spi_slave stepper);
void supervisor(client spi_if spi, client interface error_if error_data , in port p);

int simStepper(unsigned total_steps  , unsigned minPulseTime , unsigned acc);
void init_sqrt_tbl(double &scale);
float softlim(char *data , struct move_t &s);

void stepperthreads(client interface usb_cdc_interface cdc_data ,
             client pwm_if pwm, client spi_if spi_data[SPI_INTERFACES],
             steppers_t &p , port p_SCL , port p_SDA );

void g_code(struct moveGroup_t &s , client interface usb_cdc_interface cdc, client spi_if spi , client i2c_master_if i2c  , client pwm_if pwm , streaming chanend c[4], server interface error_if error ,out port p_ena , clock clk);


#endif /* STEPPER_SERVER_H_ */
