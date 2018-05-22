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

#define MAX_PUSLE_TIME (1<<16) //2us
#define GAIN 1
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

#define XY_STEPPING 256
#define HOME_XY_T 1000

#define XYticks_per_mm (200 *XY_STEPPING / 90)
#define Zticks_per_mm (Z_STEPPING*200 / 32)
#define Cticks_per_deg (C_STEPPING * 200/360)
#define xlim 620 //Soft lim in mm
#define ylim 440 //Soft lim
#define zlim 33.5 //Soft lim
#define Z_OFFSET_UM 3500 // DISTANCE FROM optical detection to mid

enum CMD{HOME , MOVE , SIM , HICURRENT};
enum OCaxes{X,Y};
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
    stepper_t X;
    stepper_t Y;
    stepper_t Z;
    stepper_t C0;
    stepper_t C1;
    out port Enable;
    in port error;
    clock clk;
}steppers_t;

int simStepper(unsigned total_steps  , unsigned minPulseTime , unsigned acc);
{float , char} softlim(float new_x , const float max , const float min , float &x);
void planner(client interface usb_cdc_interface cdc_data ,
             client pwm_if pwm, client spi_if spi_data[SPI_INTERFACES],
             steppers_t &p , port p_SCL , port p_SDA );

#endif /* STEPPER_SERVER_H_ */
