/*
 * printAMISerror.xc
 *
 *  Created on: 20 maj 2018
 *      Author: micke
 */
#include "xs1.h"
#include <print.h>
#include "spi.h"
#include "stepper_server.h"

void printAMISerror(char error[3] , enum spi_slave stepper){
    const char OPEN_Coil[]="OPEN Coil ";
    const char terminal[]=" terminal. ";
    const char transistor[]=" transistor.";
    const char overcurrent[]="Overcurrent on ";
    const char Hbridge[]=" H-bridge";
    const char bottom[]="bottom ";
    const char top[]="top ";

    if((error[0] & error[1] & error[2]) == 0xFF)
        return;

    if((error[0] | error[1] | error[2]) > 0){
        printstr("ERROR at AMIS30543: ");
        printintln(stepper);
        if((error[0]>>2)&1){
            printstr(OPEN_Coil);
            printcharln('X');
        }
        if((error[0]>>3)&1){
            printstr(OPEN_Coil);
            printcharln('Y');
        }
        if((error[0]>>5)&1)
            printstrln("Charge pump failure");
        if((error[0]>>6)&1)
            printstrln("Thermal Warning");
        if((error[1]>>3)&1){
            printstr(overcurrent);
            printstr("Xneg");
            printstr(Hbridge);
            printstr(terminal);
            printstr(bottom);
            printstrln(transistor);
        }
        if((error[1]>>4)&1){
            printstr(overcurrent);
            printstr("Xneg");
            printstr(Hbridge);
            printstr(terminal);
            printstr(top);
            printstrln(transistor);
        }
        if((error[1]>>5)&1){
            printstr(overcurrent);
            printstr("Xpos");
            printstr(Hbridge);
            printstr(terminal);
            printstr(bottom);
            printstrln(transistor);
        }
        if((error[1]>>6)&1){
            printstr(overcurrent);
            printstr("Xpos");
            printstr(Hbridge);
            printstr(terminal);
            printstr(top);
            printstrln(transistor);
        }
        if((error[2]>>2)&1)
            printstrln("THERMAL SHUTDOWN");
        if((error[2]>>3)&1){
            printstr(overcurrent);
            printstr("Yneg");
            printstr(Hbridge);
            printstr(terminal);
            printstr(bottom);
            printstrln(transistor);
        }
        if((error[2]>>4)&1){
            printstr(overcurrent);
            printstr("Yneg");
            printstr(Hbridge);
            printstr(terminal);
            printstr(top);
            printstrln(transistor);
        }
        if((error[2]>>5)&1){
            printstr(overcurrent);
            printstr("Ypos");
            printstr(Hbridge);
            printstr(terminal);
            printstr(bottom);
            printstrln(transistor);
        }
        if((error[2]>>6)&1){
            printstr(overcurrent);
            printstr("Ypos");
            printstr(Hbridge);
            printstr(terminal);
            printstr(top);
            printstrln(transistor);
        }
    }
}


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
