/*
 * supervisor.xc
 *
 *  Created on: 3 jun 2018
 *      Author: micke
 */
#include "spi.h"
#include "stepper_server.h"
#include "xs1.h"

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
