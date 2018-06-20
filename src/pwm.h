/*
 * pwm.h
 *
 *  Created on: 7 maj 2018
 *      Author: micke
 */

#include "xs1.h"
#include "i2c.h"

#ifndef PWM_H_
#define PWM_H_

enum Voltage{V0 , V12 , V24};

#define SOFTSTART 250 //ms
#define WAIT (SOFTSTART*1e5/2000)

typedef struct{
    char i;
    char state;
}Q_t;

typedef struct{
    out port LED;
    out port PUMP;
    out port Qlo;
    out port Qhi;
    out port ena;
    clock clk;
}pwm_t;

typedef interface pwm_if{
    void start_pump();
    void stop_pump();
    void vaccuumOn(int nozzle);
    void vaccuumOff(int nozzle);
    void actuatorsOff();
    void LEDintensity(unsigned DC);
    void LEDon(int led);
    void LEDoff(int led);
}pwm_if;

void pwm_server(server pwm_if pwm , pwm_t &pwm_r);


//void pwm_RTloop(out port p_pump , out port p_LED , out port p_Qlo , out port p_Qhi , clock clk , streaming chanend c_pump , streaming chanend c_LED , streaming chanend c_Q);
#endif /* PWM_H_ */
