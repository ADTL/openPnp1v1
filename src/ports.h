/*
 * ports.h
 *
 *  Created on: 6 maj 2018
 *      Author: micke
 */
#include "defines.h"
#include "spi.h"
#include "pwm.h"
#include "stepper_server.h"
#ifndef PORTS_H_
#define PORTS_H_

on tile[I2C_TILE]: port p_SDA = XS1_PORT_1F;
on tile[I2C_TILE]: port p_SCL = XS1_PORT_1E;
on tile[SPI_TILE]: SPI_t p_SPI={XS1_PORT_1A , XS1_PORT_1B , XS1_PORT_1P , XS1_PORT_4D, XS1_CLKBLK_1};
on tile[PWM_TILE]: pwm_t pwm_r={XS1_PORT_1L , XS1_PORT_1O , XS1_PORT_4C, XS1_PORT_4A , XS1_PORT_4B, XS1_CLKBLK_2};

on tile[0]: steppers_t stepper_r={
        //{dir , step}
        {XS1_PORT_1N, XS1_PORT_1M , XS1_PORT_1G }, //X
        {XS1_PORT_4A, XS1_PORT_4C , XS1_PORT_4D }, //Y
        {XS1_PORT_1K, XS1_PORT_1L,  XS1_PORT_1H },  //Z
        {XS1_PORT_1O, XS1_PORT_1P , null},  //C0
        {XS1_PORT_1I, XS1_PORT_1J , null},  //C1
        XS1_PORT_4F,
        XS1_PORT_4E,
        XS1_CLKBLK_5
};


#endif /* PORTS_H_ */
