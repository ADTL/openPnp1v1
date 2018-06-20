/*
 * openPnp1v1.xc
 *
 *  Created on: 6 maj 2018
 *      Author: micke
 */
#include <platform.h>
#include "defines.h"
#include "ports.h"
#include <usb.h>
#include "xud_cdc.h"
#include <i2c.h>
#include "spi.h"
#include "pwm.h"
#include "stepper_server.h"



int main() {
    /* Channels to communicate with USB endpoints */
    chan c_ep_out[XUD_EP_COUNT_OUT], c_ep_in[XUD_EP_COUNT_IN];
    spi_if spi_data[SPI_INTERFACES];
    pwm_if pwm;
    interface usb_cdc_interface cdc_data;
    par
    {
        on USB_TILE: xud(c_ep_out, XUD_EP_COUNT_OUT, c_ep_in, XUD_EP_COUNT_IN,
                         null, XUD_SPEED_HS, XUD_PWR_SELF);
        on USB_TILE: Endpoint0(c_ep_out[0], c_ep_in[0]);
        on USB_TILE: CdcEndpointsHandler(c_ep_in[1], c_ep_out[1], c_ep_in[2], cdc_data);
        on tile[SPI_TILE]: SPI_server(spi_data, p_SPI);
        on tile[PWM_TILE]: pwm_server(pwm , pwm_r);
        on tile[STEPPER_TILE]: stepperthreads(cdc_data , pwm , spi_data  , stepper_r , p_SCL , p_SDA);
        }
return 0;
}
