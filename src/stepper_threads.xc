/*
 * stepper_threads.xc
 *
 *  Created on: 3 jun 2018
 *      Author: micke
 */

#include <xs1.h>
#include "i2c.h"
#include "spi.h"
#include "pwm.h"
#include "stepper_server.h"

void stepperthreads(client interface usb_cdc_interface cdc_data ,
             client pwm_if pwm,
             client spi_if spi_data[SPI_INTERFACES],
             steppers_t &p , port p_SCL , port p_SDA ){

    streaming chan c[4];
    interface error_if error;
    i2c_master_if i2c[1];
    set_clock_xcore(p.clk);
    configure_out_port_no_ready(p.X.step , p.clk , 0);
    configure_out_port_no_ready(p.Y.step , p.clk , 0);
    configure_out_port_no_ready(p.Z.step , p.clk , 0);
    configure_out_port_no_ready(p.C0.step , p.clk , 0);
    configure_out_port_no_ready(p.C1.step , p.clk , 0);
    struct moveGroup_t s={0};
    unsafe{
        move_data_t * unsafe x_ptr = &s.XYZ[X].m;
        move_data_t * unsafe y_ptr = &s.XYZ[Y].m;
        move_data_t * unsafe z_ptr = &s.XYZ[Z].m;

    par{

        g_code(s , cdc_data , spi_data[3] , i2c[0]  , pwm , c , error , p.Enable , p.clk );
        supervisor(spi_data[0] , error , p.error);
        stepper_server(x_ptr , c[X] , p.X , null , X , null  , (PORT_CLK_FREQ_kHz / DM860_MAX_FREQ_kHz/2) );
        stepper_server(y_ptr , c[Y] , p.Y , null , Y , null  , (PORT_CLK_FREQ_kHz / DM860_MAX_FREQ_kHz/2) );
        stepper_server(z_ptr , c[Z] , p.Z , null , Z , spi_data[1] , (PORT_CLK_FREQ_kHz / AMIS_MAX_FREQ_kHz/2) );
       // stepper_server(c[C] , p.C0 , p.C1 , C ,  spi_data[2] , (PORT_CLK_FREQ_kHz / AMIS_MAX_FREQ_kHz/2));
        i2c_master(i2c , 1 , p_SCL , p_SDA , 100);

    }
    }
}
