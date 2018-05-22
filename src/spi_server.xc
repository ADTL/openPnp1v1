/*
 * spi_server.xc
 *
 *  Created on: 22 aug 2017
 *      Author: Mikael standard
 */

#include "xs1.h"
#include <xclib.h>
#include "spi.h"
#include <print.h>



void configure_SPI(SPI_t &p_SPI){
    p_SPI.CS <:0xF;
    configure_clock_ref(p_SPI.clkBlock, 50); //1 MHz
    configure_port_clock_output( p_SPI.CLK , p_SPI.clkBlock);
    configure_out_port(p_SPI.MOSI , p_SPI.clkBlock , 1);
    configure_in_port(p_SPI.MISO , p_SPI.clkBlock);
    set_port_sample_delay(p_SPI.MISO);
}


const char ss[4]={0b1110 , 0b1101 , 0b1011 , 0b0111};

void WriteToAmis30543(unsigned addr , unsigned data , unsigned cs , SPI_t &p_SPI){

    p_SPI.CS  <:ss[cs];
    configure_out_port(p_SPI.MOSI , p_SPI.clkBlock , 1); //Write
    //unsigned mosi = byterev(bitrev(addr<<1 || data<<9));
    //unsigned mosi_data = byterev(bitrev(data));
    unsigned mosi = bitrev(((addr<<8) | data)<<17) ;
    start_clock(p_SPI.clkBlock);
    partout(p_SPI.MOSI , 15 , mosi);
    //partout(p_SPI.MOSI , 8 , mosi_data);
    sync(p_SPI.MOSI);
    stop_clock(p_SPI.clkBlock);
    p_SPI.CS <:0xF;
    wait(250);
}

unsigned ReadFromAMIS30543(unsigned addr , unsigned cs , SPI_t &p_SPI){
    p_SPI.CS  <:ss[cs];
    configure_out_port(p_SPI.MOSI , p_SPI.clkBlock , 0); //Read
    clearbuf(p_SPI.MISO);
    unsigned mosi_addr = byterev(bitrev(addr<<1));
    unsigned miso;
    start_clock(p_SPI.clkBlock);
    partout(p_SPI.MOSI , 15 , mosi_addr);
    miso = partin(p_SPI.MISO , 15);
    //sync(p_SPI.MOSI);
    stop_clock(p_SPI.clkBlock);
    miso = bitrev(miso);
    p_SPI.CS <:0xF;
    wait(250);
    return (miso>>16);
}

const unsigned short cur[]={
132, 245 , 355 , 395 , 445 ,485 , 540 , 585 , 640 , 715 ,
780 , 870 , 955 , 1060, 1150 , 1260 , 1405 , 1520 ,
1695 , 1850 , 2070 , 2240 , 2440 , 2700 , 2845 , 3000};


[[combinable]] void SPI_server(server interface spi_if spi[SPI_INTERFACES], SPI_t &p_SPI ){
    configure_SPI(p_SPI);
    while(1){
        select{
         case spi[int i].setCurrent(unsigned mA , enum spi_slave stepper):
                int k=0;
                while( cur[k] < mA )
                    k++;
                //printint(cur[k]);
                //printstrln(" mA");
                int data = ReadFromAMIS30543(CR0, stepper , p_SPI);
                data &= 0xE0;   //reset current field
                data |= k;      //set current field
                WriteToAmis30543(CR0 , data , stepper, p_SPI);
            break;
         case spi[int i].MotorEnable(int state ,  enum spi_slave stepper):
                 unsigned SLAT =  0x10; //is transparent
                 unsigned SLAG =  0x20; //Gain settings is 0.25
                 unsigned MOTEN = 0x80; //Motor enable
                 if(state)
                     WriteToAmis30543( CR2 , MOTEN | SLAG , stepper, p_SPI);
                 else
                     WriteToAmis30543( CR2 , 0 , stepper, p_SPI);
                 break;
         case spi[int i].setMicroStepping(int steps,  enum spi_slave stepper):
                       int ESM=0;
                       int SM=0;
                       int data = ReadFromAMIS30543(CR0, stepper , p_SPI);
                       data &= 0x1F;   //reset SM field
                       switch(steps){
                       case 1:
                           data |=0b11100000;
                           break;
                       case 2:
                           data |=0b10000000;
                           break;
                       case 4:
                           data |=0b01100000;
                           break;
                       case 8:
                           data |=0b01000000;
                           break;
                       case 16:
                           data |=0b00100000;
                           break;
                       case 64:
                           ESM=0b010;
                           break;
                       case 128:
                           ESM=0b001;
                           break;
                       default: //32 or unvalid value
                           return;
                       }
                       WriteToAmis30543(CR0 , data , stepper, p_SPI);
                       WriteToAmis30543(CR3 , ESM  , stepper, p_SPI);
                       break;
           case spi[int i].setPWMfreq(int state,  enum spi_slave stepper):
                   int data = ReadFromAMIS30543(CR1 , stepper , p_SPI);
                   unsigned mask = 0b00001000;
                   data &= ~mask; //reset PWMF
                   if(state)
                       data |= mask; // set motor ena
                   WriteToAmis30543( CR1 , data , stepper, p_SPI);
                   break;
           case spi[int i].setPWMjitter(int state,  enum spi_slave stepper):
                   int data = ReadFromAMIS30543(CR1 , stepper , p_SPI);
                   unsigned mask = 0b00001000;
                   data &= ~mask; //reset PWMF
                   if(state)
                       data |= mask; // set motor ena
                   WriteToAmis30543( CR1 , data , stepper, p_SPI);
                   break;
           case spi[int i].setEMC(int state,  enum spi_slave stepper):
                   int data = ReadFromAMIS30543(CR1 , stepper , p_SPI);
                   unsigned mask = 0b11;
                   data &= ~mask; //reset EMC
                   data |= state & mask;
                   WriteToAmis30543( CR1 , data , stepper, p_SPI);
                   break;
           case spi[int i].readError(char error[3] ,  enum spi_slave stepper):
                for(int k=0; k<3 ; k++)
                   error[k] = (char) ReadFromAMIS30543(k+4 , stepper , p_SPI);


               break;
           case spi[int i].readPosistion( enum spi_slave stepper) -> short msp:
                   msp = ReadFromAMIS30543(SR4 , stepper , p_SPI);
                   msp |=(ReadFromAMIS30543(SR3 , stepper , p_SPI)<<2);
                   break;

        }
    }
}
