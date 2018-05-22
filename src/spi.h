/*
 * spi.h
 *
 *  Created on: 22 aug 2017
 *      Author: Mikael standard
 */


#ifndef SPI_H_
#define SPI_H_

enum spi_slave{spiZ ,spiADC ,spiC1 , spiC0 };
enum stepper{stepperX , stepperY , stepperZ , stepperC0 , stepperC1};
enum EMC{VeryFast , Fast , Slow , VerySlow};
enum MODE{AMISmode , ADCmode};
enum ADDR{WR=0 , CR0=1 , CR1=2 , CR2=3 , SR0=4 , SR1=5 , SR2=6 , SR3=7,  CR3=9 , SR4=10};

#define SPI_INTERFACES 4

typedef interface spi_if {
    //void writeAMIS(unsigned addr , unsigned data , unsigned steppper);
    void setCurrent(unsigned mA, enum spi_slave stepper);
    void setMicroStepping(int steps, enum spi_slave stepper);
    void MotorEnable(int state, enum spi_slave stepper);
    void setPWMfreq(int state, enum spi_slave stepper);
    void setPWMjitter(int state, enum spi_slave stepper);
    void setEMC(int state, enum spi_slave stepper);
    void readError(char error[3] , enum spi_slave stepper);
    short readPosistion(enum spi_slave stepper);
}spi_if;




typedef struct{
    out buffered port:32 MOSI;
    in buffered port:32 MISO;
    out port CLK;
    out port CS;
    clock clkBlock;
}SPI_t;

void wait(unsigned time);
void configure_SPI(SPI_t &p_SPI);
void WriteToAmis30543(unsigned addr , unsigned data , unsigned cs , SPI_t &p_SPI);
unsigned ReadFromAMIS30543(unsigned addr, unsigned cs , SPI_t &p_SPI);
void printAMISerror(char error[3] , enum spi_slave stepper);
[[combinable]] void SPI_server(server interface spi_if spi[SPI_INTERFACES] , SPI_t &p_SPI );


#endif /* SPI_H_ */
