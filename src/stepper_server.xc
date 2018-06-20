/*
 * stepper_server.xc
 *
 *  Created on: 7 maj 2018
 *      Author: micke
 */

#include "xs1.h"
#include <platform.h>
#include <stdio.h>
#include "safestring.h"
#include <stdlib.h>
#include <string.h>
#include "print.h"
#include "stepper_server.h"
#include "math.h"
#include "calc.h"


const float DIV = 1+(float)UINT32_MAX;
#define XY_HOME_PERIOD 10000ull <<32

#define HOME_XTICKS  (XYticks_per_mm * (XLIM+20))
#define HOME_YTICKS  (XYticks_per_mm * (YLIM+20))
#define HOME_ZTICKS  (Zticks_per_mm * (ZLIM+5))

const int dir[3][2]={{1,0},{2,1},{0,1}};


unsafe void stepper_server(move_data_t * unsafe data, streaming chanend c , stepper_t &p ,  stepper_t &?p2 , int axis , client spi_if ?spi , const unsigned minPulseTime){

    unsigned cont;
    unsigned offset_steps=0;
    unsigned offset_dir=0;
    switch(axis){
    case Z:
        spi.setCurrent(CURRENT_Z>>1 , spiZ);
        spi.setMicroStepping(Z_STEPPING , spiZ);
        break;
    case C:
        spi.setCurrent(CURRENT_C>>1 , spiC0);
        spi.setCurrent(CURRENT_C>>1 , spiC1);
        spi.setMicroStepping(C_STEPPING, spiC0);
        spi.setMicroStepping(C_STEPPING, spiC1);
        break;
    default:
        break;
    }
    char cmd;
    while(1){
          cmd=sinct(c);
          switch(cmd){
          case HOME:
              switch(axis){
              case X:
                  //printstr("XHOME");
                  int p_val=1;
                  p.dir <: dir[X][0];
                  moveStepperLim(c , p.step , p_val , data , minPulseTime  , p.endstop , 0);
                  p.dir <: dir[X][1];
                  moveStepperLim(c , p.step , p_val , data , minPulseTime  , p.endstop , 1);
                  break;
              case Y:
                  p.dir <: dir[Y][0];
                  moveStepperLim(c , p.step , 3 , data , minPulseTime  , p.endstop , 0 );
                  moveStepperLim(c , p.step , 1 , data , minPulseTime  , p.endstop , 2 );
                  moveStepperLim(c , p.step , 2 , data , minPulseTime  , p.endstop , 1 );
                  p.dir <: dir[Y][1];
                  moveStepperLim(c , p.step , 3 , data , minPulseTime  , p.endstop , 3 );
                  moveStepperLim(c , p.step , 1 , data , minPulseTime  , p.endstop , 1 );
                  moveStepperLim(c, p.step , 2 , data , minPulseTime  , p.endstop , 2 );
                  break;
              case Z:
                 // printstrln("HOME_Z");
                  unsigned endstop, p_val=1;
                  spi.setCurrent(CURRENT_Z , spiZ);
                  p.endstop :> endstop;
                  p.dir <: !endstop;
                  if(!moveStepperLim(c , p.step , p_val , data , minPulseTime  , p.endstop , endstop)){
                      //printstrln("Z-axis did not move!");
                  }
                  p.dir <: endstop;
                  moveStepperLim(c , p.step , p_val , data , minPulseTime  , p.endstop , !endstop);
                  spi.setCurrent(CURRENT_Z>>1 , spiZ);
                  break;
              default:
                  break;
              }//switch
              soutct(c , DONE);
              break;
              case MOVE:
                  if( data->dir)
                      p.dir <:dir[axis][0];
                  else
                      p.dir <:dir[axis][1];
                  switch(axis){
                  case X:
                      moveStepper(c , p.step , 1 , data , minPulseTime);
                      break;
                  case Y:
                      moveStepper(c , p.step , 3 , data , minPulseTime);
                      break;
                  case Z:
                      spi.setCurrent(CURRENT_Z , spiZ);
                      moveStepper(c , p.step , 1 , data , minPulseTime);
                      spi.setCurrent(CURRENT_Z>>1 , spiZ);
                      break;
                  }
                  soutct(c , DONE);
                  break;
              case MOVE_LIM:
               if( data->dir)
                  p.dir <:dir[axis][0];
              else
                  p.dir <:dir[axis][1];
              switch(axis){
              case X:
#ifdef PRINT
                  printf("X: Acc_steps=%d, v_steps%d, exp=%d, period=%u , tot=%u\n" , data.acc_steps , data.v_steps , data.acc_exp ,(data.period , unsigned [])[1] , (data.t_tot , unsigned [])[1]);
                  unsigned long long test=8ull<<32;
                  test++;
                  printuint((test , unsigned [])[0]);
                  printuint((test , unsigned [])[1]);
#endif

                  char status = moveStepperLim(c , p.step , 1 , data , minPulseTime , p.endstop , 0);
                  soutct(c , status );
                  break;
              case Y:
#ifdef PRINT
                  printf("Y: Acc_steps=%d, v_steps%d, exp=%d, period=%u , tot=%u\n" , data.acc_steps , data.v_steps , data.acc_exp , (data.period , unsigned[])[1] , (data.t_tot , unsigned[])[1]);
#endif
                  char status;
                  status = moveStepperLim(c , p.step , 3 , data , minPulseTime , p.endstop , 0 );
                  soutct(c , status );
                  break;
              case Z:
#ifdef PRINT
                  printf("\nZ: acc_steps:=%d v_Steps=%d tp=%llu tot=%llu\n" , data.acc_steps , data.v_steps ,(data.period>>(32+data.acc_exp)) , (data.t_tot>>(32+data.acc_exp)));
#endif
                  spi.setCurrent(CURRENT_Z , spiZ);
                  moveStepper(c , p.step , 1 , data , minPulseTime);
                  soutct(c , 0);
                  spi.setCurrent(CURRENT_Z>>1 , spiZ);
                  break;
              default:
                  break;
              }//switch
              break;
          case ROTATE:
              int head;
              if(data->acc_steps<0){
                  p.dir <:0;
                  p2.dir <:0;
                  data->acc_steps = -data->acc_steps;
              }else{
                  p.dir <:1;
                  p2.dir <:1;
              }
              if(head){
                  spi.setCurrent(CURRENT_C , spiC0);
                  moveStepper(c , p.step , 1 , data , minPulseTime);
                  spi.setCurrent(CURRENT_C>>1 , spiC0);
              }else{
                  spi.setCurrent(CURRENT_C , spiC1);
                  moveStepper(c , p2.step , 1 , data , minPulseTime);
                  spi.setCurrent(CURRENT_C>>1 , spiC1);
              }
              soutct(c , DONE);
              break;
          case OFFSET:
              int offset;
              c:> offset;
              if(offset<0){
                  offset_dir=0;
                  offset_steps = -offset;
              }else{
                  offset_dir=1;
                  offset_steps = offset;
              }
              break;
              soutct(c , DONE);
          default:
              break;
         }
      }
}
