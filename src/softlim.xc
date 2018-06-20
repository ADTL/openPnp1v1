/*
 * softlim.xc
 *
 *  Created on: 20 maj 2018
 *      Author: micke
 */

#include <stdlib.h>

{float , char} softlim(char* data , const float max , const float min , float &x){
    float new_x = atoff(data);
    float dx = new_x-x;
    if(x + dx > max){
        dx = max - x;
        x = max;
        return {dx , '+'};
    }
    else if(x + dx < min){
        dx = min-x;
        x = min;
        return {dx , '-'};
    }else{
        x += dx;
        return {dx , 0};
    }
}
