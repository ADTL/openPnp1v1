/*
 * calc.h
 *
 *  Created on: 11 jun 2018
 *      Author: micke
 */


#ifndef CALC_H_
#define CALC_H_


void calcAccSteps(struct move_t &s ,  double &scale);
void calcTimes(struct move_t &s , int update_period ,  double &scale);
void calcFixedPoint(struct move_t &s);
void calcAllMovement(struct moveGroup_t &s);
void calcSquares(struct move_t &s);
#endif /* CALC_H_ */
