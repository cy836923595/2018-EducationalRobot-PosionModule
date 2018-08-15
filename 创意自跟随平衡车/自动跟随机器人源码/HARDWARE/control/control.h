#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f10x.h"
#include "Motor.h"


void Pwm_Calculate();
void Position_Calculate();
int Turn_Pwm(int encoder_left,int encoder_right,float gyro);
int myabs(int a);
#endif