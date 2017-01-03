/*
 *  calc.c
 *
 *  Created by Sysadmin on 16.July.13.
 *  Copyright 2011 Ruedi Heimlicher. All rights reserved.
 *
 */
#include <avr/wdt.h>
#include "calc.h"

// $C$34*(EXP(1)^(F$50*$C53))-$C$34

//y = A*e^x - A

float expo(float x)
{
   return EXP_A*EXP(x)-EXP_A;
}