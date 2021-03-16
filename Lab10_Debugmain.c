// Lab10_Debugmain.c
// Runs on MSP432
// Student version to Debug lab
// Daniel and Jonathan Valvano
// September 4, 2017
// Interrupt interface for QTRX reflectance sensor array
// Pololu part number 3672.
// Debugging dump, and Flash black box recorder

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/
// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include "msp.h"
#include "..\inc\bump.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
// needed for Debug
#include "..\inc\FlashProgram.h"
#include <stdio.h>
#include <stdlib.h>
// globals for RAM debug
uint8_t *dbbump; 
uint8_t *dbline;

//Start of stuff I wrote

// Allocates the required space for the debug arrays
void Debug_Init(void)
{
  dbbump = malloc(sizeof(uint8_t) * 258);
  dbline = malloc(sizeof(uint8_t) * 258);
}

// writes x value to the bump debug array and the y value to line debug array
void Debug_Dump(uint8_t x, uint8_t y)
{
  // ststic index value to keep track of what in the array the next data point goes to
  static int dbindex=0;
  // assign values to array index
  dbbump[dbindex]= x;
  dbline[dbindex]= y;
  // if we arent at the end increment as normal 
  if(dbindex<258)
  {dbindex+=1;}
  // if we are at the end loop back to 0 
  else
  {dbindex=0;}
}

// initalizes all flash ROM we are using to 0xFF
void Debug_FlashInit(void)
{
  // for every address in the range we need, write F's to it
  for (uint32_t i=0; 0x00020000+i<=0x0003FFFF; i+=4)
  {Flash_Write(0x00020000+i, 0xFFFFFFFF);}
  // NOTE: this might be able to be slightly optimized via the Flash_Erase function, but this ensures everything is cleared
  // i increments by 4 since one address refers to 8 bits 
}

// records all values in the buffer to flash ROM
void Debug_FlashRecord(uint16_t *pt)
{
  // static address to keep track of where we start each call 
  static uint32_t romadr=0x00020000;
  // buffer size variable (Can be passed in or global in future implementations) 
  uint16_t buffsize = 256;
  // if we still have flash ROM space, write to ROM (Could potentally be swapped to a global flag used to skip FlashRecord calls)
  if (romadr<=0x0003FFFF)
  {Flash_WriteArray((uint32_t*)pt, romadr, buffsize/2);}
  // NOTE: Data writes to flash ROM in order of buffer index ie. Buffer[0] -> Buffer[buffsize]
  // address increments by 2 times buffer size sinve each buffer value takes up 2 addresses 
  romadr+=(2*buffsize);
}

// End of stuff I wrote

void SysTick_Handler(void){ // every 1ms
  // write this as part of Lab 10
}

/*int main(void){
  // write this as part of Lab 10
    Debug_FlashInit();
  while(1){
  // write this as part of Lab 10

  }
}
*/
int main /*Program10_1*/(void)
{
  uint8_t data=0;
  Clock_Init48MHz();
  Debug_Init();
  LaunchPad_Init();
  while(1){
    P1->OUT |= 0x01;
    Debug_Dump(data,data+1);// linear sequence
    P1->OUT &= ~0x01;
    data=data+2;
  }
}


// Driver test
#define SIZE 256  // feel free to adjust the size
uint16_t Buffer[SIZE];
int  Program10_2(void)
{
  uint16_t i;
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  for(i=0;i<SIZE;i++)
  {
    Buffer[i] = (i<<8)+(255-i); // test data
  }
  i = 0;
  while(1){
    P1->OUT |= 0x01;
    Debug_FlashInit();
    P1->OUT &= ~0x01;
    P2->OUT |= 0x01;
    Debug_FlashRecord(Buffer); // 114us
    P2->OUT &= ~0x01;
    i++;
  }
}


int Program10_3(void){ uint16_t i;
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  for(i=0;i<SIZE;i++){
    Buffer[i] = (i<<8)+(255-i); // test data
  }
  P1->OUT |= 0x01;
  Debug_FlashInit();
  P1->OUT &= ~0x01;
  i = 0;
  while(1){
    P2->OUT |= 0x01;
    Debug_FlashRecord(Buffer);
    P2->OUT &= ~0x01;
    i++;
  }
}

/*
uint8_t Buffer[1000];
uint32_t I=0;
uint8_t *pt;
void DumpI(uint8_t x){
  if(I<1000){
    Buffer[I]=x;
    I++;
  }
}
void DumpPt(uint8_t x){
  if(pt<&Buffer[1000]){
    *pt=x;
    pt++;
  }
}
void Activity(void){
  DumpI(5);
  DumpI(6);
  pt = Buffer;
  DumpPt(7);
  DumpPt(8);

}
*/
