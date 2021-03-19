// Reflectance.c
// Provide functions to take measurements using the kit's built-in
// QTRX reflectance sensor array.  Pololu part number 3672. This works by outputting to the
// sensor, waiting, then reading the digital value of each of the
// eight phototransistors.  The more reflective the target surface is,
// the faster the voltage decays.
// Daniel and Jonathan Valvano
// July 11, 2019

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

#include <stdint.h>
#include "msp432.h"
#include "..\inc\Clock.h"
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


// ------------Reflectance_Init------------
// Initialize the GPIO pins associated with the QTR-8RC
// reflectance sensor.  Infrared illumination LEDs are
// initially off.
// Input: none
// Output: none
void Reflectance_Init(void){
    P5->SEL0 &= 0xF7; //1111 0111
    P5->SEL1 &= 0xF7;
    P5->DIR |= 0x08; //0000 1000
    P5->REN &= 0xF7;
    P5->OUT &= 0xF7;

    /*P7->SEL0 &= 0xFE; //1111 1110
    P7->SEL1 &= 0xFE;
    P7->DIR &= 0xFE;
    P7->REN &= 0xFE;*/

    P7->SEL0 &= 0x00;
    P7->SEL1 &= 0x00;
    P7->DIR &= 0x00;
    P7->REN &= 0x00;

    P4->SEL0 &= 0xFE; //1111 1110
    P4->SEL1 &= 0xFE;
    P4->DIR &= 0xFE;
    P4->REN &= 0xFE;

    P9->SEL0 &= 0xFB; //1111 1011
    P9->SEL1 &= 0xFB;
    P9->DIR |= 0x04; //0000 0100
    P9->REN &= 0xFB;
    P9->OUT &= 0xFB;
}

// ------------Reflectance_Read------------
// Read the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Read(uint32_t time){
uint8_t result;
  /*result = P7->IN;
  result = result & 0x01;*/

    P5->OUT |= 0x08;
    P9->OUT |= 0x04;
    P7->DIR |= 0xFF;
    P7->OUT |= 0xFF;
    Clock_Delay1us(10);
    P7->DIR &= 0x00;
    Clock_Delay1us(time);
    result = P7->IN;
    P5->OUT &= 0xF7; //1111 0111
    P9->OUT &= 0xFB;

  return result;
}

// ------------Reflectance_Center------------
// Read the two center sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: 0 (off road), 1 off to left, 2 off to right, 3 on road
// (Left,Right) Sensors
// 1,1          both sensors   on line
// 0,1          just right     off to left
// 1,0          left left      off to right
// 0,0          neither        lost
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Center(uint32_t time){
    // write this as part of Lab 6
  return 0; // replace this line
}


// Perform sensor integration
// Input: data is 8-bit result from line sensor
// Output: position in 0.1mm relative to center of line
int32_t Reflectance_Position(uint8_t data){
    int32_t distance;
    int W[8] = {334, 238, 142, 48, -48, -142, -238, -334};
    int b=0;
    int sum=0;
    uint8_t temp;

    temp = data & 0x01;
    if(temp != 0) //0000 0001
    {
        sum+=W[0];
        b++;
    }

    temp = data & 0x02;
    if(temp != 0) //0000 0010
    {
        sum+=W[1];
        b++;
    }

    temp = data & 0x04;
    if(temp != 0) //0000 0100
    {
        sum+=W[2];
        b++;
    }

    temp = data & 0x08;
    if(temp != 0) //0000 1000
    {
        sum+=W[3];
        b++;
    }

    temp = data & 0x10;
    if(temp != 0) //0001 0000
    {
        sum+=W[4];
        b++;
    }

    temp = data & 0x20;
    if(temp != 0) //0010 0000
    {
        sum+=W[5];
        b++;
    }

    temp = data & 0x40;
    if(temp != 0) //0100 0000
    {
        sum+=W[6];
        b++;
    }

    temp = data & 0x80;
    if(temp != 0) //1000 0000
    {
        sum+=W[7];
        b++;
    }

    distance = sum/b;
    return distance;
}


// ------------Reflectance_Start------------
// Begin the process of reading the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// Input: none
// Output: none
// Assumes: Reflectance_Init() has been called
void Reflectance_Start(void){
    // write this as part of Lab 10
    P5->OUT |= 0x08;
    P9->OUT |= 0x04;
    P7->DIR |= 0xFF;
    P7->OUT |= 0xFF;
    Clock_Delay1us(10);
    P7->DIR &= 0x00;
}


// ------------Reflectance_End------------
// Finish reading the eight sensors
// Read sensors
// Turn off the 8 IR LEDs
// Input: none
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
// Assumes: Reflectance_Start() was called 1 ms ago
uint8_t Reflectance_End(void){
    // write this as part of Lab 10
    uint8_t result;
    result = P7->IN;
    P5->OUT &= ~0x08;
    P9->OUT &= ~0x04;
    return result;
}
