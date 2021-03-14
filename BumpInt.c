// BumpInt.c
// Runs on MSP432, interrupt version
// Provide low-level functions that interface bump switches on the robot.
// Daniel Valvano and Jonathan Valvano
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

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#include <stdint.h>
#include "msp.h"
// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
// Interrupt on falling edge (on touch)
void (*BumpIntTask)(uint8_t);
void BumpInt_Init(void(*task)(uint8_t)){
    // write this as part of Lab 14
    BumpIntTask = task;
    P4->SEL0 &= ~0xED; //General IO
    P4->SEL1 &= ~0xED;
    P4->DIR &= ~0xED; //Input
    P4->REN |= 0xED; //Enable pull resistors
    P4->OUT |= 0xED; //Pull-up
    P4->IES |= 0xED; //Falling edge event (0 rising, 1 falling)
    P4->IFG &= ~0xED; //Clear interrupt trigger flags
    P4->IE |= 0xED; //Enable interrupt on pins
    NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF) | 0x00400000; //Set interrupt priority of 2, NVIC_IPR9 bits 23-21
    NVIC->ISER[1] = 0x00000040; //Enable edge-interrupt (non-destructive) IRQ for PORT4_IRQHandler = 38, 38-32 = bit 6
}
// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump5
// bit 4 Bump4
// bit 3 Bump3
// bit 2 Bump2
// bit 1 Bump1
// bit 0 Bump0
uint8_t Bump_Read(void){
    // write this as part of Lab 14
    uint8_t pendingInterrupt = (P4->IV)/2 - 1; //If multiple triggers, get lowest pin with an interrupt
    uint8_t sensorReading = 0x01 << pendingInterrupt;
    uint8_t returnValue = (sensorReading >> 2) & 0x38; //Get Bump5 (P4.7), Bump4 (P4.6), and Bump3 (P4.5);
    returnValue |= (sensorReading >> 1) & 0x06; //Get Bump2 (P4.3) and Bump1 (P4.2)
    returnValue |= sensorReading & 0x01; //Get Bump0 (P4.0)
    return returnValue;
}
// we do not care about critical section/race conditions
// triggered on touch, falling edge
void PORT4_IRQHandler(void){
    // write this as part of Lab 14
    if(P4->IFG &= 0xED) { //if interrupt wasn't cleared yet
        P4->IFG &= ~0xED; //Clear interrupts trigger flags
        uint8_t sensorReading = Bump_Read();
        (*BumpIntTask)(sensorReading);
    }
}

