//*****************************************************************************
//
// Jacki FSM test main
// MSP432 with Jacki
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

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/TExaS.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "../inc/BumpInt.h"
#include "../inc/Reflectance.h"
#include "../inc/Motor.h"

// Linked data structure
struct State {
  uint32_t out;                // 2-bit output
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[4]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Center  &fsm[0]
#define Left1   &fsm[1]
#define Left2   &fsm[2]
#define OffL1   &fsm[3] //5000
#define Off2    &fsm[4] //5000
#define Stop    &fsm[5]
#define OffR1   &fsm[6] //5000
#define Right1  &fsm[7]
#define Right2  &fsm[8]

State_t fsm[9]={
                {0x03, 500,     {OffR1, Left1, Right1, Center}},
                {0x02, 500,     {OffL1, Left2, Right1, Center}},
                {0x03, 500,     {OffL1, Left1, Right1, Center}},
                {0x02, 5000,    {Off2, Off2, Off2, Off2}},
                {0x03, 5000,    {Stop, Left1, Right1, Center}},
                {0x00, 500,     {Stop, Left1, Right1, Center}},
                {0x01, 5000,    {Off2, Off2, Off2, Off2}},
                {0x01, 500,     {OffR1, Left1, Right2, Center}},
                {0x01, 500,     {OffR1, Left1, Right1, Center}}
};

State_t *Spt;  // pointer to the current state
uint32_t Input;
uint32_t Output;

void HandleCollision(uint8_t bumpSensor){
   Motor_Stop();
}

void main(void){
    Clock_Init48MHz();
    BumpInt_Init(&HandleCollision);
    LaunchPad_Init();

    Spt = Center;
    EnableInterrupts();
    while(1){
        Output = Spt->out;            // set output from FSM
        LaunchPad_Output(Output);     // do output to two motors
        Clock_Delay1ms(Spt->delay);   // wait
        Input = LaunchPad_Input();    // read sensors
        Spt = Spt->next[Input];       // next depends on input and state
    }

}
