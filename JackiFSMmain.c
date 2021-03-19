/*
 * Team Delta Design Project 1
 * Authors. Anthony Caamano, Haleigh Defoor, Preston Brazzle, Thomas Driscoll
 * ECE 1188, Cyber-Pysical Systems
 * University of Pittsburgh
 * March 2021
 *
 * This code implements a line-following state machine that incorporates
 *  - Bump sensor interrupt
 *  - Line sensor SysTick interrupt
 *  - PWM motor control
 *  - Data logging
 */

#include <stdint.h>
#include "msp.h"
#include "../inc/AP.h"
#include "../inc/BumpInt.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/FlashProgram.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motor.h"
#include "../inc/PWM.h"
#include "../inc/Reflectance.h"
#include "../inc/SysTickInts.h"
#include "../inc/TExaS.h"
#include "../inc/UART0.h"


/*********************************
 *      STATE MACHINE
 *********************************/
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

/*********************************
 *      BUMP SENSOR INTERRUPT
 *********************************/

void HandleCollision(uint8_t bumpSensor){
   Motor_Stop();
}

/*********************************
 *      LINE SENSOR INTERRUPT
 *********************************/

uint8_t reading;

void SysTick_Handler(void){ // every 1ms
  // write this as part of Lab 10
    uint32_t i=0;
    while(1){
        i++;
        if (i%10==0)
            Reflectance_Start();
        else if (i%10==1)
            reading = Reflectance_End();
    }
}

/*********************************
 *      MAIN FUNCTION
 *********************************/

void main(void){
    Clock_Init48MHz();
    BumpInt_Init(&HandleCollision);
    Reflectance_Init();
    SysTick_Init(48000,2);
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
