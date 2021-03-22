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

// swap slashes for windows 
#include <stdint.h>
#include "msp.h"
#include "../inc/AP.h"
#include "../inc/BumpInt.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/FlashProgram.h"
#include "../inc/LaunchPad.h"
#include "../inc/Motors.h"
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
  uint8_t left;       // Left output
  uint8_t right;      // Right Duty
  uint32_t delay;   // Delay
  const struct State *next[16]; // Next if 2-bit input is 0-3
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
#define FarR    &fsm[9] //1000
#define AngledR &fsm[10] //3000
#define AngledL &fsm[11] //3000
#define FarL    &fsm[12] //1000

State_t fsm[13]={
                                        //0000,  0001,   0010,   0011,   0100,  0101,  0110,   0111,    1000, 1001, 1010,  1011,  1100,    1101,    1110, 1111
                {PERIOD, PERIOD, 0,      {OffR1, FarR,   Right1, Right1, Left1, Left1, Center, AngledR, FarL, FarL, Left1, Left1, AngledL, AngledL, OffR1}}, //Center
                {PERIOD, 0,      0,      {OffL1, FarR,   Right1, Right1, Left2, Left2, Center, AngledR, FarL, FarL, Left2, Left2, AngledL, AngledL, OffL1}}, //Left1
                {PERIOD, PERIOD, 0,      {OffL1, FarR,   Right1, Right1, Left1, Left1, Center, AngledR, FarL, FarL, Left1, Left1, AngledL, AngledL, OffL1}}, //Left2
                {PERIOD, 0,      5000,   {Off2,  Off2,   Off2,   Off2,   Off2,  Off2,  Off2,   Off2,    Off2, Off2, Off2,  Off2,  Off2,    Off2,    Off2}}, //OffL1
                {PERIOD, PERIOD, 5000,   {Stop,  FarR,   Right1, Right1, Left1, Left1, Center, AngledR, FarL, FarL, Left1, Left1, AngledL, AngledL, OffR1}}, //Off2
                {0,      0,      0,      {Stop,  FarR,   Right1, Right1, Left1, Left1, Center, AngledR, FarL, FarL, Left1, Left1, AngledL, AngledL, OffL1}}, //Stop
                {0,      PERIOD, 5000,   {Off2,  Off2,   Off2,   Off2,   Off2,  Off2,  Off2,   Off2,    Off2, Off2, Off2,  Off2,  Off2,    Off2,    Off2}}, //OffR1
                {0,      PERIOD, 0,      {OffR1, FarR,   Right2, Right2, Left1, Left1, Center, AngledR, FarL, FarL, Left1, Left1, AngledL, AngledL, OffR1}}, //Right1
                {PERIOD, PERIOD, 0,      {OffR1, FarR,   Right1, Right1, Left1, Left1, Center, AngledR, FarL, FarL, Left1, Left1, AngledL, AngledL, OffR1}}, //Right2
                {PERIOD, 0,      1000,   {OffR1, Right2, Right1, Right1, Left1, Left1, Center, AngledR, FarL, FarL, Left1, Left1, AngledL, AngledL, OffR1}}, //FarR
                {PERIOD, 0,      3000,   {OffR1, FarR,   Right1, Right1, Left1, Left1, Center, Right2,  FarL, FarL, Left1, Left1, AngledL, AngledL, OffR1}}, //AngledR
                {0,      PERIOD, 3000,   {OffL1, FarR,   Right1, Right1, Left2, Left2, Center, AngledR, FarL, FarL, Left1, Left1, Left2,   Left2,   OffL1}}, //AngledL
                {0,      PERIOD, 3000,   {OffL1, FarR,   Right1, Right1, Left1, Left1, Center, AngledR, FarL, FarL, Left1, Left1, AngledL, AngledL, OffL1}}
};

State_t *Spt;  // pointer to the current state
uint32_t Input;
uint32_t Output;
float globalSpeed = 0.50;

/*********************************
 *      BUMP SENSOR INTERRUPT
 *********************************/

uint8_t bumpSensorData;

void HandleCollision(uint8_t bumpSensor){
    bumpSensorData = bumpSensor;
    Motor_Stop();
}

/*********************************
 *      LINE SENSOR INTERRUPT
 *********************************/

uint8_t reading;
uint32_t i = 0;

void SysTick_Handler(void){ // every 1ms
  // write this as part of Lab 10
//    uint32_t i=0;
        i++;
        if (i%10==0)
            Reflectance_Start();
        else if (i%10==1) {
            reading = Reflectance_End();
    }
}

/*********************************
 *      8 TO 4 BIT DATA FUNCTION
 *********************************/
#define rightMask   0b00000011
#define leftMask    0b11000000
#define centerMask  0b00111100

uint8_t adjustReadingTo4(uint8_t readValue) {
    if(!readValue) return 0x0000;
    if((rightMask & readValue) && !(centerMask & readValue))    return 0x0001;
    if((rightMask & readValue) && (centerMask & readValue))     return 0x0011;
    if((leftMask & readValue) && !(centerMask & readValue))     return 0x1000;
    if((leftMask & readValue) && (centerMask & readValue))      return 0x1100;
    if(centerMask & readValue)                                  return 0x0110;
    return 0x0000;
}

/*********************************
 *      MAIN FUNCTION
 *********************************/

void main(void){
    Clock_Init48MHz();
    BumpInt_Init(&HandleCollision);
    LaunchPad_Init();
    //Reflectance_Init();
    //SysTick_Init(48000,2);
    LaunchPad_Init();
    Motor_Init(0, 0);

    Spt = Center;
    EnableInterrupts();

    while(LaunchPad_Input()==0);  // wait for touch
    while(LaunchPad_Input());     // wait for release

    Motor_Start();
    while(1){
        Motor_DutyLeft(Spt->left * globalSpeed);      //Drive Left Motor
        Motor_DutyRight(Spt->right * globalSpeed);    //Drive Right Motor
        Clock_Delay1ms(Spt->delay);     // wait
        Spt = Spt->next[adjustReadingTo4(reading)];       // next depends on input and state
    }

}
