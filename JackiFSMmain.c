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
#include "Motors.h"
#include "../inc/PWM.h"
#include "../inc/Reflectance.h"
#include "../inc/SysTickInts.h"
#include "../inc/TExaS.h"
#include "../inc/UART0.h"
//#include "Debug.c"

/*********************************
 *      STATE MACHINE
 *********************************/
// Linked data structure
struct State {
  int16_t left;       // Left output
  int16_t right;      // Right Duty
  uint32_t delay;   // Delay
  const struct State *next[16]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Center  &fsm[0]
#define Left1   &fsm[1]
#define Left2   &fsm[2]
#define OffL1   &fsm[3] //5000
#define OffL2    &fsm[4] //5000
#define Stop    &fsm[5]
#define OffR1   &fsm[6] //5000
#define OffR2   &fsm[7]
#define Right1  &fsm[8]
#define Right2  &fsm[9]
#define FarR    &fsm[10] //1000
#define AngledR &fsm[11] //3000
#define AngledL &fsm[12] //3000
#define FarL    &fsm[13] //1000

State_t fsm[14]={
                           //0000,  0001,   0010,   0011,   0100,   0101,   0110,   0111,    1000, 1001, 1010,  1011,   1100,  1101,    1110,    1111
{PERIOD,    PERIOD,    10,  {OffL2,  FarR,  AngledR, Right1, AngledL,  Left1,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}}, //Center
{PERIOD/4,  PERIOD/2,  10,  {OffL1, FarR,   AngledR, Right1, AngledL,  Left2,  Center, AngledR, FarL, FarL, Left2, Right1, Left2, AngledL, AngledL, Center}}, //Left1
{PERIOD,    PERIOD,    10,  {OffL1, FarR,   AngledR, Right1, AngledL,  Left1,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}}, //Left2
{PERIOD,    -PERIOD,   20,  {OffL2, FarR,   AngledR, Right1, AngledL,  OffL2,   Center, AngledR, FarL, OffL2, OffL2,  OffL2,   Left1, OffL2,    AngledL, Center}}, //OffL1
{PERIOD,    PERIOD,    10,  {OffL1, FarR,   AngledR, Right1, AngledL,  Left1,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}}, //OffL2
{0,         0,         10,  {Stop,  FarR,   AngledR, Right1, AngledL,  Left1,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}}, //Stop
{-PERIOD,    PERIOD,   20,  {OffR2, FarR,   AngledR, Right1, AngledL,  OffR2,   Center, AngledR, FarL, OffR2, OffR2,  OffR2,   Left1, OffR2,    AngledL, Center}}, //OffR1
{-PERIOD,    PERIOD,   10,  {OffR1, FarR,   AngledR, Right1, AngledL,  OffR2,   Center, AngledR, FarL, OffR2, OffR2,  OffR2,   Left1, OffR2,    AngledL, Center}}, //OffR2
{PERIOD/2,  PERIOD/4,  10,  {OffR1, FarR,   AngledR, Right2, AngledL,  Left1,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}}, //Right1
{PERIOD,    PERIOD,    10,  {OffR1, FarR,   AngledR, Right1, AngledL,  Left1,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}}, //Right2
{PERIOD,    0,         10,  {OffR1, Right2, AngledR, Right1, AngledL,  Left1,  Center, Center,  FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}}, //FarR
{PERIOD,    PERIOD/2,  10,  {OffR1, FarR,   AngledR, Right1, AngledL,  Left1,  Center, Right2,  FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}}, //AngledR
{PERIOD/2,  PERIOD,    10,  {OffL1, FarR,   AngledR, Right1, AngledL,  Left2,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, Left2,   Left2,   Center}}, //AngledL
{0,         PERIOD,    10,  {OffL1, FarR,   AngledR, Right1, AngledL,  Left1,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, AngledL, Center,  Center}}  //FarL
};

State_t *Spt;  // pointer to the current state
uint32_t Input;
uint32_t Output;
float globalSpeed = 0.5;

/*********************************
 *      BUMP SENSOR INTERRUPT
 *********************************/

uint8_t bumpSensorData;

void HandleCollision(uint8_t bumpSensor){
    bumpSensorData = bumpSensor;
    //Motor_Stop();
}

/*********************************
 *      LINE SENSOR INTERRUPT
 *********************************/

uint8_t reading;
uint32_t i = 0;

void SysTick_Handler(void){ // every 1ms
    if (i%10==0)
        Reflectance_Start();
    else if (i%10==1) {
        reading = Reflectance_End();
    }
    i++;
}

/*********************************
 *      8 TO 4 BIT DATA FUNCTION
 *********************************/
#define rightHardMask   0b00000011
#define rightSoftMask   0b00000100
#define leftHardMask    0b11000000
#define leftSoftMask    0b00100000
#define centerMask      0b00111100

#define simpleMask1 0b11000000
#define simpleMask2 0b00110000
#define simpleMask3 0b00001100
#define simpleMask4 0b00000011

uint8_t buffer[256*2];
uint8_t adjustReadingTo4(uint8_t readValue) {
    uint8_t r = 0;
    if (readValue & simpleMask1) r += 0b1000;
    if (readValue & simpleMask2) r += 0b0100;
    if (readValue & simpleMask3) r += 0b0010;
    if (readValue & simpleMask4) r += 0b0001;
    return r;

    /*
    uint8_t r = 0;
    if (centerMask & readValue) r += 0b0110;
    if (rightHardMask & readValue) r += 0b0001;
    if (leftHardMask & readValue) r += 0b1000;
    return r;
    */

    /*
    if(!readValue)                                                  return 0b0000;
    if(rightHardMask & readValue)                                   return 0b0001;
    if((rightSoftMask & readValue) && !(centerMask & readValue))    return 0b0001;
    if(rightSoftMask & readValue)                                   return 0b0011;
    if(leftHardMask & readValue)                                    return 0b1000;
    if((leftSoftMask & readValue) && !(centerMask & readValue))     return 0b1000;
    if(leftSoftMask & readValue)                                    return 0b1100;
    if(centerMask & readValue)                                      return 0b0110;
    return 0b0000;
    */
}

/*********************************
 *      MAIN FUNCTION
 *********************************/
uint8_t adjusted;

void main(void){
    Clock_Init48MHz();
    BumpInt_Init(&HandleCollision);
    LaunchPad_Init();
    Reflectance_Init();
    SysTick_Init(48000,2);
    LaunchPad_Init();
    Motor_Init(0, 0);

    Spt = Center;
    EnableInterrupts();

    while(LaunchPad_Input()==0);  // wait for touch
    while(LaunchPad_Input());     // wait for release
    Motor_Start();
    int bindex = 0;
    while(1){
        Motor_DutyLeft(Spt->left * globalSpeed);      //Drive Left Motor
        Motor_DutyRight(Spt->right * globalSpeed);    //Drive Right Motor
        Clock_Delay1ms(Spt->delay);     // wait
        Reflectance_Start();
        Clock_Delay1ms(1);
        reading = Reflectance_End();
        adjusted = adjustReadingTo4(reading);
        /*
        buffer[bindex]= adjusted;
        if(bindex<256*2)
            bindex++;
        else
        {
            bindex=0;
            Debug_FlashRecord((uint16_t *) buffer);
        }
        */
        Spt = Spt->next[adjusted];       // next depends on input and state
    }

}
