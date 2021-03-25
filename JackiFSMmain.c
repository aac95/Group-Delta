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
#include "Debug.c"


/*********************************
 *      STATE MACHINE
 *********************************/
// Linked data structure
struct State {
  char id[2];
  int16_t left;       // Left output
  int16_t right;      // Right Duty
  uint32_t delay;   // Delay
  const struct State *next[16]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Center  &fsm[0]
#define Left1   &fsm[1]
#define Left2   &fsm[2]
#define OffL1   &fsm[3]
#define Off2    &fsm[4]
#define Stop    &fsm[5]
#define OffR1   &fsm[6]
#define Right1  &fsm[7]
#define Right2  &fsm[8]
#define FarR    &fsm[9]
#define AngledR &fsm[10]
#define AngledL &fsm[11]
#define FarL    &fsm[12]

State_t fsm[13]={
                                    //0000,  0001,   0010,   0011,   0100,   0101,   0110,   0111,    1000, 1001, 1010,  1011,   1100,  1101,    1110,    1111
    {"CC", PERIOD,    PERIOD,    10,  {Off2, FarR,   Right1, Right1, Left1,  Left1,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}},  //Center
    {"L1", PERIOD/4,  PERIOD/2,  20,  {OffL1, FarR,   Right1, Right1, Left2,  Left2,  Center, AngledR, FarL, FarL, Left2, Right1, Left2, AngledL, AngledL, Center}},  //Left1
    {"L2", PERIOD,    PERIOD,    10,  {OffL1, FarR,   Right1, Right1, Left1,  Left1,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}},  //Left2
    {"OL", PERIOD,    -PERIOD,   350, {Off2,  Off2,   Off2,   Off2,   Off2,   Off2,   Off2,   Off2,    Off2, Off2, Off2,  Off2,   Off2,  Off2,    Off2,    Center}},  //OffL1
    {"OO", PERIOD,    PERIOD,    100, {Stop,  FarR,   Right1, Right1, Left1,  Left1,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}},  //Off2
    {"SS", 0,         0,         10,  {Stop,  FarR,   Right1, Right1, Left1,  Left1,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}},  //Stop
    {"OR", -PERIOD,    PERIOD,   350, {Off2,  Off2,   Off2,   Off2,   Off2,   Off2,   Off2,   Off2,    Off2, Off2, Off2,  Off2,   Off2,  Off2,    Off2,    Center}},  //OffR1
    {"R1", PERIOD/2,  PERIOD/4,  20,  {OffR1, FarR,   Right2, Right2, Left1,  Left1,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}},  //Right1
    {"R2", PERIOD,    PERIOD,    10,  {OffR1, FarR,   Right1, Right1, Left1,  Left1,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}},  //Right2
    {"FR", PERIOD,    0,         10,  {OffR1, Right2, Right1, Right1, Left1,  Left1,  Center, Center,  FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}},  //FarR
    {"AR", PERIOD,    PERIOD/3,  10,  {OffR1, FarR,   Right1, Right1, Left1,  Left1,  Center, Right2,  FarL, FarL, Left1, Right1, Left1, AngledL, AngledL, Center}},  //AngledR
    {"AL", PERIOD/3,  PERIOD,    10,  {OffL1, FarR,   Right1, Right1, Left2,  Left2,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, Left2,   Left2,   Center}},  //AngledL
    {"FL", 0,         PERIOD,    10,  {OffL1, FarR,   Right1, Right1, Left1,  Left1,  Center, AngledR, FarL, FarL, Left1, Right1, Left1, AngledL, Center,  Center}}   //FarL
};

State_t *Spt;  // pointer to the current state
uint32_t Input;
uint32_t Output;
float globalSpeed = 0.30;

uint8_t buffer[256*2]; //Debug buffer

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
#define centerMask      0b00011000

#define simpleMask1 0b10000000
#define simpleMask2 0b00110000
#define simpleMask3 0b00001100
#define simpleMask4 0b00000001

uint8_t adjustReadingTo4(uint8_t readValue) {
    /*
    uint8_t r = 0;
    if (readValue & simpleMask1) r += 0b1000;
    if (readValue & simpleMask2) r += 0b0100;
    if (readValue & simpleMask3) r += 0b0010;
    if (readValue & simpleMask4) r += 0b0001;
    return r;
    */

    uint8_t r = 0;
    if (centerMask & readValue) r += 0b0110;
    if (rightHardMask & readValue) r += 0b0001;
    if (leftHardMask & readValue) r += 0b1000;
    return r;
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
        adjusted = adjustReadingTo4(reading);

        //ROM Debug code
        buffer[bindex]= adjusted; // adds adjusted to buffer
        if(bindex<256*2)
            bindex++; // increments index
        else
        {
            bindex=0; // resets index when buffer is full
            Debug_FlashRecord((uint16_t *) buffer); // puts buffer into ROM
        }

        Spt = Spt->next[adjusted];       // next depends on input and state
    }

}
