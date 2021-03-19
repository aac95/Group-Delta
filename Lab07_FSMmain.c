
#include <stdint.h>
#include "msp.h"
#include "../inc/clock.h"
#include "../inc/LaunchPad.h"
#include "../inc/Texas.h"

/*(Left,Right) Motors, call LaunchPad_Output (positive logic)
3   1,1     both motors, yellow means go straight
2   1,0     left motor,  green  means turns right
1   0,1     right motor, red    means turn left
0   0,0     both off,    dark   means stop
(Left,Right) Sensors, call LaunchPad_Input (positive logic)
3   1,1     both buttons pushed means on line,
2   1,0     SW2 pushed          means off to right
1   0,1     SW1 pushed          means off to left
0   0,0     neither button      means lost
 */

// Linked data structure
struct State {
  uint32_t out;                // 2-bit output
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[4]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

/*#define Center    &fsm[0]
#define Left      &fsm[1]
#define Right     &fsm[2]*/

#define Center  &fsm[0]
#define Left1   &fsm[1]
#define Left2   &fsm[2]
#define OffL1   &fsm[3] //5000
#define Off2    &fsm[4] //5000
#define Stop    &fsm[5]
#define OffR1   &fsm[6] //5000
#define Right1  &fsm[7]
#define Right2  &fsm[8]

// student starter code

/*State_t fsm[3]={
  {0x03, 500, { Right, Left,   Right,  Center }},  // Center
  {0x02, 500, { Left,  Center, Right,  Center }},  // Left
  {0x01, 500, { Right, Left,   Center, Center }}   // Right
};*/
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
/*Run FSM continuously
1) Output depends on State (LaunchPad LED)
2) Wait depends on State
3) Input (LaunchPad buttons)
4) Next depends on (Input,State)
 */
int main(void){ uint32_t heart=0;
  Clock_Init48MHz();
  LaunchPad_Init();
  TExaS_Init(LOGICANALYZER);  // optional
  Spt = Center;
  while(1){
    Output = Spt->out;            // set output from FSM
    LaunchPad_Output(Output);     // do output to two motors
    TExaS_Set(Input<<2|Output);   // optional, send data to logic analyzer
    Clock_Delay1ms(Spt->delay);   // wait
    Input = LaunchPad_Input();    // read sensors
    Spt = Spt->next[Input];       // next depends on input and state
    heart = heart^1;
    LaunchPad_LED(heart);         // optional, debugging heartbeat
  }
}