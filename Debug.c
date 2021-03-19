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
