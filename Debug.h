#include "../inc/FlashProgram.h"

// globals for RAM debug
uint8_t *dbbump; 
uint8_t *dbline;

//Start of stuff I wrote

// Allocates the required space for the debug arrays
void Debug_Init(void);
// writes x value to the bump debug array and the y value to line debug array
void Debug_Dump(uint8_t x, uint8_t y);
// initalizes all flash ROM we are using to 0xFF
void Debug_FlashInit(void);
// records all values in the buffer to flash ROM
void Debug_FlashRecord(uint16_t *pt);
