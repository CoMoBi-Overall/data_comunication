#include <stdio.h>
#include <iostream>
#include "MD400T.h"

// Make interger from two bytes
int BYTE2Int(uint8_t byteLow, uint8_t byteHigh)
{
    return (byteLow | (short)byteHigh<<8);
}

// Make long type data from four uint8_ts
int BYTE2LongInt(uint8_t byteData1, uint8_t byteData2, uint8_t byteData3, uint8_t byteData4)
{
    return((int)byteData1 | (int)byteData2<<8 | (int)byteData3<<16 | (int)byteData4<<24);
}