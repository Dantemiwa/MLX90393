#ifndef __MLX90393__H
#define __MLX90393__H

#include "stdint.h"

//COMMAND_byte
#define EX 0x80
#define SB 0X10
#define SWOC 0x20
#define SM 0x30
#define RM 0x40
#define RR 0x50
#define WR 0x60
#define RT 0xF0
#define HR 0xD0
#define HS 0xE0
#define NOP 0x00

//MEMORY
/*********************************

**********************************/
#define WRITE_REG_MLX(address,val)  ((WR)|((val&0xff00) << 8) | (val & 0xff) << 16| (address << 26))//((WR)|((val&0xff00) << 8) | (val & 0xff) << 16| (address << 26))
#define READ_REG_MLX(address)        ((RR)| (address << 10))  
//val
#define  SPI_MODE_ONLY 0x0400
#define GAIN_SEL(val) ((val << 4) & 0x70) 
#define RES(z,y,x)  ((z << 9 | y << 7 | x << 5) & 0x7E0) 
#define DIG_FILT(val) ((val << 2) & 0x1C)
#define OSR(val)   (val)

#define Z_AXIS 0x008
#define Y_AXIS 0x004
#define X_AXIS 0x002
#define TEMPERATURE 0x01
#define BURST_SEL_Z 0x0200
#define BURST_SEL_Y 0x0100
#define BURST_SEL_X 0x0080
#define BURST_SEL_T 0x0040

#define TRIG_INT_SEL 0x80
#define WOC_DIFF 0x10
//address
#define ADDRESS0 0x00
#define ADDRESS1 0x01
#define ADDRESS2 0x02
#define ADDRESS3 0x03
#define ADDRESS4 0x04
#define ADDRESS5 0x05
#define ADDRESS6 0x06
#define ADDRESS7 0x07
#define ADDRESS8 0x08
#define ADDRESS9 0x09

void MLX90393_Init(void);
void MLX90393_ReadPos(void);
int32_t MLX90393_GetPosX(void);
void MLX90393_ReadStatus(void);
#endif