/**
 ******************************************************************************
 * @file	  mlx90393
 * @author	Oliver
 * @version V1.0
 * @date	  2018.12.5
 * @brief	 mlx90393的读取
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
/* Includes -------------------------------------------------------------------*/
#include "mlx90393.h"
#include <stdint.h>
#include <stdlib.h>
#include "gpio.h"
#include "tim.h"
#include "spi.h"
#include "usart.h"
#include "ctrl.h"
#include "util.h"
#include "motorconf.h"
#include "can.h"
#include "stm32f4xx_hal_spi.h"
/* Extern	 variables ---------------------------------------------------------*/
extern DriverType Driver;
/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/**
 * 由于使用SPI单线模式，即数据线只有一条，
 * 进行发送和接受信息时，需要配置发送引脚的模式，使不冲突
 */
#define SPI_TX_OFF()            \
	do                            \
	{                             \
		GPIOA->MODER &= 0xFFFF3FFF; \
		GPIOA->MODER |= 0x00000000; \
	} while (0) // PA7--MOSI输入模式

#define SPI_TX_ON()             \
	do                            \
	{                             \
		GPIOA->MODER &= 0xFFFF3FFF; \
		GPIOA->MODER |= 0x00008000; \
	} while (0) // PA7--MOSI复用

#define MLX90393_CS_ENABLE() HAL_GPIO_WritePin(NSS1_GPIO_Port, NSS1_Pin, GPIO_PIN_RESET)
#define MLX90393_CS_DISABLE() HAL_GPIO_WritePin(NSS1_GPIO_Port, NSS1_Pin, GPIO_PIN_SET)
/* Private	variables ---------------------------------------------------------*/


/* Private	function pototype -------------------------------------------------*/



/**
 * @brief
 * @param  None
 * @retval
 */
static uint8_t posture[8];//Z,Y,X,T
// int16_t dataTest1 = 0x0000; //
// uint16_t dataTest = 0x0000; //
static uint8_t status  = 0;
static uint32_t write[3] ;
static uint32_t read[2] = {0};
static uint8_t write_buffer[10];


void MLX90393_ReadStatus(void)
{
//	MLX90393_CS_ENABLE();
//	status = 0;
////	write_buffer[0] = SM|X_AXIS;
//	write_buffer[0] = WR;
//	write_buffer[1] = ((0x0C|GAIN_SEL(0))&0xFF00)>>8;
//	write_buffer[2] = (0x0C|GAIN_SEL(0))&0x00FF;
//	write_buffer[3] = ADDRESS0  << 2;
//	SPI_TX_ON();
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[1],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[2],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[3],1,4);
//	SPI_TX_OFF();
//	HAL_SPI_Receive(&hspi1,&status,1,3);
//	MLX90393_CS_DISABLE();
//	
//	DMAPRINTF("%d\t",status);
//	MLX90393_CS_ENABLE();
//	write_buffer[0] = WR;
//	write_buffer[1] = (((RES(0,0,0)) | (OSR(3)) | DIG_FILT(7))&0xFF00)>>8;
//	write_buffer[2] = (((RES(0,0,0)) | (OSR(3)) | DIG_FILT(7))&0x00FF);
//	write_buffer[3] = ADDRESS2  << 2;	
//	SPI_TX_ON();
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[1],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[2],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[3],1,4);
//	SPI_TX_OFF();
//	HAL_SPI_Receive(&hspi1,&status,1,3);
//	MLX90393_CS_DISABLE();
//	
//	DMAPRINTF("%d\t",status);	
//	MLX90393_CS_ENABLE();
//	write_buffer[0] = WR;
//	write_buffer[1] = (SPI_MODE_ONLY&0xFF00)>>8;
//	write_buffer[2] = (SPI_MODE_ONLY&0x00FF);
//	write_buffer[3] = ADDRESS1  << 2;
//	SPI_TX_ON();
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[1],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[2],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[3],1,4);	
//	SPI_TX_OFF();
//	HAL_SPI_Receive(&hspi1,&status,1,4);
//	MLX90393_CS_DISABLE();
//	DMAPRINTF("%d\r\n",status);
}

void MLX90393_Init(void)
{
	MLX90393_CS_ENABLE();
	SPI_TX_ON();
	write_buffer[0] = (RT);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
	SPI_TX_OFF();
	HAL_SPI_Receive(&hspi1,(uint8_t*)&status,1,4);
	DMAPRINTF("%d\t",(uint8_t)status);
	MLX90393_CS_DISABLE();

	
//	MLX90393_CS_ENABLE();
//	write_buffer[0] = WR;
//	write_buffer[1] = ((0x0C|GAIN_SEL(0))&0xFF00)>>8;
//	write_buffer[2] = (0x0C|GAIN_SEL(0))&0x00FF;
//	write_buffer[3] = ADDRESS0  << 2;
//	SPI_TX_ON();
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[1],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[2],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[3],1,4);
//	SPI_TX_OFF();
//	
	HAL_SPI_Receive(&hspi1,&status,1,4);	
	MLX90393_CS_DISABLE();
	HAL_Delay(1);
	DMAPRINTF("%d\t",status);
	write_buffer[0] = WR;
	write_buffer[1] = ((SPI_MODE_ONLY|TRIG_INT_SEL)&0xFF00)>>8;
	write_buffer[2] = ((SPI_MODE_ONLY|TRIG_INT_SEL)&0x00FF);
	write_buffer[3] = ADDRESS1  << 2;
//	
	MLX90393_CS_ENABLE();
	SPI_TX_ON();
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[1],1,4);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[2],1,4);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[3],1,4);	
	SPI_TX_OFF();
	HAL_SPI_Receive(&hspi1,&status,1,4);
	MLX90393_CS_DISABLE();
	HAL_Delay(1);
	DMAPRINTF("%d\t",status);
//	
	write_buffer[0] = WR;
	write_buffer[1] = (((RES(0,0,0)) | (OSR(2)) | DIG_FILT(2))&0xFF00)>>8;
	write_buffer[2] = (((RES(0,0,0)) | (OSR(2)) | DIG_FILT(2))&0x00FF);
	write_buffer[3] = ADDRESS2  << 2;	
	
	
	MLX90393_CS_ENABLE();
	SPI_TX_ON();
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[1],1,4);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[2],1,4);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[3],1,4);
	SPI_TX_OFF();
	HAL_SPI_Receive(&hspi1,&status,1,4);
	DMAPRINTF("%d\t",status);
	SendBuf();
	MLX90393_CS_DISABLE();
	

	write_buffer[0] = WR;
	write_buffer[1] = ((3000)&0xFF00)>>8;
	write_buffer[2] = ((3000)&0x00FF);
	write_buffer[3] = ADDRESS7  << 2;	
	
	MLX90393_CS_ENABLE();
	SPI_TX_ON();
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[1],1,4);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[2],1,4);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[3],1,4);
	SPI_TX_OFF();
	HAL_SPI_Receive(&hspi1,&status,1,4);
	DMAPRINTF("%d\t",status);
	SendBuf();
	MLX90393_CS_DISABLE();

	write_buffer[0] = WR;
	write_buffer[1] = ((3000)&0xFF00)>>8;
	write_buffer[2] = ((3000)&0x00FF);
	write_buffer[3] = ADDRESS8  << 2;	
	
	MLX90393_CS_ENABLE();
	SPI_TX_ON();
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[1],1,4);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[2],1,4);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[3],1,4);
	SPI_TX_OFF();
	HAL_SPI_Receive(&hspi1,&status,1,4);
	DMAPRINTF("%d\t",status);
	SendBuf();
	MLX90393_CS_DISABLE();	

	write_buffer[0] = WR;
	write_buffer[1] = ((3000)&0xFF00)>>8;
	write_buffer[2] = ((3000)&0x00FF);
	write_buffer[3] = ADDRESS9  << 2;	
	
	MLX90393_CS_ENABLE();
	SPI_TX_ON();
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[1],1,4);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[2],1,4);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[3],1,4);
	SPI_TX_OFF();
	HAL_SPI_Receive(&hspi1,&status,1,4);
	DMAPRINTF("%d\t",status);
	SendBuf();
	MLX90393_CS_DISABLE();	

	MLX90393_CS_ENABLE();
	SPI_TX_ON();
	write_buffer[0] = (SB|X_AXIS|Y_AXIS);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
	SPI_TX_OFF();
	HAL_SPI_Receive(&hspi1,(uint8_t*)&status,1,4);
	DMAPRINTF("%d\r\n",(uint8_t)status);
	MLX90393_CS_DISABLE();
	SendBuf();


	MLX90393_CS_ENABLE();
	SPI_TX_ON();
	write_buffer[0] = (SWOC|X_AXIS|Y_AXIS);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
	SPI_TX_OFF();
	HAL_SPI_Receive(&hspi1,(uint8_t*)&status,1,4);
	DMAPRINTF("%d\r\n",(uint8_t)status);
	MLX90393_CS_DISABLE();
	SendBuf();
}
int a = 0;
void MLX90393_ReadPos(void)
{
 
	uint8_t statusByte;
	static int pos = 0;
//	MLX90393_CS_ENABLE();
//	SPI_TX_ON();
//	write_buffer[0] = (SM|0x06);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
//	SPI_TX_OFF();
//	HAL_SPI_Receive(&hspi1,(uint8_t*)&statusByte,1,4);
//	DMAPRINTF("%d\t",(uint8_t)statusByte);
//	MLX90393_CS_DISABLE();
//	HAL_Delay(1);
//	HAL_Delay(5);
	MLX90393_CS_ENABLE();
	SPI_TX_ON();
	write_buffer[0] = (RM|X_AXIS|Y_AXIS);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
	SPI_TX_OFF();
	HAL_SPI_Receive(&hspi1,(uint8_t*)&statusByte,1,4);
	DMAPRINTF("%d\t",(uint8_t)statusByte);
	for(int i = 0; i < 6; i++)
	{
		HAL_SPI_Receive(&hspi1,(uint8_t*)&posture[i],1,4);
	}
	MLX90393_CS_DISABLE();
	

	
//	HAL_SPI_Receive(&hspi1,(uint8_t*)&posture,8,2); 	
	DMAPRINTF("%d\t%d\t%d\r\n",(int16_t)(posture[0]*256 + posture[1]),(int16_t)(posture[2]*256 + posture[3]),(int16_t)(posture[4]*256 + posture[5]));
//	HAL_Delay(1);

//	SendBuf();
}

int32_t MLX90393_GetPosX(void)
{
	return (posture[4] << 8 | posture[3]);
}

/***COPY RIGHT: ACTION 2019*******************************************************************************/