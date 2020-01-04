/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
uint8_t rx_rollover_u8[6];
float heading;
float declinationAngle;
float headingDegrees_XY;
float headingDegrees_XY_old;
float headingDegrees_XZ;
float headingDegrees_YZ;
uint8_t rx_06H_register = 0;
uint8_t tx_0AH_register = 0x40;
uint8_t tx_0BH_register = 0x01;
uint8_t tx_09H_register = 0x19;

uint32_t tickstart = 0U;
static uint8_t get_first_time_flag = 0U;

typedef struct
{
	int16_t X_axis;
	int16_t Y_axis;
	int16_t Z_axis;
} QMC5883L_axis_type ;

QMC5883L_axis_type QMC5883L;

#define max_angle	5
#define	wait		100U /*100ms*/

uint16_t angle_diff_u16;
uint8_t direction_diff_u8;

void angle_diff_report(void);
void car_moving_regulate(void);

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void QMC5883L_Init(void)
{
	/*Set period: required to write 0x01 to 0BH register*/
	HAL_I2C_Mem_Write(&hi2c1, QMC5883L_ADDRESS, 0x000B, (uint16_t)1U, &tx_0BH_register, (uint16_t)1U, (uint32_t)100);

	/*configure I2C*/
	HAL_I2C_Mem_Write(&hi2c1, QMC5883L_ADDRESS, 0x0009, (uint16_t)1U, &tx_09H_register, (uint16_t)1U, (uint32_t)100);

	/*read sensor status*/
	HAL_I2C_Master_Receive(&hi2c1, QMC5883L_ADDRESS, &rx_06H_register, (uint16_t)1U, (uint32_t)100);

	/*enable pointer roll-over function*/
	HAL_I2C_Mem_Write(&hi2c1, QMC5883L_ADDRESS, 0x000A, (uint16_t)1U, &tx_0AH_register, (uint16_t)1U, (uint32_t)100);
}

void read_QMC5883L(void)
{
	if(get_first_time_flag == 0U)
	{
		tickstart = HAL_GetTick();
	}

	get_first_time_flag = 1U;

	/*read data from QMC5883L at every "wait" (ms)*/
	if((HAL_GetTick() - tickstart) < wait)
	{
		__asm("nop");
	}
	else
	{
		HAL_I2C_Mem_Read(&hi2c1, QMC5883L_ADDRESS, 0x0000, (uint16_t)1U, rx_rollover_u8, (uint16_t)6U, (uint32_t)100);

		QMC5883L.X_axis = ((rx_rollover_u8[1]<<8)|rx_rollover_u8[0])&0xFFFF;
		QMC5883L.Y_axis = ((rx_rollover_u8[3]<<8)|rx_rollover_u8[2])&0xFFFF;
		QMC5883L.Z_axis = ((rx_rollover_u8[5]<<8)|rx_rollover_u8[4])&0xFFFF;

		/* Calculate heading when the magnetometer is level, then correct for signs of axis.
		   Atan2() automatically check the correct formula taking care of the quadrant you are in*/
		heading = atan2(QMC5883L.X_axis, QMC5883L.Y_axis);

		declinationAngle = 0.0404;
		heading += declinationAngle;
		// Find yours here: http://www.magnetic-declination.com/

		/* Correct for when signs are reversed. */
		if(heading < 0)
		 heading += 2*3.14;

		/* Check for wrap due to addition of declination.*/
		if(heading > 2*3.14)
		 heading -= 2*3.14;

		/* Convert radians to degrees for readability. */
		headingDegrees_XY = heading * 180/3.14;

		/*get the angle difference report*/
		angle_diff_report();

		/*regulate the car moving direction*/
		car_moving_regulate();

		heading = atan2(QMC5883L.X_axis, QMC5883L.Z_axis);
		// Convert radians to degrees for readability.
		headingDegrees_XZ = heading * 180/3.14;

		heading = atan2(QMC5883L.Y_axis, QMC5883L.Z_axis);
		// Convert radians to degrees for readability.
		headingDegrees_YZ = heading * 180/3.14;

		/*get the time again for next-event*/
		tickstart = HAL_GetTick();
	}
}

void angle_diff_report(void)
{
	static uint8_t l_get_first_time = 1U;

	/*compare with old angle*/
	/*get the angle difference at the first time*/
	if(l_get_first_time == 1U)
	{
		headingDegrees_XY_old = headingDegrees_XY;
	}
	else
	{
		if(headingDegrees_XY > headingDegrees_XY_old)
		{
			angle_diff_u16 = headingDegrees_XY - headingDegrees_XY_old;
			direction_diff_u8 = 1U; /*stir to the left*/
		}
		else
		{
			angle_diff_u16 = headingDegrees_XY_old - headingDegrees_XY;
			direction_diff_u8 = 2U; /*stir to the right*/
		}
	}

	l_get_first_time = 0U;
}

void car_moving_regulate(void)
{
	/*if the car stir is too big*/
	if(angle_diff_u16 > max_angle)
	{
		switch(direction_diff_u8)
		{
			case(1U):
				/*stir to the left. Hence, make the car back to the right*/
			{
				/*decrease the right motor, increase the left motor*/

				break;
			}
			case(2U):
				/*stir to the right. Hence, make the car back to the left*/
			{
				/*decrease the left motor, increase the right motor*/

				break;
			}
			default:
			{

				break;
			}
		}
	}
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
