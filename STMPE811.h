/*
 * A simple single-header STMPE811/STMPE610 driver using HAL with 2-point calibration and averaging
 *
 * License: MIT(http://opensource.org/licenses/MIT)
 *
 */

#ifndef INC_STMPE811_H_
#define INC_STMPE811_H_

#include "stm32h7xx_hal.h"

#define STMPE811_I2C_ADDR 0x82  //including W bit

#define STMPE811_CHIP_ID            0x00
#define STMPE811_ID_VER             0x02
#define STMPE811_SYS_CTRL1          0x03
#define STMPE811_SYS_CTRL2          0x04
#define STMPE811_SPI_CFG            0x08
#define STMPE811_INT_CTRL           0x09
#define STMPE811_INT_EN             0x0A
#define STMPE811_INT_STA            0x0B
#define STMPE811_GPIO_EN            0x0C
#define STMPE811_GPIO_INT_STA       0x0D
#define STMPE811_ADC_INT_EN         0x0E
#define STMPE811_ADC_INT_STA        0x0F
#define STMPE811_GPIO_SET_PIN       0x10
#define STMPE811_GPIO_CLR_PIN       0x11
#define STMPE811_MP_STA             0x12
#define STMPE811_GPIO_DIR           0x13
#define STMPE811_GPIO_ED            0x14
#define STMPE811_GPIO_RE            0x15
#define STMPE811_GPIO_FE            0x16
#define STMPE811_GPIO_ALT_FUNCTION  0x17
#define STMPE811_ADC_CTRL1          0x20
#define STMPE811_ADC_CTRL2          0x21
#define STMPE811_ADC_CAPT           0x22
#define STMPE811_ADC_DATA_CHO       0x30
#define STMPE811_ADC_DATA_CH1       0x32
#define STMPE811_ADC_DATA_CH2       0x34
#define STMPE811_ADC_DATA_CH3       0x36
#define STMPE811_ADC_DATA_CH4       0x38
#define STMPE811_ADC_DATA_CH5       0x3A
#define STMPE811_ADC_DATA_CH6       0x3C
#define STMPE811_ADC_DATA_CH7       0x3E
#define STMPE811_TSC_CTRL           0x40
#define STMPE811_TSC_CFG            0x41
#define STMPE811_WDW_TR_X           0x42
#define STMPE811_WDW_TR_Y           0x44
#define STMPE811_WDW_BL_X           0x46
#define STMPE811_WDW_BL_Y           0x48
#define STMPE811_FIFO_TH            0x4A
#define STMPE811_FIFO_CTRL_STA      0x4B
#define STMPE811_FIFO_SIZE          0x4C
#define STMPE811_TSC_DATA_X         0x4D
#define STMPE811_TSC_DATA_Y         0x4F
#define STMPE811_TSC_DATA_Z         0x51
#define STMPE811_TSC_DATA_XYZ       0x52
#define STMPE811_TSC_DATA_INC       0x57
#define STMPE811_TSC_DATA_NON_INC   0xD7
#define STMPE811_TSC_FRACTION_Z     0x56
#define STMPE811_TSC_DATA           0x57
#define STMPE811_TSC_I_DRIVE        0x58
#define STMPE811_TSC_SHIELD         0x59
#define STMPE811_TEMP_CTRL          0x60
#define STMPE811_TEMP_DATA          0x61
#define STMPE811_TEMP_TH            0x62

#define STMPE811_ADC_FCT            0x01
#define STMPE811_TP_FCT             0x02
#define STMPE811_IO_FCT             0x04

static uint8_t tp_cnt, tp_swap_xy, tp_inv_x, tp_inv_y;
static uint16_t tp_hor_res, tp_ver_res, tp_x_min, tp_x_max, tp_y_min, tp_y_max;

uint16_t STMPE811_ReadID(I2C_HandleTypeDef *hi2c)
{
	uint8_t d[2];

	if (HAL_I2C_Mem_Read(hi2c, (uint16_t) STMPE811_I2C_ADDR, STMPE811_CHIP_ID, I2C_MEMADD_SIZE_8BIT, (uint8_t*) d, 2, 2) != HAL_OK)
		return 0;

	if ((int16_t) d[0] < 0 || (int16_t) d[1] < 0)
		return 0;

	uint16_t out = (int16_t) d[0];
	out = (uint32_t) (out << 8);
	out |= (int16_t) d[1];

	return (uint16_t) out;
}

HAL_StatusTypeDef STMPE811_Reset(I2C_HandleTypeDef *hi2c)
{
	uint8_t temp = 0x02;
	if (HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_SYS_CTRL1, I2C_MEMADD_SIZE_8BIT, &temp, 1, 2) != HAL_OK)
		return HAL_ERROR;

	HAL_Delay(20);

	temp = 0x00;
	return HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_SYS_CTRL1, I2C_MEMADD_SIZE_8BIT, &temp, 1, 2);
}

HAL_StatusTypeDef STMPE811_Function(I2C_HandleTypeDef *hi2c, uint8_t function, uint8_t en)
{
	uint8_t temp;
	int16_t data;

	HAL_I2C_Mem_Read(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_SYS_CTRL2, I2C_MEMADD_SIZE_8BIT, &temp, 1, 2);
	data = (int16_t) temp;

	if (data < 0)
		return HAL_ERROR;

	temp = (uint8_t) (data);

	if (en)
		temp &= ~(uint8_t) function;
	else
		temp |= (uint8_t) function;

	HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_SYS_CTRL2, I2C_MEMADD_SIZE_8BIT, &temp, 1, 2);

	return HAL_OK;
}

HAL_StatusTypeDef STMPE811_Touch_Config(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef ret = HAL_OK;

	static uint8_t regArray[12] =
	{ 0x50, // ADC CTRL1
	  0x00, // ADC CTRL2
	  0x9A, // TSC CFG
	  0x01, // FIFO TH
	  0x01, // FIFO CTRL
	  0x00, // FIFO CTRL
	  0x01, // TSC FRACTION
	  0x01, // TSC I DRIVE
	  0x03, // TSC CTRL
	  0xFF, // INT STA
	  0x01, // INT CTRL
	  0x01  // INT ENA
	        };
	ret = STMPE811_Function(hi2c, STMPE811_TP_FCT, 1);

	ret = HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_ADC_CTRL1, I2C_MEMADD_SIZE_8BIT, &regArray[0], 1, 2);
	ret = HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_INT_CTRL, I2C_MEMADD_SIZE_8BIT, &regArray[10], 1, 2);
	ret = HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_INT_EN, I2C_MEMADD_SIZE_8BIT, &regArray[11], 1, 2);

	HAL_Delay(20);

	ret = HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_ADC_CTRL2, I2C_MEMADD_SIZE_8BIT, &regArray[1], 1, 2);
	ret = HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_TSC_CFG, I2C_MEMADD_SIZE_8BIT, &regArray[2], 1, 2);
	ret = HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_FIFO_TH, I2C_MEMADD_SIZE_8BIT, &regArray[3], 1, 2);
	ret = HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_FIFO_CTRL_STA, I2C_MEMADD_SIZE_8BIT, &regArray[4], 1, 2);
	ret = HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_FIFO_CTRL_STA, I2C_MEMADD_SIZE_8BIT, &regArray[5], 1, 2);
	ret = HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_TSC_FRACTION_Z, I2C_MEMADD_SIZE_8BIT, &regArray[6], 1, 2);
	ret = HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_TSC_I_DRIVE, I2C_MEMADD_SIZE_8BIT, &regArray[7], 1, 2);
	ret = HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_TSC_CTRL, I2C_MEMADD_SIZE_8BIT, &regArray[8], 1, 2);
	ret = HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_INT_STA, I2C_MEMADD_SIZE_8BIT, &regArray[9], 1, 2);

	return ret;
}

HAL_StatusTypeDef STMPE811_Read16(I2C_HandleTypeDef *hi2c, uint32_t RegisterAddr, uint16_t *out)
{
	uint8_t d[2];

	if (HAL_I2C_Mem_Read(hi2c, (uint16_t) STMPE811_I2C_ADDR, RegisterAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t*) d, 2, 2) != HAL_OK)
		return HAL_ERROR;

	if ((int16_t) d[0] < 0 || (int16_t) d[1] < 0)
		return HAL_ERROR;

	*out = (int16_t) d[0];
	*out = (uint32_t) (*out << 8);
	*out |= (int16_t) d[1];

	return HAL_OK;
}

void STMPE811_SetCalData(uint8_t tp_swap_xy_, uint8_t tp_inv_x_, uint8_t tp_inv_y_, uint16_t tp_hor_res_, uint16_t tp_ver_res_, uint16_t tp_x_min_, uint16_t tp_x_max_, uint16_t tp_y_min_, uint16_t tp_y_max_)
{
	tp_swap_xy = tp_swap_xy_;
	tp_inv_x = tp_inv_x_;
	tp_inv_y = tp_inv_y_;
	tp_hor_res = tp_hor_res_;
	tp_ver_res = tp_ver_res_;
	tp_x_min = tp_x_min_;
	tp_x_max = tp_x_max_;
	tp_y_min = tp_y_min_;
	tp_y_max = tp_y_max_;
}

static void STMPE811_Correct(uint16_t *x, uint16_t *y)
{
	if (tp_swap_xy)
	{
		int16_t swap_tmp;
		swap_tmp = *x;
		*x = *y;
		*y = swap_tmp;
	}

	if (*x > tp_x_min)
		*x -= tp_x_min;
	else
		*x = 0;

	if (*y > tp_y_min)
		*y -= tp_y_min;
	else
		*y = 0;

	*x = (uint32_t) ((uint32_t) (*x) * tp_hor_res) / (tp_x_max - tp_x_min);
	*y = (uint32_t) ((uint32_t) (*y) * tp_ver_res) / (tp_y_max - tp_y_min);

	if (tp_inv_x)
		*x = tp_hor_res - *x;

	if (tp_inv_y)
		*y = tp_ver_res - *y;

	if (*x > tp_hor_res)
		*x = tp_hor_res;

}

HAL_StatusTypeDef STMPE811_Read(I2C_HandleTypeDef *hi2c, uint16_t *x, uint16_t *y, uint8_t *touching)
{
	HAL_StatusTypeDef ret = HAL_OK;

	if (HAL_I2C_Mem_Read(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_FIFO_SIZE, I2C_MEMADD_SIZE_8BIT, &tp_cnt, 1, 2) != HAL_OK)
		return HAL_ERROR;

	static uint8_t ficon[3] =
	{ 0x01, 0x00, 0xFF };

	if (tp_cnt > 0)
	{
		uint32_t x_sum = 0, y_sum = 0;
		for (uint8_t i = tp_cnt; i > 0; i--) // Average out
		{
			uint16_t xt, yt;
			ret = STMPE811_Read16(hi2c, STMPE811_TSC_DATA_X, (uint16_t*) &xt);
			ret = STMPE811_Read16(hi2c, STMPE811_TSC_DATA_Y, (uint16_t*) &yt);

			x_sum += xt;
			y_sum += yt;

			if (ret == HAL_ERROR)
				return HAL_ERROR;

		}

		*x = (x_sum / tp_cnt);
		*y = (y_sum / tp_cnt);

		if (tp_hor_res != 0)
			STMPE811_Correct(x, y);

		*touching = 1;
	}
	else
		*touching = 0;

	ret = HAL_I2C_Mem_Write(hi2c, (uint16_t) STMPE811_I2C_ADDR, (uint16_t) STMPE811_INT_STA, I2C_MEMADD_SIZE_8BIT, &ficon[0], 1, 2); // Reset touching interrupt

	return ret;
}

HAL_StatusTypeDef STMPE811_Init(I2C_HandleTypeDef *hi2c)
{
	if (STMPE811_ReadID(hi2c) != 0x811)
		return HAL_ERROR;

	if (STMPE811_Reset(hi2c) != HAL_OK)
		return HAL_ERROR;

	if (STMPE811_Function(hi2c, STMPE811_ADC_FCT, 1) != HAL_OK)
		return HAL_ERROR;

	if (STMPE811_Touch_Config(hi2c) != HAL_OK)
		return HAL_ERROR;

	return HAL_OK;
}
#endif /* INC_STMPE811_H_ */
