/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       oled.h
 * @brief      this file contains sd card basic operating function
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-28-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __OLED__H
#define __OLED__H

#ifdef  __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx.h"
#include "spi.h"
#include <stdint.h>

#define Max_Column      128
#define Max_Row         64

#define X_WIDTH         128
#define Y_WIDTH         64

#define OLED_CMD        0x00
#define OLED_DATA       0x01

#define CHAR_SIZE_WIDTH     6
#define VHAR_SIZE_HIGHT     12


#define OLED_CMD_Set()      HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)
#define OLED_CMD_Clr()      HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)

#define OLED_RST_Set()      HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET)
#define OLED_RST_Clr()      HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET) 

#define OLED_SCLK_Clr()     HAL_GPIO_WritePin(RM_OLEDB3_GPIO_Port, RM_OLEDB3_Pin, GPIO_PIN_SET)
#define OLED_SCLK_Set()     HAL_GPIO_WritePin(RM_OLEDB3_GPIO_Port, RM_OLEDB3_Pin, GPIO_PIN_RESET)

#define OLED_MOSI_Clr()     HAL_GPIO_WritePin(RM_OLEDA7_GPIO_Port, RM_OLEDA7_Pin, GPIO_PIN_SET)
#define OLED_MOSI_Set()     HAL_GPIO_WritePin(RM_OLEDA7_GPIO_Port, RM_OLEDA7_Pin, GPIO_PIN_RESET)


typedef enum
{
    Pen_Clear = 0x00,
    Pen_Write = 0x01,
    Pen_Inversion = 0x02,
}Pen_Typedef;

typedef enum
{
    Button_Mid   = 0,
    Button_Left  = 1,
    Button_Right = 2,
    Button_Up    = 3,
    Button_Down  = 4,
    Button_None  = 5
}Button_e;


/* function define */
void oled_init(void);
void oled_write_byte(uint8_t dat, uint8_t cmd);
void oled_display_on(void);
void oled_display_off(void);
void oled_refresh_gram(void);
void oled_clear(Pen_Typedef pen);
void oled_drawpoint(int8_t x, int8_t y, Pen_Typedef pen);
void oled_drawline(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, Pen_Typedef pen);
void oled_showchar(uint8_t row, uint8_t col, uint8_t chr);
void oled_shownum(uint8_t row, uint8_t col, int32_t num, uint8_t mode, uint8_t len);
void oled_showstring(uint8_t row, uint8_t col, uint8_t *chr);
void oled_printf(uint8_t row, uint8_t col, const char *fmt,...);
void RM_LOGO(void);
void OLED_ShowChinese(uint16_t x, uint16_t y, uint8_t* index, uint8_t size);
uint16_t ADC_Average(uint8_t size);
void OLED_Button(void);
void Wolf_SPI_Transmit(SPI_HandleTypeDef *hspi,uint8_t *pData,uint16_t Size);
void OLED_ShowMessage(uint8_t page);


extern uint8_t OLED_Page;

#ifdef __cplusplus
}
#endif

#endif








