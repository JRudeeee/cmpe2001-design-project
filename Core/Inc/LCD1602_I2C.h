/**
  ******************************************************************************
  * @file    LCD1602_I2C.h
  * @brief   16x2 LCD over I2C - HOME VERSION for the Waveshare LCD1602 I2C
  *          Module (AiP31068L controller, no PCF8574 backpack).
  *
  *          Same public API as the lab's PCF8574 driver (Support Materials,
  *          section 5). In the lab, replace this .h/.c pair with theirs;
  *          application code does not change.
  ******************************************************************************
  */

#ifndef INC_LCD1602_I2C_H_
#define INC_LCD1602_I2C_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>

/* AiP31068L 7-bit address 0x3E, shifted left by one for the HAL (= 0x7C).
 * Run lcd_scan_bus() if nothing displays. */
#define LCD_I2C_ADDR   (0x3E << 1)

void lcd_init(I2C_HandleTypeDef *hi2c);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(const char *str);
void lcd_put_cur(int row, int col);
void lcd_clear(void);
void lcd_backlight(int on);

/** Probe every address on the bus. Returns the first 8-bit address that
 * answers, or 0 if nothing does. */
uint8_t lcd_scan_bus(I2C_HandleTypeDef *hi2c);

#endif /* INC_LCD1602_I2C_H_ */
