#include "LCD1602_I2C.h"

/* Waveshare LCD1602 I2C Module (AiP31068L).
 * Every transfer is two bytes: a control byte, then the value.
 *   0x80 -> next byte is a command
 *   0x40 -> next byte is display data
 * No nibbles and no E strobe: the controller takes a full byte over I2C.
 * A 2-byte transfer at 100 kHz takes ~200 us, longer than the ~37 us a
 * normal instruction needs, so only clear/home need an explicit wait. */

static I2C_HandleTypeDef *s_hi2c = 0;

static void lcd_write(uint8_t control, uint8_t value)
{
    uint8_t buf[2] = { control, value };
    if (s_hi2c == 0) { return; }
    (void)HAL_I2C_Master_Transmit(s_hi2c, LCD_I2C_ADDR, buf, 2, 100);
}

void lcd_send_cmd(char cmd)   { lcd_write(0x80u, (uint8_t)cmd); }
void lcd_send_data(char data) { lcd_write(0x40u, (uint8_t)data); }

void lcd_send_string(const char *str)
{
    while (*str) { lcd_send_data(*str++); }
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01);
    HAL_Delay(2);           /* clear takes ~1.5 ms */
}

/* Row 0 starts at DDRAM address 0x00, row 1 at 0x40 */
void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0: col |= 0x80; break;
        case 1: col |= 0xC0; break;
        default: return;
    }
    lcd_send_cmd((char)col);
}

/* This module has no software backlight control; kept for API compatibility */
void lcd_backlight(int on)
{
    (void)on;
}

void lcd_init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;

    HAL_Delay(50);                       /* > 40 ms after power-up */
    lcd_send_cmd(0x28); HAL_Delay(5);    /* function set: 2 lines, 5x8 */
    lcd_send_cmd(0x28); HAL_Delay(5);
    lcd_send_cmd(0x28); HAL_Delay(1);
    lcd_send_cmd(0x0C); HAL_Delay(1);    /* display on, no cursor, no blink */
    lcd_clear();
    lcd_send_cmd(0x06); HAL_Delay(1);    /* entry mode: increment, no shift */
}

uint8_t lcd_scan_bus(I2C_HandleTypeDef *hi2c)
{
    for (uint8_t a = 0x02u; a < 0xFEu; a = (uint8_t)(a + 2u))
    {
        if (HAL_I2C_IsDeviceReady(hi2c, a, 2, 10) == HAL_OK)
        {
            return a;
        }
    }
    return 0u;
}
