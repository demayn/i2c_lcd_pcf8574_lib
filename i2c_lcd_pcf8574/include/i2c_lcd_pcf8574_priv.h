#ifndef I2C_LCD_PCF8574_PRIV_H
#define I2C_LCD_PCF8574_PRIV_H

#include "i2c_lcd_pcf8574.h"

#define TAG "I2C_LCD_PCF8574"

#define I2C_MASTER_TIMEOUT_MS 1000

// commands
enum command_bitmask
{
    // clears the display
    lcd_clear_mask = 0x01,
    // return cursor to home position
    cursor_home_mask = 0x02,
    // what happens after DDRAM read/write
    lcd_entrymode_mask = 0x04,
    // lcd on off control
    lcd_control_mask = 0x08,
    // shifts the cursor or the whole display
    cursor_disp_shift_mask = 0x10,
    // set lcd functionality (4/8 Bit, Character size and )
    lcd_function_mask = 0x20,
    // set custom graphics(?) RAM address; write 5 Bytes for one custom char (top down, left right); can be accessed to display via 0x00 to 0x07 in DDRAM
    cgram_addr_set_mask = 0x40,
    // set display data(?) RAM address
    ddram_addr_set_mask = 0x80,
};

enum pcf_pinmask
{
    // 1 for data, 0 for instruction
    rs_mask = 0x01,
    // 1 for read, 0 for write
    rw_mask = 0x02,
    // chip reads/writes on falling edge
    en_mask = 0x04,
    // backlight control (not always connected)
    backlight_mask = 0x08,
    // Data Bus (4-Bit Mode)
    DB4_mask = 0x10,
    DB5_mask = 0x20,
    DB6_mask = 0x40,
    DB7_mask = 0x80,
};

// settings as defines
#define ENTRYMODE_LEFT2RIGHT 0x02
#define ENTRYMODE_AUTOSCROLL 0x01

#define DISPLAY_ON 0x04
#define CURSOR_ON 0x02
#define CURSOR_BLINK 0x01

#define DISPLAY_SHIFT 0x08 // display or cursor shift
#define SHIFT_RIGHT 0x04   // right or left shift

#define FUNCTION_8BIT 0x10   // 8- or 4-Bit Mode
#define FUNCTION_2LINES 0x08 // use one or two lines of the LCD
#define FUNCTION_5X11 0x04   // 5x11 or 5x8 Characters

// private functions
static void lcd_send(i2c_lcd_pcf8574_handle_t *lcd, uint8_t value);
static void lcd_instr(i2c_lcd_pcf8574_handle_t *lcd, uint8_t value);
static void lcd_write_nibble(i2c_lcd_pcf8574_handle_t *lcd, uint8_t half_byte, bool is_data);

#endif