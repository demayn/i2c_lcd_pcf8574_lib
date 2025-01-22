#ifndef I2C_LCD_PCF8574_H
#define I2C_LCD_PCF8574_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define LCD_MAX_ROWS 4

    typedef struct
    {
        uint8_t i2c_addr;
        bool backlight;
        uint8_t cols;
        uint8_t lines;
        uint8_t entrymode; // what happens aufter DDRAM r/w (enables setting properties individually and keeping the others)
        uint8_t displaycontrol; // display and cursor control (enables setting properties individually and keeping the others)
        uint8_t row_offsets[LCD_MAX_ROWS];
        i2c_master_dev_handle_t dev_hdl;
        i2c_master_bus_handle_t bus_hdl;
    } i2c_lcd_pcf8574_handle_t;

    /// @brief print the curent version od the library
    /// @param buf char pointer of the buffer to write the version to
    void i2c_lcd_pcf8574_version(char buf[6]);

    // Initialize the LCD
    void lcd_init(i2c_lcd_pcf8574_handle_t *lcd, uint8_t i2c_addr, i2c_master_bus_handle_t bus_hdl);

    // Begin using the LCD
    void lcd_begin(i2c_lcd_pcf8574_handle_t *lcd, uint8_t cols, uint8_t rows);

    // Clear the LCD
    void lcd_clear(i2c_lcd_pcf8574_handle_t *lcd);

    // Move cursor to home position
    void lcd_home(i2c_lcd_pcf8574_handle_t *lcd);

    // Set cursor position
    void lcd_set_cursor(i2c_lcd_pcf8574_handle_t *lcd, uint8_t col, uint8_t row);

    // Turn the display on/off
    void lcd_display_onoff(i2c_lcd_pcf8574_handle_t *lcd, bool on);

    // Turn the cursor on/off
    void lcd_cursor_onoff(i2c_lcd_pcf8574_handle_t *lcd, bool on);

    // Turn blinking cursor on/off
    void lcd_blink_onoff(i2c_lcd_pcf8574_handle_t *lcd, bool on);

    // Scroll the display
    void lcd_scroll_display(i2c_lcd_pcf8574_handle_t *lcd, bool right);

    // Set the direction for text that flows automatically
    void lcd_entry_mode(i2c_lcd_pcf8574_handle_t *lcd, bool left2right);

    // Turn on/off autoscroll
    void lcd_autoscroll(i2c_lcd_pcf8574_handle_t *lcd, bool on);

    // Set backlight onoff
    void lcd_set_backlight(i2c_lcd_pcf8574_handle_t *lcd, bool backlight_on);

    // Create a custom character
    void lcd_create_char(i2c_lcd_pcf8574_handle_t *lcd, uint8_t location, uint8_t charmap[]);

    // Write a character to the LCD
    void lcd_write(i2c_lcd_pcf8574_handle_t *lcd, uint8_t value);

    // Print a string to the LCD
    void lcd_print(i2c_lcd_pcf8574_handle_t *lcd, const char *str);

    void lcd_print_number(i2c_lcd_pcf8574_handle_t *lcd, uint8_t col, uint8_t row, uint8_t buf_len, const char *str, ...);

#ifdef __cplusplus
}
#endif // C++ extern

#endif // I2C_LCD_PCF8574_H