#include <stdio.h>
#include "i2c_lcd_pcf8574.h"
#include "i2c_lcd_pcf8574_priv.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_task.h"

#ifndef APP_VERSION
#define APP_VERSION "unknown"
#endif

/// @brief copies and initializes struct data
/// @param lcd the lcd handle
/// @param i2c_addr pcf 8574 i2c address (per default 0x27)
/// @param bus_hdl bus handle of the esp32 i2c driver
void lcd_init(i2c_lcd_pcf8574_handle_t *lcd, uint8_t i2c_addr, i2c_master_bus_handle_t bus_hdl)
{
    ESP_LOGI(TAG, APP_VERSION);
    lcd->i2c_addr = i2c_addr;
    lcd->backlight = true;                 // sometimes not supported (PCF5874 P3 is NC on some modules)
    lcd->entrymode = ENTRYMODE_LEFT2RIGHT; // Init the LCD with an internal reset
    lcd->displaycontrol = DISPLAY_ON & ~CURSOR_ON & ~CURSOR_BLINK;
    lcd->bus_hdl = bus_hdl;
}

/// @brief initializes the display (I2 and startup sequence)
/// @param lcd screen handle
/// @param cols number of columns
/// @param rows number of rows
void lcd_begin(i2c_lcd_pcf8574_handle_t *lcd, uint8_t cols, uint8_t rows)
{
    i2c_device_config_t lcd_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = lcd->i2c_addr,
        .scl_speed_hz = 100000,
    };

    i2c_master_bus_add_device(lcd->bus_hdl, &lcd_dev_cfg, &lcd->dev_hdl);

    // Ensure the cols and rows stay within max limit
    lcd->cols = (cols > 80) ? 80 : cols;
    lcd->lines = (rows > 4) ? 4 : rows;

    lcd->row_offsets[0] = 0x00;
    lcd->row_offsets[1] = 0x40;
    lcd->row_offsets[2] = 0x00 + cols;
    lcd->row_offsets[3] = 0x40 + cols;

    // Initialize the LCD
    // write en low (all pins low except backlight) by hand
    uint8_t buf = 0x8;
    ESP_ERROR_CHECK(i2c_master_transmit(lcd->dev_hdl, &buf, 1, I2C_MASTER_TIMEOUT_MS));
    // startup delay (via taskdelay to let other tasks do their thing)
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // The following are the reset sequence as found on the intrenet (see doku)
    // 3x "special function set"
    lcd_write_nibble(lcd, 0x03, false);
    esp_rom_delay_us(4500);
    lcd_write_nibble(lcd, 0x03, false);
    esp_rom_delay_us(200);
    lcd_write_nibble(lcd, 0x03, false);
    esp_rom_delay_us(200);

    // another special function set with correct 4 bit mode bit
    lcd_write_nibble(lcd, 0x02, false);
    esp_rom_delay_us(200);

    // actual function set
    lcd_instr(lcd, (lcd_function_mask | (rows > 1 ? FUNCTION_2LINES : 0x00)) & (~FUNCTION_5X11));
    esp_rom_delay_us(60);

    // display off
    lcd_display_onoff(lcd, false);
    esp_rom_delay_us(60);

    // display clear
    lcd_clear(lcd);
    esp_rom_delay_us(60);

    // entry mode set
    lcd_instr(lcd, 0x05 | lcd->entrymode);
    esp_rom_delay_us(60);

    // entry mode set
    lcd_entry_mode(lcd, true);
    esp_rom_delay_us(60);

    // display on
    lcd_display_onoff(lcd, true);
    esp_rom_delay_us(60);
}

/// @brief clear the display
/// @param lcd display handle
void lcd_clear(i2c_lcd_pcf8574_handle_t *lcd)
{
    lcd_instr(lcd, lcd_clear_mask);
    // takes approx. 1.5ms, we give approx. 10 and let other tasks do their thing
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

/// @brief set the cursor to home position
/// @param lcd display handle
void lcd_home(i2c_lcd_pcf8574_handle_t *lcd)
{
    // Instruction: Return home = 0x02
    lcd_instr(lcd, 0x02);
    // takes approx. 1.5ms, we give approx. 10 and let other tasks do their thing
    vTaskDelay(10 / portTICK_PERIOD_MS);
} // lcd_home()

/// @brief set the cursor to a new position (set DDRAM Address)
/// @param lcd display handle
/// @param col target column position
/// @param row target row position
void lcd_set_cursor(i2c_lcd_pcf8574_handle_t *lcd, uint8_t col, uint8_t row)
{
    // Check the display boundaries
    if (row >= lcd->lines)
    {
        row = lcd->lines - 1;
    }
    if (col >= lcd->cols)
    {
        col = lcd->cols - 1;
    }
    lcd_instr(lcd, ddram_addr_set_mask | (lcd->row_offsets[row] + col));
}

/// @brief Turn the display on or off
/// @param lcd display handle
/// @param on true for on, false for off
void lcd_display_onoff(i2c_lcd_pcf8574_handle_t *lcd, bool on)
{
    if (on)
        lcd->displaycontrol |= DISPLAY_ON;
    else
        lcd->displaycontrol &= ~DISPLAY_ON;
    lcd_instr(lcd, lcd_control_mask | lcd->displaycontrol);
}

/// @brief Turn the cursor on or off
/// @param lcd display handle
/// @param on true for on, false for off
void lcd_cursor_onoff(i2c_lcd_pcf8574_handle_t *lcd, bool on)
{
    // Display Control: Cursor on/off control = 0x02
    if (on)
        lcd->displaycontrol |= CURSOR_ON;
    else
        lcd->displaycontrol &= ~CURSOR_ON;
    // Instruction: Display mode: 0x08
    lcd_instr(lcd, lcd_control_mask | lcd->displaycontrol);
}

/// @brief Turn the cursor blinking on or off
/// @param lcd display handle
/// @param on true for on, false for off
void lcd_blink_onoff(i2c_lcd_pcf8574_handle_t *lcd, bool on)
{
    // Display Control: Blink on/off control = 0x01
    if (on)
        lcd->displaycontrol |= CURSOR_BLINK;
    else
        lcd->displaycontrol &= ~CURSOR_BLINK;
    // Instruction: Display mode: 0x08
    lcd_instr(lcd, lcd_control_mask | lcd->displaycontrol);
}

/// @brief Shifts the displayed text without changing data
/// @param lcd display handle
/// @param right true for rightshift, false for leftshift
void lcd_scroll_display(i2c_lcd_pcf8574_handle_t *lcd, bool right)
{
    lcd_instr(lcd, cursor_disp_shift_mask | DISPLAY_SHIFT | (right ? SHIFT_RIGHT : 0));
}

/// @brief control entry mode (what happens aufter DDRAM r/w)
/// @param lcd display handle
/// @param left2right true for legft to right text; false for right to left
void lcd_entry_mode(i2c_lcd_pcf8574_handle_t *lcd, bool left2right)
{
    if (left2right)
        lcd->entrymode |= ENTRYMODE_LEFT2RIGHT;
    else
        lcd->entrymode &= ~ENTRYMODE_LEFT2RIGHT;
    lcd_instr(lcd, lcd_entrymode_mask | lcd->entrymode);
}

/// @brief control autoscroll (if actiuve, shifts the display contents with every write operation)
/// @param lcd display handle
/// @param on true to enable, false to disable
void lcd_autoscroll(i2c_lcd_pcf8574_handle_t *lcd, bool on)
{
    // Instruction: Entry mode set, set shift = 0x01
    if (on)
        lcd->entrymode |= ENTRYMODE_AUTOSCROLL;
    else
        lcd->entrymode &= ~ENTRYMODE_AUTOSCROLL;
    lcd_instr(lcd, lcd_entrymode_mask | lcd->entrymode);
}

/// @brief en-/disable LED Backlight
/// @param lcd display handle
/// @param backlight_on true to enable, false to disable
void lcd_set_backlight(i2c_lcd_pcf8574_handle_t *lcd, bool backlight_on)
{
    lcd->backlight = backlight_on;
    uint8_t buf = backlight_mask * backlight_on;
    // set pin "by hand"
    i2c_master_transmit(lcd->dev_hdl, &buf, 1, -1);
}

/// @brief save a custom acarcter to CGRAM
/// @param lcd display handle
/// @param index 0-7; index of the new character
/// @param charmap 5*8 array display data (right (LSB) to left (MSB), top to bottom (Byte order))
void lcd_create_char(i2c_lcd_pcf8574_handle_t *lcd, uint8_t index, uint8_t charmap[])
{
    index &= 0x7;
    lcd_instr(lcd, cgram_addr_set_mask | (index << 3));
    for (int i = 0; i < 8; i++)
    {
        lcd_send(lcd, charmap[i]);
    }
}

/// @brief print string to lcd
/// @param lcd display handle
/// @param str char array to print (terminated via '\0)
void lcd_print(i2c_lcd_pcf8574_handle_t *lcd, const char *str)
{
    while (*str)
    {
        lcd_send(lcd, *str++);
    }
}

// Private functions

/// @brief send data to lcd
/// @param lcd display handle
/// @param value value to send
static void lcd_send(i2c_lcd_pcf8574_handle_t *lcd, uint8_t value)
{
    bool is_data = true;
    lcd_write_nibble(lcd, ((value >> 4) & 0x0F), is_data);
    lcd_write_nibble(lcd, (value & 0x0F), is_data);
}

/// @brief send instruction to lcd
/// @param lcd display handle
/// @param instruction instruction to send
static void lcd_instr(i2c_lcd_pcf8574_handle_t *lcd, uint8_t instruction)
{
    bool is_data = false;
    lcd_write_nibble(lcd, ((instruction >> 4) & 0x0F), is_data);
    lcd_write_nibble(lcd, (instruction & 0x0F), is_data);
}

/// @brief send halfbyte (=nibble) to lcd
/// @param lcd display handle
/// @param half_byte halfbyte to send (0x00 to 0x0F)
/// @param is_data set true if data is transmitted, false for instruction (rs pin)
static void lcd_write_nibble(i2c_lcd_pcf8574_handle_t *lcd, uint8_t half_byte, bool is_data)
{
    uint8_t data = is_data ? rs_mask : 0;

    if (lcd->backlight)
    {
        data |= backlight_mask;
    }

    // shift to upper 4 ports of PCF5874 (connected to LCD Data Pins)
    data |= (half_byte << 4);

    // write data and generate falling edge on EN
    uint8_t buf = data | en_mask;
    // ESP_LOGI(TAG, "IDK");
    ESP_ERROR_CHECK(i2c_master_transmit(lcd->dev_hdl, &buf, 1, I2C_MASTER_TIMEOUT_MS));
    esp_rom_delay_us(10);
    // i2c_master_bus_wait_all_done(lcd->bus_hdl, I2C_MASTER_TIMEOUT_MS);
    buf = data;
    ESP_ERROR_CHECK(i2c_master_transmit(lcd->dev_hdl, &buf, 1, I2C_MASTER_TIMEOUT_MS));
    // i2c_master_bus_wait_all_done(lcd->bus_hdl, I2C_MASTER_TIMEOUT_MS);
    esp_rom_delay_us(10);
}