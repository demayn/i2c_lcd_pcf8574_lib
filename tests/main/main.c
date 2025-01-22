// This example project shows various features basic setup to use the i2c_lcd_pcf8574 driver with 16 chars by 2 lines.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "i2c_lcd_pcf8574.h"

#define I2C_MASTER_SCL_IO 5       // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO 6       // GPIO number for I2C master data
#define I2C_MASTER_NUM I2C_NUM_0  // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 100000 // I2C master clock frequency

#define LCD_ADDR 0x27 // I2C address of the LCD
#define LCD_COLS 20   // Number of columns in the LCD
#define LCD_ROWS 4    // Number of rows in the LCD

static const char *TAG = "LCD_EXAMPLE";

i2c_master_bus_config_t i2c_mst_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_6,
    .scl_io_num = GPIO_NUM_5,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .intr_priority = 0,
    .trans_queue_depth = 0,
    .flags.enable_internal_pullup = true, // enable internal pullups
};
i2c_master_bus_handle_t bus_handle;

void i2c_master_init()
{
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing I2C");
    i2c_master_init();

    ESP_LOGI(TAG, "Initializing LCD");

    i2c_lcd_pcf8574_handle_t lcd;
    lcd_init(&lcd, LCD_ADDR, bus_handle);
    ESP_LOGI(TAG, "LCD initialized");
    ESP_LOGI(TAG, "LCD begin");
    lcd_begin(&lcd, LCD_COLS, LCD_ROWS);
    ESP_LOGI(TAG, "LCD rdy");

    // Turn on the backlight
    lcd_set_backlight(&lcd, true);
    ESP_LOGI(TAG, "LCD rdy");

    // Print a message
    vTaskDelay(1);
    lcd_set_cursor(&lcd, 0, 0);
    vTaskDelay(1);
    lcd_print(&lcd, "Hello, ESP32!");
    vTaskDelay(1);
    lcd_set_cursor(&lcd, 0, 1);
    vTaskDelay(1);
    lcd_print(&lcd, "LCD Test");

    int counter = 0;
    while (1)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for 1 second

        // Update the counter on the LCD
        lcd_set_cursor(&lcd, 10, 1);
        char buffer[12];
        snprintf(buffer, sizeof(buffer), "%5d", counter);
        lcd_print(&lcd, buffer);

        counter++;

        // Reset counter to avoid display overflow
        if (counter > 99999)
            counter = 0;
    }
}