#include "freertos/FreeRTOS.h"

#include "esp_timer.h"
#include "esp_log.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include <driver/i2c.h>

#include "mpu6050.h"

#define DISP_WIDTH 240
#define DISP_HEIGHT 240
#define DISP_DRAW_BUFFER_HEIGHT 40
#define DISP_GPIO_RES GPIO_NUM_0
#define DISP_GPIO_DC GPIO_NUM_1
#define DISP_SCLK GPIO_NUM_4
#define DISP_MOSI GPIO_NUM_6

static esp_lcd_panel_handle_t main_lcd_panel_handle;
static lv_disp_t* lvgl_main_display_handle;

static mpu6050_handle_t* mpu6050_handle;

void init_spi_bus(){
    const spi_bus_config_t spi_bug_cfg = {
            .sclk_io_num = DISP_SCLK,
            .mosi_io_num = DISP_MOSI,
            .miso_io_num = GPIO_NUM_NC,
            .quadwp_io_num = GPIO_NUM_NC,
            .quadhd_io_num = GPIO_NUM_NC,
            .max_transfer_sz = DISP_HEIGHT * DISP_DRAW_BUFFER_HEIGHT * sizeof(uint16_t)
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bug_cfg, SPI_DMA_CH_AUTO));
}

void i2c_setup(){
    i2c_config_t i2c_config =
            {
                    .mode = I2C_MODE_MASTER,
                    .sda_io_num = GPIO_NUM_8,
                    .sda_pullup_en = GPIO_PULLUP_ENABLE,
                    .scl_io_num = GPIO_NUM_9,
                    .scl_pullup_en = GPIO_PULLUP_ENABLE,
                    .master.clk_speed =  10 * 10000,
            };
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void init_lvgl_disp(){
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    /* LCD IO */
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
            .dc_gpio_num = DISP_GPIO_DC,
            .cs_gpio_num = GPIO_NUM_NC,
            .pclk_hz = 40 * 1000 * 1000,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            // this varies depending on hardware
            // bugfix ref: https://github.com/Bodmer/TFT_eSPI/issues/163
            .spi_mode = 3,
            .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) SPI2_HOST, &io_config, &io_handle));

    /* LCD driver initialization */

    const esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = DISP_GPIO_RES,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
            .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &main_lcd_panel_handle));

    esp_lcd_panel_reset(main_lcd_panel_handle);
    esp_lcd_panel_init(main_lcd_panel_handle);
    // turn off display before first lvgl update to avoid flickering
    esp_lcd_panel_disp_on_off(main_lcd_panel_handle, false);
    // depend on hardware
    esp_lcd_panel_invert_color(main_lcd_panel_handle, true);
    esp_lcd_panel_set_gap(main_lcd_panel_handle, 0, 80);

    /* Add LCD screen */
    const lvgl_port_display_cfg_t disp_cfg = {
            .io_handle = io_handle,
            .panel_handle = main_lcd_panel_handle,
            .buffer_size = DISP_HEIGHT * DISP_DRAW_BUFFER_HEIGHT * sizeof(uint16_t),
            .double_buffer = true,
            .hres = DISP_WIDTH,
            .vres = DISP_HEIGHT,
            .monochrome = false,
            .color_format = LV_COLOR_FORMAT_RGB565,
            .rotation = {
                    .swap_xy = false,
                    .mirror_x = true,
                    .mirror_y = true,
            },
            .flags = {
                    .buff_dma = true,
                    .swap_bytes = false,
            }
    };

    lvgl_main_display_handle = lvgl_port_add_disp(&disp_cfg);
}

void init_lvgl_scene(void){
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_update_layout(lv_screen_active());
}

void task_mpu6050(void* pvParameters) {

    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_text_color(lv_screen_active(), lv_color_hex(0xffffff), LV_PART_MAIN);

    lv_style_t label_style_title;
    lv_style_t label_style_value;
    lv_style_init(&label_style_title);
    lv_style_init(&label_style_value);

    lv_style_set_pad_all(&label_style_title, 24);
    lv_style_set_pad_all(&label_style_value, 10);
    lv_style_set_text_font(&label_style_title, &lv_font_montserrat_20);
    lv_style_set_text_color(&label_style_title, lv_color_hex(0xb3b3b3));
    lv_style_set_text_font(&label_style_value, &lv_font_montserrat_26);

    lv_obj_t *label_gyro_title = lv_label_create(lv_screen_active());
    lv_obj_t *label_acceleration_title = lv_label_create(lv_screen_active());

    lv_obj_t *label_gyro_val_x = lv_label_create(lv_screen_active());
    lv_obj_t *label_gyro_val_y = lv_label_create(lv_screen_active());
    lv_obj_t *label_gyro_val_z = lv_label_create(lv_screen_active());
    lv_obj_t *label_acceleration_val_x = lv_label_create(lv_screen_active());
    lv_obj_t *label_acceleration_val_y = lv_label_create(lv_screen_active());
    lv_obj_t *label_acceleration_val_z = lv_label_create(lv_screen_active());
    lv_obj_t *label_last_failure_time_val = lv_label_create(lv_screen_active());

    // keep hidden until aligned
    lv_obj_add_flag(label_gyro_title, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(label_acceleration_title, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(label_gyro_val_x, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(label_gyro_val_y, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(label_gyro_val_z, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(label_acceleration_val_x, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(label_acceleration_val_y, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(label_acceleration_val_z, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(label_last_failure_time_val, LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_style(label_gyro_title, &label_style_title, LV_STATE_DEFAULT);
    lv_obj_add_style(label_acceleration_title, &label_style_title, LV_STATE_DEFAULT);
    lv_obj_add_style(label_gyro_val_x, &label_style_value, LV_STATE_DEFAULT);
    lv_obj_add_style(label_gyro_val_y, &label_style_value, LV_STATE_DEFAULT);
    lv_obj_add_style(label_gyro_val_z, &label_style_value, LV_STATE_DEFAULT);
    lv_obj_add_style(label_acceleration_val_x, &label_style_value, LV_STATE_DEFAULT);
    lv_obj_add_style(label_acceleration_val_y, &label_style_value, LV_STATE_DEFAULT);
    lv_obj_add_style(label_acceleration_val_z, &label_style_value, LV_STATE_DEFAULT);
    lv_obj_add_style(label_last_failure_time_val, &label_style_value, LV_STATE_DEFAULT);

    lv_label_set_text(label_acceleration_title, "Acceleration");
    lv_label_set_text(label_gyro_title, "Gyroscope");
    lv_label_set_text_fmt(label_acceleration_val_x, "--");
    lv_label_set_text_fmt(label_acceleration_val_y, "--");
    lv_label_set_text_fmt(label_acceleration_val_z, "--");
    lv_label_set_text_fmt(label_gyro_val_x, "--");
    lv_label_set_text_fmt(label_gyro_val_y, "--");
    lv_label_set_text_fmt(label_gyro_val_z, "--");
    lv_label_set_text_fmt(label_last_failure_time_val, "--");

    lv_obj_update_layout(lv_screen_active());

    lv_coord_t coord_height_label_gyro_title = lv_obj_get_height(label_gyro_title);
    lv_coord_t coord_height_label_acceleration_title = lv_obj_get_height(label_acceleration_title);
    lv_coord_t coord_height_label_gyro_val = lv_obj_get_height(label_gyro_val_x);
    lv_coord_t coord_height_label_acceleration_val = lv_obj_get_height(label_acceleration_val_x);
    lv_coord_t coord_height_label_last_failure_time_val = lv_obj_get_height(label_last_failure_time_val);

    lv_obj_align(label_gyro_title, LV_ALIGN_CENTER, 0,
                 0 - coord_height_label_gyro_val / 2 - coord_height_label_gyro_title / 2);
    lv_obj_align(label_gyro_val_x, LV_ALIGN_CENTER, -75, 0 - coord_height_label_gyro_val / 2);
    lv_obj_align(label_gyro_val_y, LV_ALIGN_CENTER, 0, 0 - coord_height_label_gyro_val / 2);
    lv_obj_align(label_gyro_val_z, LV_ALIGN_CENTER, 75, 0 - coord_height_label_gyro_val / 2);
    lv_obj_align(label_acceleration_title, LV_ALIGN_CENTER, 0, coord_height_label_acceleration_title / 2);
    lv_obj_align(label_acceleration_val_x, LV_ALIGN_CENTER, -75, coord_height_label_acceleration_title / 2 + coord_height_label_acceleration_val / 2);
    lv_obj_align(label_acceleration_val_y, LV_ALIGN_CENTER, 0, coord_height_label_acceleration_title / 2 + coord_height_label_acceleration_val / 2);
    lv_obj_align(label_acceleration_val_z, LV_ALIGN_CENTER, 75, coord_height_label_acceleration_title / 2 + coord_height_label_acceleration_val / 2);

    lv_obj_align(label_last_failure_time_val, LV_ALIGN_CENTER, 0, coord_height_label_acceleration_title / 2 + coord_height_label_acceleration_val / 2 + coord_height_label_last_failure_time_val / 2);

    lv_obj_remove_flag(label_gyro_title, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(label_acceleration_title, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(label_gyro_val_x, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(label_gyro_val_y, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(label_gyro_val_z, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(label_acceleration_val_x, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(label_acceleration_val_y, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(label_acceleration_val_z, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(label_last_failure_time_val, LV_OBJ_FLAG_HIDDEN);

    lv_obj_update_layout(lv_screen_active());

    mpu6050_acce_value_t acceleration_value = {};
    mpu6050_gyro_value_t gyroscope_value = {};

    esp_err_t acce_ret = ESP_OK;
    esp_err_t gyro_ret = ESP_OK;
    int64_t last_failure_time = 0;

    while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        acce_ret = mpu6050_get_acce(mpu6050_handle, &acceleration_value);
        gyro_ret = mpu6050_get_gyro(mpu6050_handle, &gyroscope_value);

        if (acce_ret != ESP_OK || gyro_ret != ESP_OK) {
            last_failure_time = esp_timer_get_time() / 1000;
            continue;
        }

        lv_label_set_text_fmt(label_gyro_val_x, "%.2f", gyroscope_value.gyro_x);
        lv_label_set_text_fmt(label_gyro_val_y, "%.2f", gyroscope_value.gyro_y);
        lv_label_set_text_fmt(label_gyro_val_z, "%.2f", gyroscope_value.gyro_z);
        lv_label_set_text_fmt(label_acceleration_val_x, "%.2f", acceleration_value.acce_x);
        lv_label_set_text_fmt(label_acceleration_val_y, "%.2f", acceleration_value.acce_y);
        lv_label_set_text_fmt(label_acceleration_val_z, "%.2f", acceleration_value.acce_z);

        lv_label_set_text_fmt(label_last_failure_time_val, "%lld", last_failure_time);
    }
}

void app_main() {

    i2c_setup();
    init_spi_bus();
    init_lvgl_disp();
    init_lvgl_scene();

    esp_err_t ret = ESP_OK;

    mpu6050_handle = mpu6050_create(I2C_NUM_0, 0x68);

    vTaskDelay(40 / portTICK_PERIOD_MS);
    ret = mpu6050_config(mpu6050_handle, ACCE_FS_4G, GYRO_FS_500DPS);
    if (ret != ESP_OK){
        ESP_LOGE("mpu6050", "Failed to configure MPU6050: %d", ret);
    }

    ret = mpu6050_wake_up(mpu6050_handle);
    if (ret != ESP_OK){
        ESP_LOGE("mpu6050", "Failed to wake up MPU6050: %d", ret);
    }

    esp_lcd_panel_disp_on_off(main_lcd_panel_handle, true);
    xTaskCreate(&task_mpu6050, "task_mpu6050", 2048, NULL, 2, NULL);
}
