#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "lvgl.h"
#include "include/lcd_st7789v.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_dpp.h"
#include "esp_log.h"
#include "nvs_flash.h"
// #include "qrcode.h"
#include "mqtt_client.h"
#include "cJSON.h"


#include "lvgl__lvgl/src/extra/libs/qrcode/lv_qrcode.h"
#include "lvgl__lvgl/src/extra/libs/qrcode/qrcodegen.h"



 
 

static const char *TAG = "small_tv";


#define DEG_TO_RAD                  (M_PI / 180.0)

#define LCD_HOST  SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ     (60 * 1000 * 1000)
#define PIN_NUM_SCLK           13
#define PIN_NUM_MOSI           14
#define PIN_NUM_MISO           -1
#define PIN_NUM_LCD_DC         11
#define PIN_NUM_LCD_RST        15
#define PIN_NUM_LCD_CS         12
#define PIN_NUM_TOUCH_CS       15

#define PIN_NUM_BK_LIGHT       10
#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL

#define I2C_HOST               0
#define CST816T_SENSOR_ADDR    0x15
#define I2C_MASTER_SCL_IO      18
#define I2C_MASTER_SDA_IO      17
#define I2C_MASTER_FREQ_HZ     600000
#define I2C_MASTER_TIMEOUT_MS  10

#define FingerNum			   0x02
#define XposH				   0x03

#define LCD_H_RES              240
#define LCD_V_RES              280

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

#define LVGL_TICK_PERIOD_MS    2

// extern void lvgl_demo_ui(lv_disp_t *disp);


uint8_t cst816t_read_len(uint16_t reg_addr,uint8_t *data,uint8_t len);
static esp_err_t  testtouch(uint8_t *touch_points_num);
esp_err_t get_coordinates(uint16_t *x, uint16_t *y);
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
static void lvgl_port_update_callback(lv_disp_drv_t *drv);
static void lvgl_touch_cb(lv_indev_drv_t * drv, lv_indev_data_t * data);
static void increase_lvgl_tick(void *arg);

uint8_t cst816t_read_len(uint16_t reg_addr,uint8_t * data,uint8_t len)
{
    uint8_t res=0;
    res = i2c_master_write_read_device(I2C_HOST, CST816T_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return res;
}

static esp_err_t testtouch(uint8_t *touch_points_num)
{
    uint8_t res=0;
    res = cst816t_read_len(FingerNum, touch_points_num,1);

    return res; 
}

esp_err_t get_coordinates(uint16_t *x, uint16_t *y)
{
    uint8_t data[4];
    int a, b;
    cst816t_read_len(XposH, data,4);
    a = ((data[0] & 0x0f) << 8) | data[1];
    b = ((data[2] & 0x0f) << 8) | data[3];
    if ((a<=240) && (b<=280))
    {
        *x = a;
        *y = b;
    }
    return ESP_OK;
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1 + 20, offsetx2 + 1, offsety2 + 21, color_map);
        break;
    case LV_DISP_ROT_90:
        esp_lcd_panel_draw_bitmap(panel_handle, offsetx1 + 20, offsety1, offsetx2 + 21, offsety2 + 1, color_map);
        break;
    case LV_DISP_ROT_180:
        esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1 + 20, offsetx2 + 1, offsety2 + 21, color_map);
        break;
    case LV_DISP_ROT_270:
        esp_lcd_panel_draw_bitmap(panel_handle, offsetx1 + 20, offsety1, offsetx2 + 21, offsety2 + 1, color_map);
        break;
    }
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
static void lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    }
}

static void lvgl_touch_cb(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
    uint16_t touchpad_x = 0;
    uint16_t touchpad_y = 0;
    uint8_t touch_points_num=0;
    testtouch(&touch_points_num);

    if (touch_points_num == 1)
    {
        get_coordinates(&touchpad_x, &touchpad_y);
        if ((touchpad_x != 0) && (touchpad_y != 0))
        {
            data->point.x = touchpad_x;
            data->point.y = touchpad_y;
        }

        data->state = LV_INDEV_STATE_PRESSED;
    } else 
    {
        //printf("else callback");
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}
 

void test()
{
    const char *data = "DPP:C:81/6;M:34:85:18:90:82:80;I:DPP;K:MDkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDIgACRxw+dYxJBChbun5TEY7Q9SSt6wdX0lvS+Oew1236cUw=;;";

    /*Create a 100x100 QR code*/
    lv_obj_t *qr = lv_qrcode_create(lv_scr_act(), 200, lv_color_hex3(0x33f), lv_color_hex3(0xeef));
    lv_obj_align(qr, LV_ALIGN_CENTER, 0, 0);
    /*Set data*/
    lv_qrcode_update(qr, data, strlen(data));
}

void lvgl_demo_ui(lv_disp_t *disp)
{
   

    lv_obj_t *scr = lv_disp_get_scr_act(disp);

 
    test();
     
}
static void ta_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * ta = lv_event_get_target(e);
    lv_obj_t * kb = lv_event_get_user_data(e);
    if(code == LV_EVENT_FOCUSED) {
        lv_keyboard_set_textarea(kb, ta);
        lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }

    if(code == LV_EVENT_DEFOCUSED) {
        lv_keyboard_set_textarea(kb, NULL);
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }
}

void lv_example_keyboard_1(void)
{
    /*Create a keyboard to use it with an of the text areas*/
    lv_obj_t * kb = lv_keyboard_create(lv_scr_act());

    /*Create a text area. The keyboard will write here*/
    lv_obj_t * ta;
    ta = lv_textarea_create(lv_scr_act());
    lv_obj_align(ta, LV_ALIGN_TOP_MID, 10, 10);
    lv_obj_add_event_cb(ta, ta_event_cb, LV_EVENT_ALL, kb);
    lv_textarea_set_placeholder_text(ta, "Hello");
    lv_obj_set_size(ta, 140, 80);

    // ta = lv_textarea_create(lv_scr_act());
    // lv_obj_align(ta, LV_ALIGN_TOP_RIGHT, -10, 10);
    // lv_obj_add_event_cb(ta, ta_event_cb, LV_EVENT_ALL, kb);
    // lv_obj_set_size(ta, 140, 80);

    lv_keyboard_set_textarea(kb, NULL);
    lv_keyboard_set_mode(kb,LV_KEYBOARD_MODE_NUMBER);
}
void app_main(void)
{

  esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    //dpp_enrollee_init();
     //xTaskCreate(dpp_enrollee_init, "dpp_enrollee_init", 12288, NULL, 5, NULL);

    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t spi_buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_V_RES * LCD_H_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &spi_buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_LCD_DC,
        .cs_gpio_num = PIN_NUM_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install ST7789V panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 16,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789v(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize I2C bus");

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config((i2c_port_t)I2C_HOST, &conf));
    ESP_ERROR_CHECK(i2c_driver_install((i2c_port_t)I2C_HOST, conf.mode, 0, 0, 0));

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(LCD_V_RES * LCD_H_RES * sizeof(lv_color_t), MALLOC_CAP_DMA);
    //assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(LCD_V_RES * LCD_H_RES * sizeof(lv_color_t), MALLOC_CAP_DMA);
    //assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_V_RES * LCD_H_RES);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.drv_update_cb = lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));



    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = lvgl_touch_cb;
    //indev_drv.user_data = tp;

    lv_indev_drv_register(&indev_drv);

    ESP_LOGI(TAG, "Display LVGL");
    //lvgl_demo_ui(disp);
    lv_example_keyboard_1();

    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(5));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
    }
}
