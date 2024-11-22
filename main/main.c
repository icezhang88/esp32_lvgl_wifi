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



#ifdef CONFIG_ESP_DPP_LISTEN_CHANNEL
#define EXAMPLE_DPP_LISTEN_CHANNEL_LIST     CONFIG_ESP_DPP_LISTEN_CHANNEL_LIST
#else
#define EXAMPLE_DPP_LISTEN_CHANNEL_LIST     "6"
#endif

#ifdef CONFIG_ESP_DPP_BOOTSTRAPPING_KEY
#define EXAMPLE_DPP_BOOTSTRAPPING_KEY   CONFIG_ESP_DPP_BOOTSTRAPPING_KEY
#else
#define EXAMPLE_DPP_BOOTSTRAPPING_KEY   0
#endif

#ifdef CONFIG_ESP_DPP_DEVICE_INFO
#define EXAMPLE_DPP_DEVICE_INFO      CONFIG_ESP_DPP_DEVICE_INFO
#else
#define EXAMPLE_DPP_DEVICE_INFO      0
#endif

#define CURVE_SEC256R1_PKEY_HEX_DIGITS     64

static const char *WIFI_TAG = "wifi dpp-enrollee";
wifi_config_t s_dpp_wifi_config;

static int s_retry_num = 0;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_dpp_event_group;

#define DPP_CONNECTED_BIT  BIT0
#define DPP_CONNECT_FAIL_BIT     BIT1
#define DPP_AUTH_FAIL_BIT           BIT2

static int count=0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_ERROR_CHECK(esp_supp_dpp_start_listen());
        ESP_LOGI(WIFI_TAG, "Started listening for DPP Authentication");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(WIFI_TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_dpp_event_group, DPP_CONNECT_FAIL_BIT);
        }
        ESP_LOGI(WIFI_TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(WIFI_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_dpp_event_group, DPP_CONNECTED_BIT);
    }
}

void dpp_enrollee_event_cb(esp_supp_dpp_event_t event, void *data)
{
    switch (event) {
    case ESP_SUPP_DPP_URI_READY:
        if (data != NULL) {
            // esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();

            // ESP_LOGI(TAG, "Scan below QR Code to configure the enrollee:\n");
            // esp_qrcode_generate(&cfg, (const char *)data);
        }
        break;
    case ESP_SUPP_DPP_CFG_RECVD:
        memcpy(&s_dpp_wifi_config, data, sizeof(s_dpp_wifi_config));
        esp_wifi_set_config(ESP_IF_WIFI_STA, &s_dpp_wifi_config);
        ESP_LOGI(WIFI_TAG, "DPP Authentication successful, connecting to AP : %s",
                 s_dpp_wifi_config.sta.ssid);
        s_retry_num = 0;
        esp_wifi_connect();
        break;
    case ESP_SUPP_DPP_FAIL:
        if (s_retry_num < 5) {
            ESP_LOGI(WIFI_TAG, "DPP Auth failed (Reason: %s), retry...", esp_err_to_name((int)data));
            ESP_ERROR_CHECK(esp_supp_dpp_start_listen());
            s_retry_num++;
        } else {
            xEventGroupSetBits(s_dpp_event_group, DPP_AUTH_FAIL_BIT);
        }
        break;
    default:
        break;
    }
}
 
 
esp_err_t dpp_enrollee_bootstrap(void)
{
    esp_err_t ret;
    const char *key = "303102010104201234567890abcdef1234567890abcdef1234567890abcdef1234567890abcdefa00a06082a8648ce3d030107";
    const char *device_info = "DPP Test Device";
    const char *listen_channel = "6";
                        
    /* 使用固定参数生成二维码 */
    ret = esp_supp_dpp_bootstrap_gen(listen_channel, DPP_BOOTSTRAP_QR_CODE,
                                     key, device_info);

    if (ret != ESP_OK) {
        ESP_LOGE(WIFI_TAG, "Failed to generate DPP QR Code");
    }

    return ret;
}
 
 
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    //ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        //ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
       // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
       // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
       // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        //ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
       // ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
       // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        //ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        //ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
       // ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(WIFI_TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
           
            //ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        //ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}
static esp_mqtt_client_handle_t client ;
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        //.broker.address.uri = "tcp://sensor.bodyta.com:1883",
        //.broker.address.hostname="mqtt://broker.emqx.io",
        //.broker.address.port=1883,
        .broker.address.uri="mqtt://sensor.bodyta.com:1883"
         
    };


    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    
}
void wifi_signal_monitor_task(void *pvParameters)
{
    while (1) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            ESP_LOGI(WIFI_TAG, "Current RSSI: %d dBm", ap_info.rssi);
             
             //esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);

                cJSON *root = cJSON_CreateObject();

                // 向 JSON 对象添加数据
                cJSON_AddNumberToObject(root, "rssi",ap_info.rssi);
                cJSON_AddNumberToObject(root, "count", count);
                cJSON *array = cJSON_CreateArray();
                for (int i = 0; i < 1024; i++) {
                    cJSON_AddItemToArray(array, cJSON_CreateNumber(count%8)); // 添加数字元素
                }

            // 将数组添加到 JSON 对象
             cJSON_AddItemToObject(root, "large_array", array);
                //cJSON_AddBoolToObject(root, "status", true);

                // 将 JSON 对象转换为字符串
              
                char *json_str = cJSON_PrintUnformatted(root); // 不带格式化输出
                 
                size_t json_object_size = strlen(json_str);
                esp_mqtt_client_publish(client, "/topic/qos1",json_str,json_object_size,1,0);
                //cJSON_free(root);
                free(json_str); 
                cJSON_Delete(root);  // 释放内存

        }
        count++;
        vTaskDelay(pdMS_TO_TICKS(1500)); // 每 5 秒获取一次信号强度
    }
}
void dpp_enrollee_init(void)
{
    s_dpp_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_supp_dpp_init(dpp_enrollee_event_cb));
    ESP_ERROR_CHECK(dpp_enrollee_bootstrap());
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_dpp_event_group,
                                           DPP_CONNECTED_BIT | DPP_CONNECT_FAIL_BIT | DPP_AUTH_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & DPP_CONNECTED_BIT) {
        ESP_LOGI(WIFI_TAG, "connected to ap SSID:%s password:%s",s_dpp_wifi_config.sta.ssid, s_dpp_wifi_config.sta.password);
       
        // mqtt_app_start();
      
        // xTaskCreate(wifi_signal_monitor_task, "wifi_signal_monitor_task", 4096, NULL, 5, NULL);


    } else if (bits & DPP_CONNECT_FAIL_BIT) {
        ESP_LOGI(WIFI_TAG, "Failed to connect to SSID:%s, password:%s",
                 s_dpp_wifi_config.sta.ssid, s_dpp_wifi_config.sta.password);
    } else if (bits & DPP_AUTH_FAIL_BIT) {
        ESP_LOGI(WIFI_TAG, "DPP Authentication failed after %d retries", s_retry_num);
    } else {
        ESP_LOGE(WIFI_TAG, "UNEXPECTED EVENT");
    }

    esp_supp_dpp_deinit();
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_dpp_event_group);
}

 

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
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(LCD_V_RES * LCD_H_RES * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
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
    lvgl_demo_ui(disp);
    dpp_enrollee_init();

    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(5));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
    }
}
