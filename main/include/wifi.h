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

static const char *TAG = "wifi dpp-enrollee";
wifi_config_t s_dpp_wifi_config;

static int s_retry_num = 0;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_dpp_event_group;

#define DPP_CONNECTED_BIT  BIT0
#define DPP_CONNECT_FAIL_BIT     BIT1
#define DPP_AUTH_FAIL_BIT           BIT2

static int count=0;