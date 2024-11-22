/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/widgets/extra/meter.html#simple-meter

#include "lvgl.h"

#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
 

 
#include "lvgl__lvgl/src/extra/libs/qrcode/lv_qrcode.h"
#include "lvgl__lvgl/src/extra/libs/qrcode/qrcodegen.h"
 
#define TAG "LVGL_QR_DEMO"

 

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
