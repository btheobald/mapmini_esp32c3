#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <wchar.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <soc/rtc_wdt.h>

#include "sdkconfig.h"
#include "hagl_hal.h"
#include "bitmap.h"
#include "hagl.h"
#include "font6x9.h"
#include "fps.h"
#include "aps.h"
#include "spi_bus.h"
#include "ssd1283a.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "spi_bus.h"
#include "sdkconfig.h"
#include "sd.h"
#include "aa.h"
#include "map.h"
#include "rgb332.h"
#include "memory.h"

static const char *TAG = "main";

static SemaphoreHandle_t mutex;
static float fb_fps;
static float fx_fps;
static uint16_t current_demo = 0;
static bitmap_t *bb;
static uint32_t drawn = 0;

const char mount_point[] = "/sdcard";

void framebuffer_task(void *params)
{
    TickType_t last;
    const TickType_t frequency = 1000 / 15 / portTICK_RATE_MS;

    last = xTaskGetTickCount();

    while (1) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        hagl_flush();
        xSemaphoreGive(mutex);
        fb_fps = fps();
        vTaskDelayUntil(&last, frequency);
    }

    vTaskDelete(NULL);
}

void app_main()
{
    ESP_LOGI(TAG, "SDK version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Heap when starting: %d", esp_get_free_heap_size());

    spi_bus_init();
    ESP_LOGI(TAG, "SPI Init");

    lcd_init();
    ESP_LOGI(TAG, "LCD Init");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI_HOST;
    host.max_freq_khz = 40000;
    sdspi_device_config_t devcfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    devcfg.gpio_cs = PIN_NUM_CS_SD;
    devcfg.host_id = host.slot;

    sdmmc_card_t* card;
    esp_vfs_fat_sdmmc_mount_config_t mount_config;

    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;

    gpio_set_pull_mode(2, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(7, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
    gpio_set_pull_mode(6, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    gpio_set_pull_mode(10, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode(5, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes

    esp_err_t ret;

    do {
        ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &devcfg, &mount_config, &card);
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        vTaskDelay(100);
    } while(ret != ESP_OK);

    ESP_LOGI(TAG, "SD Init");
    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
    
    bb = hagl_init();
    if (bb) {
        ESP_LOGI(TAG, "Back buffer: %dx%dx%d", bb->width, bb->height, bb->depth);
    }

    hagl_set_clip_window(1,1,DISPLAY_WIDTH-1,DISPLAY_HEIGHT-1);

    ESP_LOGI(TAG, "Heap after HAGL init: %d", esp_get_free_heap_size());

    mutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(framebuffer_task, "Framebuffer", 8192, NULL, 1, NULL, 0);

    float rot = 0.0;

    way_prop* way_list_ptr = NULL;

    arena_t a0;
    arena_init(&a0, ARENA_DEFAULT_SIZE);

    uint8_t wd = load_map(&a0, "/sdcard/scotland_roads.map", &way_list_ptr, 8044, 5108, 14, 0, 0, 0xFFFF, 0, 128);   

    ESP_LOGI(TAG, "Loaded in %d ways", wd);
    ESP_LOGI(TAG, "Allocated: %d", a0.current);

    while(1) {

        vTaskDelay(1);
        xSemaphoreTake(mutex, portMAX_DELAY);
        hagl_clear_screen();
        
        for(int w = 0; w < wd; w++) {
            g_draw_way(way_list_ptr+w, 0, 0, 0, 0, rot, 128);
        }

        uint16_t compass_len = 10;
        uint16_t border = 3;
        uint16_t compass_x = compass_len+border;
        uint16_t compass_y = DISPLAY_HEIGHT-compass_len-border;
        uint8_t compass_lw = 2;

        hagl_fill_circle(compass_x, compass_y, compass_len+2, hagl_color(0,0,0));
        hagl_draw_circle(compass_x, compass_y, compass_len+2, hagl_color(255,255,255));

        draw_line_antialias(compass_x, compass_y, compass_x-(compass_len-1)*sin(-rot), compass_y-(compass_len-1)*cos(-rot),  rgb332(255,0,0));
        draw_line_antialias(compass_x, compass_y, compass_x, compass_y-compass_len,  rgb332(0,255,0));

        xSemaphoreGive(mutex);
        //ESP_LOGI(TAG,"Loaded %d bytes from map", hp);
        
        rot += M_PI/157;
        
    }
}