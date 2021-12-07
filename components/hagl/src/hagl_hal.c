#include "sdkconfig.h"
#include "hagl_hal.h"
#include "ssd1283a.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <string.h>
#include <bitmap.h>
#include <hagl.h>

uint8_t buffer1[DISPLAY_WIDTH*DISPLAY_HEIGHT];

static bitmap_t fb = {
    .width = DISPLAY_WIDTH,
    .height = DISPLAY_HEIGHT,
    .depth = DISPLAY_DEPTH,
};

bitmap_t *hagl_hal_init()
{
    bitmap_init(&fb, buffer1);

    return &fb;
}

size_t hagl_hal_flush()
{
    display_update(buffer1);
    return DISPLAY_WIDTH*DISPLAY_HEIGHT;
}

void hagl_hal_put_pixel(int16_t x0, int16_t y0, color_t color)
{
    buffer1[DISPLAY_WIDTH*y0 + x0] = color;
}

color_t hagl_hal_color(uint8_t r, uint8_t g, uint8_t b) {
    return (((255-r) & 0xe0) >> 6 | ((255-g) & 0xe0) >> 2 | ((255-b) & 0xc0));
}

void hagl_hal_hline(int16_t x0, int16_t y0, uint16_t width, color_t color)
{
    for (uint16_t x = 0; x < width; x++) {
        buffer1[DISPLAY_WIDTH*y0 + x0+x] = color;
    }
}

void hagl_hal_vline(int16_t x0, int16_t y0, uint16_t height, color_t color)
{
    for (uint16_t y = 0; y < height; y++) {
        buffer1[DISPLAY_WIDTH*(y0+y) + x0] = color;
    }
}