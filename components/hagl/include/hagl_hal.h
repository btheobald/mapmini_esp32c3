#include <stdint.h>
#include <stdbool.h>
#include <bitmap.h>
#include "driver/spi_master.h"
#include "driver/spi_common.h"

typedef uint8_t color_t;

/* HAL must provide display dimensions and depth. */
#define DISPLAY_WIDTH   (130)
#define DISPLAY_HEIGHT  (130)
#define DISPLAY_DEPTH   (8)

/* These are the optional features this HAL provides. */
#define HAGL_HAS_HAL_INIT
#define HAGL_HAS_HAL_FLUSH
#define HAGL_HAS_HAL_COLOR
#define HAGL_HAS_HAL_HLINE
#define HAGL_HAS_HAL_VLINE

/**
 * @brief Draw a single pixel
 *
 * This is the only mandatory function HAL must provide.
 *
 * @param x0 X coordinate
 * @param y0 Y coorginate
 * @param color color
 */
void hagl_hal_put_pixel(int16_t x0, int16_t y0, color_t color);

/**
 * @brief Initialize the HAL
 *
 * Initialises all hardware and possible memory buffers needed
 * to draw and display an image. If HAL uses double or triple
 * buffering should return a pointer to current back buffer.
 * This HAL does not use buffering so it returns NULL instead.
 *
 * @return pointer to bitmap_t or NULL
 */
bitmap_t *hagl_hal_init();

/**
 * @brief Output the current frame
 *
 * This is used for HAL implementations which do not display
 * the drawn pixels automatically. Call this function always when
 * you have finished rendering.
 */
size_t hagl_hal_flush();

/**
 * Draw a horizontal line
 *
 * @param x0 X coordinate
 * @param y0 Y coorginate
 * @param w width of the line
 */
void hagl_hal_hline(int16_t x0, int16_t y0, uint16_t w, color_t color);

/**
 * Draw a vertical line
 *
 * @param x0 X coordinate
 * @param y0 Y coorginate
 * @param h height of the line
 */
void hagl_hal_vline(int16_t x0, int16_t y0, uint16_t h, color_t color);

//void hagl_hal_thick_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t t, color_t color);

color_t hagl_hal_color(uint8_t r, uint8_t g, uint8_t b);