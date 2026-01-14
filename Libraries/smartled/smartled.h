#ifndef SMARTLED_INC_H
#define SMARTLED_INC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "utils.h"
#include "colorspaces.h"

enum
{
    SMARTLED_PULSE_LOW_DURATION = 16,
    SMARTLED_PULSE_HIGH_DURATION = 43,
    SMARTLED_PULSES_PER_LED = 24,
    SMARTLED_BYTES_PER_LED = 3
};

typedef enum
{
    SMARTLED_RGB = 0,
    SMARTLED_GRB
} smartled_color_format_t;

typedef struct smartled_struct
{
    uint32_t length;
    int32_t led_idx;
    uint8_t brightness;
    uint8_t *leds_buffer;
    uint8_t *pulses_buffer;
    
    smartled_color_format_t color_format;
    
    void (*start)(struct smartled_struct *led);
    void (*stop)(struct smartled_struct *led);
} smartled_t;

#define SMARTLED_DEF(NAME, LENGTH)\
static uint8_t CONCAT(NAME, leds_buffer)[((LENGTH) * SMARTLED_BYTES_PER_LED)];\
static uint8_t CONCAT(NAME, pulses_buffer)[(SMARTLED_PULSES_PER_LED * 2)];\
static smartled_t NAME = {(LENGTH), 0, 0, CONCAT(NAME, leds_buffer), CONCAT(NAME, pulses_buffer), SMARTLED_RGB, NULL, NULL}

void smartled_init(smartled_t *led,
                   smartled_color_format_t color_format,
                   uint8_t brightness,
                   void (*start)(smartled_t *led),
                   void (*stop)(smartled_t *led));

void smartled_clear(smartled_t *led);

void smartled_set_rgb(smartled_t *led, uint32_t idx, RGB_t color);
void smartled_set_hsv(smartled_t *led, uint32_t idx, HSV_t color);

void smartled_set_brightness(smartled_t *led, uint8_t brightness);
uint8_t smartled_get_brightness(smartled_t *led);

void smartled_flush(smartled_t *led);
/* must be called in half transfer and full transfer interrupts */
bool smartled_next(smartled_t *led);

#ifdef __cplusplus
}
#endif

#endif /* SMARTLED_INC_H */