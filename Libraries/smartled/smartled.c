#include "smartled.h"
#include <string.h>

void smartled_init(smartled_t *led,
                   smartled_color_format_t color_format,
                   uint8_t brightness,
                   void (*start)(smartled_t *led),
                   void (*stop)(smartled_t *led))
{
    led->color_format = color_format;
    led->brightness = brightness;
    
    led->start = start;
    led->stop = stop;
    
    smartled_clear(led);
}

void smartled_clear(smartled_t *led)
{
    uint32_t i;
    RGB_t black = rgb(0, 0, 0);
    
    for (i = 0; i < led->length; i++) {
        smartled_set_rgb(led, i, black);
    }
}

static uint8_t gamma_correction(uint8_t data)
{
    return ((uint32_t) data * (uint32_t) (data + 1)) / 256;
}

void smartled_set_rgb(smartled_t *led, uint32_t idx, RGB_t color)
{
    uint8_t *dst_ptr = led->leds_buffer + SMARTLED_BYTES_PER_LED * idx;
    
    uint8_t red =   gamma_correction(color.r);
    uint8_t green = gamma_correction(color.g);
    uint8_t blue =  gamma_correction(color.b);
    
    switch (led->color_format)
    {
        case SMARTLED_GRB:
            dst_ptr[0] = green;
            dst_ptr[1] = red;
            dst_ptr[2] = blue;
            break;
        
        default: /* RGB */
            dst_ptr[0] = red;
            dst_ptr[1] = green;
            dst_ptr[2] = blue;
            break;
    }
}

void smartled_set_hsv(smartled_t *led, uint32_t idx, HSV_t color)
{
    smartled_set_rgb(led, idx, hsv2rgb(color));
}

void smartled_flush(smartled_t *led)
{
    led->led_idx = -2;
    memset(led->pulses_buffer, 0, 2 * SMARTLED_PULSES_PER_LED);
    led->start(led);
}

void smartled_set_brightness(smartled_t *led, uint8_t brightness)
{
    led->brightness = brightness;
}

uint8_t smartled_get_brightness(smartled_t *led)
{
    return led->brightness;
}

static uint8_t smartled_brightness_scale(uint8_t color, uint8_t brightness)
{
    return ((uint32_t) color * (uint32_t) (brightness + 1)) >> 8;
}

bool smartled_next(smartled_t *led)
{
    uint8_t *dst_ptr;
    uint8_t *src_ptr;
    uint32_t color_bits;
    uint32_t bit_mask = 1 << 23;

    if (led->led_idx >= 0) {
        if (led->led_idx > led->length) {
            led->stop(led);
            return false;
        }
        
        if (led->led_idx != led->length)
        {
            dst_ptr = led->pulses_buffer + (led->led_idx % 2) * SMARTLED_PULSES_PER_LED;
            src_ptr = led->leds_buffer + SMARTLED_BYTES_PER_LED * led->led_idx;
            
            color_bits = (((uint32_t) smartled_brightness_scale(src_ptr[0], led->brightness)) << 16) |
                         (((uint32_t) smartled_brightness_scale(src_ptr[1], led->brightness)) <<  8) |
                         (((uint32_t) smartled_brightness_scale(src_ptr[2], led->brightness)) <<  0);
            
            do {
                *dst_ptr++ = (color_bits & bit_mask) ? SMARTLED_PULSE_HIGH_DURATION
                                                     : SMARTLED_PULSE_LOW_DURATION;
            } while (bit_mask >>= 1);
        }
    }
    
    (led->led_idx)++;
    return true;
}