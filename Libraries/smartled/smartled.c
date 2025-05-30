#include "smartled.h"
#include <string.h>

void SmartLED_Init(struct SmartLED *led,
                   uint32_t length,
                   uint8_t brightness,
                   uint8_t *leds_buffer,
                   uint8_t *pulses_buffer,
                   void (*start)(struct SmartLED *led),
                   void (*stop)(struct SmartLED *led))
{
    led->length = length;
    led->brightness = brightness;
    
    led->leds_buffer = leds_buffer;
    led->pulses_buffer = pulses_buffer;
    
    led->start = start;
    led->stop = stop;
    
    SmartLED_Clear(led);
}

void SmartLED_Clear(struct SmartLED *led)
{
    uint32_t i;
    RGB_t black = rgb(0, 0, 0);
    
    for (i = 0; i < led->length; i++) {
        SmartLED_Set_RGB(led, i, black);
    }
}

static uint8_t gamma_correction(uint8_t data)
{
    return ((uint32_t) data * (uint32_t) (data + 1)) / 256;
}

void SmartLED_Set_RGB(struct SmartLED *led, uint32_t idx, RGB_t color)
{
    uint8_t *dst_ptr = led->leds_buffer + 3 * idx;
    
    dst_ptr[0] = gamma_correction(color.r);
    dst_ptr[1] = gamma_correction(color.g);
    dst_ptr[2] = gamma_correction(color.b);
}

void SmartLED_Set_HSV(struct SmartLED *led, uint32_t idx, HSV_t color)
{
    SmartLED_Set_RGB(led, idx, hsv2rgb(color));
}

void SmartLED_Flush(struct SmartLED *led)
{
    led->led_idx = -2;
    memset(led->pulses_buffer, 0, 2 * smartled_pulses_per_led);
    led->start(led);
}

void SmartLED_Set_Brightness(struct SmartLED *led, uint8_t brightness)
{
    led->brightness = brightness;
}

uint8_t SmartLED_Get_Brightness(struct SmartLED *led)
{
    return led->brightness;
}

static uint8_t SmartLED_Brightness_Scale(uint8_t color, uint8_t brightness)
{
    return ((uint32_t) color * (uint32_t) (brightness + 1)) >> 8;
}

bool SmartLED_Next(struct SmartLED *led)
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
            dst_ptr = led->pulses_buffer + (led->led_idx % 2) * smartled_pulses_per_led;
            src_ptr = led->leds_buffer + 3 * led->led_idx;
            
            color_bits = (((uint32_t) SmartLED_Brightness_Scale(src_ptr[0], led->brightness)) << 16) |
                         (((uint32_t) SmartLED_Brightness_Scale(src_ptr[1], led->brightness)) <<  8) |
                         (((uint32_t) SmartLED_Brightness_Scale(src_ptr[2], led->brightness)) <<  0);
            
            do {
                *dst_ptr++ = (color_bits & bit_mask) ? smartled_pulse_high
                                                     : smartled_pulse_low;
            } while (bit_mask >>= 1);
        }
    }
    
    (led->led_idx)++;
    return true;
}