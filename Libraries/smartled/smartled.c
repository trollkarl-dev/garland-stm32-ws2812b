#include "smartled.h"
#include <string.h>

void SmartLED_Init(struct SmartLED *led,
                   uint32_t length,
                   uint8_t *leds_buffer,
                   uint8_t *pulses_buffer,
                   void (*start)(struct SmartLED *led),
                   void (*stop)(struct SmartLED *led))
{
    led->length = length;
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

void SmartLED_Set_RGB(struct SmartLED *led, uint32_t idx, RGB_t color)
{
    uint8_t *dst_ptr = led->leds_buffer + 3 * idx;
    
    dst_ptr[0] = color.r;
    dst_ptr[1] = color.g;
    dst_ptr[2] = color.b;

}

void SmartLED_Set_HSV(struct SmartLED *led, uint32_t idx, HSV_t color)
{
    SmartLED_Set_RGB(led, idx, hsv2rgb(color));
}

void SmartLED_Flush(struct SmartLED *led)
{
    led->led_idx = -2;
    memset(led->pulses_buffer, 0, 2 * smartled_pulses_per_led);
    /* HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t *) g->pulses_buffer, 2 * pulses_per_led); */
    led->start(led);
}

uint32_t SmartLED_Next(struct SmartLED *led)
{
    uint8_t *dst_ptr;
    uint8_t *src_ptr;
    uint32_t color_bits;
    uint32_t bit_mask = 1 << 23;

    if (led->led_idx >= 0) {
        if (led->led_idx > led->length) {
            /*HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);*/
            led->stop(led);
            return 1;
        }
        
        if (led->led_idx != led->length)
        {
            dst_ptr = led->pulses_buffer + (led->led_idx % 2) * smartled_pulses_per_led;
            src_ptr = led->leds_buffer + 3 * led->led_idx;
            
            color_bits = (((uint32_t) src_ptr[0]) << 16) |
                         (((uint32_t) src_ptr[1]) <<  8) |
                         (((uint32_t) src_ptr[2]) <<  0);
            
            do {
                *dst_ptr++ = (color_bits & bit_mask) ? smartled_pulse_high
                                                     : smartled_pulse_low;
            } while (bit_mask >>= 1);
        }
    }
    
    (led->led_idx)++;
    return 0;
}