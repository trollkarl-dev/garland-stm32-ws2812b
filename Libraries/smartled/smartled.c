#include "smartled.h"

void SmartLED_Init(struct SmartLED *led,
                   uint32_t length,
                   uint8_t *raw_data,
                   void (*flush)(struct SmartLED *led))
{
    uint32_t i;
    
    led->length = length;
    led->raw_data = raw_data;
    led->flush = flush;
    
    for (i = 0; i < reset_width; i++) {
        led->raw_data[i] = 0;
    }
    
    SmartLED_Clear(led);
}

void SmartLED_Clear(struct SmartLED *led)
{
    uint32_t i;
    
    for (i = reset_width; i < SMARTLED_BUFSIZE(led->length); i++) {
      led->raw_data[i] = low;
    }
}

void SmartLED_Set_RGB(struct SmartLED *led, uint32_t idx, RGB_t color)
{
    uint8_t c[] = {color.r, color.g, color.b};
    uint32_t i;
    const uint32_t offset = reset_width + idx * led_width;
    
    for (i = 0; i < led_width; i++) {
        led->raw_data[offset + i] = c[i/8] & (0x80 >> (i%8)) ? high : low;
    }
}

void SmartLED_Set_HSV(struct SmartLED *led, uint32_t idx, HSV_t color)
{
    SmartLED_Set_RGB(led, idx, hsv2rgb(color));
}

void SmartLED_Flush(struct SmartLED *led)
{
    led->flush(led);
}