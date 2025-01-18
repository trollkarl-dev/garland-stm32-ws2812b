#ifndef SMARTLED_INC_H
#define SMARTLED_INC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "colorspaces.h"

enum {
    smartled_pulse_low = 16,
    smartled_pulse_high = 43,
    smartled_pulses_per_led = 24
};

struct SmartLED {
    uint32_t length;
    int32_t led_idx;
    uint8_t brightness;
    uint8_t *leds_buffer;
    uint8_t *pulses_buffer;
    
    void (*start)(struct SmartLED *led);
    void (*stop)(struct SmartLED *led);
};

void SmartLED_Init(struct SmartLED *led,
                   uint32_t length,
                   uint8_t brightness,
                   uint8_t *leds_buffer,
                   uint8_t *pulses_buffer,
                   void (*start)(struct SmartLED *led),
                   void (*stop)(struct SmartLED *led));

void SmartLED_Clear(struct SmartLED *led);

void SmartLED_Set_RGB(struct SmartLED *led, uint32_t idx, RGB_t color);
void SmartLED_Set_HSV(struct SmartLED *led, uint32_t idx, HSV_t color);

void SmartLED_Set_Brightness(struct SmartLED *led, uint8_t brightness);
uint8_t SmartLED_Get_Brightness(struct SmartLED *led);

void SmartLED_Flush(struct SmartLED *led);
/* must be called in half transfer and full transfer interrupts */
bool SmartLED_Next(struct SmartLED *led);

#ifdef __cplusplus
}
#endif

#endif /* SMARTLED_INC_H */