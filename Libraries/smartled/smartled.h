#ifndef SMARTLED_INC_H
#define SMARTLED_INC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "colorspaces.h"

enum {
    reset_width = 100, /* amount of dummy pulses */
    led_width = 24, /* bits (pulses) per LED */
    low = 16,
    high = 43
};

#define SMARTLED_BUFSIZE(LENGTH) (((reset_width) + (LENGTH) * (led_width)))

struct SmartLED {
    uint32_t length;
    uint8_t *raw_data;
    
    void (*flush)(struct SmartLED *led);
};

void SmartLED_Init(struct SmartLED *led,
                   uint32_t length,
                   uint8_t *raw_data,
                   void (*flush)(struct SmartLED *led));
void SmartLED_Clear(struct SmartLED *led);
void SmartLED_Set_RGB(struct SmartLED *led, uint32_t idx, RGB_t color);
void SmartLED_Set_HSV(struct SmartLED *led, uint32_t idx, HSV_t color);
void SmartLED_Flush(struct SmartLED *led);

#ifdef __cplusplus
}
#endif

#endif /* SMARTLED_INC_H */