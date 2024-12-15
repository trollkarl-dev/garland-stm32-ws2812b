#ifndef COLORSPACES_INC_H
#define COLORSPACES_INC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGB_t;

typedef struct {
    uint16_t h;
    uint8_t s;
    uint8_t v;
} HSV_t;

inline RGB_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    return (RGB_t) {r, g, b};
}

inline HSV_t hsv(uint16_t h, uint8_t s, uint8_t v) {
    return (HSV_t) {h, s, v};
}

RGB_t hsv2rgb(HSV_t color);

#ifdef __cplusplus
}
#endif

#endif /* COLORSPACES_INC_H */