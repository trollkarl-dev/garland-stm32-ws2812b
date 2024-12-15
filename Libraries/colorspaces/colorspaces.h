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
    uint8_t h;
    uint8_t s;
    uint8_t v;
} HSV_t;

enum {
    max_whiteness = 15,
    max_value = 17
};

enum {
    sixth_hue = 16,
    third_hue = sixth_hue * 2,
    half_hue = sixth_hue * 3,
    two_thirds_hue = sixth_hue * 4,
    five_sixths_hue = sixth_hue * 5,
    full_hue = sixth_hue * 6
};

inline RGB_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    return (RGB_t) {r, g, b};
}

inline HSV_t hsv(uint8_t h, uint8_t s, uint8_t v) {
    return (HSV_t) {h, s, v};
}

RGB_t hsv2rgb(HSV_t hsv);

#ifdef __cplusplus
}
#endif

#endif /* COLORSPACES_INC_H */