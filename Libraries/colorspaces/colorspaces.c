#include "colorspaces.h"

RGB_t hsv2rgb(HSV_t color) {
    int16_t hi = (color.h / 60) % 6;
    int16_t vmin = (100 - color.s) * color.v / 100;
    int16_t a = (color.v - vmin) * (color.h % 60) / 60;
    int16_t vinc = vmin + a;
    int16_t vdec = color.v - a;

    RGB_t result;

    switch (hi)
    {
        case 0: result = (RGB_t) {color.v, vinc, vmin}; break;
        case 1: result = (RGB_t) {vdec, color.v, vmin}; break;
        case 2: result = (RGB_t) {vmin, color.v, vinc}; break;
        case 3: result = (RGB_t) {vmin, vdec, color.v}; break;
        case 4: result = (RGB_t) {vinc, vmin, color.v}; break;
        case 5: result = (RGB_t) {color.v, vmin, vdec}; break;
    }

    result.r = result.r * 255 / 100;
    result.g = result.g * 255 / 100;
    result.b = result.b * 255 / 100;

    return result;
}