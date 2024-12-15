#include "colorspaces.h"

RGB_t hsv2rgb(HSV_t hsv) {
    uint8_t high, low;
    uint8_t W, H;
    uint8_t rising, falling;
    uint8_t h_after_sixth;
    
    if (hsv.v == 0)
        return rgb(0, 0, 0);
    
    high = hsv.v * max_whiteness; /* channel with max value */    
    if (hsv.s == 0)
        return rgb(high, high, high);
    
    W = max_whiteness - hsv.s;
    low = hsv.v * W; /* channel with min value */
    rising = low;
    falling = high;
    
    h_after_sixth = hsv.h % sixth_hue;
    if (h_after_sixth > 0) { /* not at primary color? ok, h_after_sixth = 1..sixth_hue - 1 */
        uint8_t z = hsv.s * (uint8_t) (hsv.v * h_after_sixth) / sixth_hue;
        rising += z;
        falling -= z + 1; /* it's never 255, so ok */
    }
    
    H = hsv.h;
    while (H >= full_hue)
        H -= full_hue;
    
    if (H < sixth_hue) return rgb(high, rising, low);
    if (H < third_hue) return rgb(falling, high, low);
    if (H < half_hue) return rgb(low, high, rising);
    if (H < two_thirds_hue) return rgb(low, falling, high);
    if (H < five_sixths_hue) return rgb(rising, low, high);
    return rgb(high, low, falling);
}