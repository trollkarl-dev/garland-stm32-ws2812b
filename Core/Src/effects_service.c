#include "effects_service.h"

#include <string.h>
#include <math.h>

#include "colorspaces.h"
#include "smartled.h"
#include "perlin.h"
#include "utils.h"

enum {
    full_hue = 360,
    max_whiteness = 100,
    max_value = 100
};

static inline int32_t effects_service_abs(int32_t a)
{
    return a < 0 ? -a : a;
}

static uint32_t lfsr_seed = 0xDEADBEEF;

static void lfsr_srand(uint32_t seed)
{
	lfsr_seed = seed;
}

static uint32_t lfsr_rand(void)
{
	lfsr_seed = (lfsr_seed << 1) | (((lfsr_seed >> 1) & 1) ^
			                ((lfsr_seed >> 5) & 1) ^
					((lfsr_seed >> 6) & 1) ^
					((lfsr_seed >> 31) & 1));

	return lfsr_seed;
}

/***************************************************************************/
/*                                 Effects                                 */
/***************************************************************************/

enum { effects_shared_data_size = 64 };

static volatile uint32_t current_effect_idx = 0;
static uint8_t effects_shared_data[effects_shared_data_size];

typedef struct {
    void (*setup)(void *raw_data, const void *initial);
    void (*loop)(smartled_t * leds, void *raw_data);
    const void *initial;
} effect_t;

/*                             Running rainbow                             */

typedef struct {
    uint32_t hue;
} running_rainbow_data_t;

STATIC_ASSERT(sizeof(running_rainbow_data_t) <= effects_shared_data_size);

static void running_rainbow_setup(void *raw_data, const void *initial)
{
    running_rainbow_data_t * const data = (running_rainbow_data_t *) raw_data;

    data->hue = 0;
}

static void running_rainbow_loop(smartled_t *leds, void *raw_data)
{
    uint32_t i;
    running_rainbow_data_t * const data = (running_rainbow_data_t *) raw_data;
    
    for (i = 0; i < leds->length; i++)
    {
        smartled_set_hsv(leds, i, hsv((data->hue+i) % full_hue, max_whiteness, max_value));
    }
        
    data->hue = (data->hue + 6) % full_hue;
}

/*                               All rainbow                               */

typedef struct {
    uint32_t hue;
} all_rainbow_data_t;

STATIC_ASSERT(sizeof(all_rainbow_data_t) <= effects_shared_data_size);

static void all_rainbow_setup(void *raw_data, const void *initial)
{
    all_rainbow_data_t * const data = (all_rainbow_data_t *) raw_data;

    data->hue = 0;
}

static void all_rainbow_loop(smartled_t *leds, void *raw_data)
{
    uint32_t i;
    all_rainbow_data_t * const data = (all_rainbow_data_t *) raw_data;
    
    for (i = 0; i < leds->length; i++)
        smartled_set_hsv(leds, i, hsv(data->hue, max_whiteness, max_value));
        
    data->hue = (data->hue + 1) % full_hue;
}

/*                             Perlin's noise                              */

typedef struct {
    RGB_t col[2];
    uint32_t offset;
} perlin_data_t;

STATIC_ASSERT(sizeof(perlin_data_t) <= effects_shared_data_size);

static void perlin_setup(void *raw_data, const void *initial)
{
    memcpy(raw_data, initial, sizeof(perlin_data_t));
}

/* Fixed-point sigmoid alternative.
 * Accepts x from 0 to 1024. Returns coefficient from 0 to 1024. */
static int32_t sigmoid_fixed(int32_t x)
{
    /* 0.15f in 1024 scale is ~154 */
    const int32_t threshold = 154; 
    
    /* Integer equivalent of fabs(x - 0.5f) */
    int32_t diff = x - 512;
    if (diff < 0) diff = -diff;
    
    if (diff < threshold)
    {
        /* Original: 0.5f * (1.0f + (x - 0.5f) / threshold)
         * In integers: 512 + (512 * (x - 512)) / threshold */
        return 512 + (512 * (x - 512)) / threshold;
    }
    
    return (x < 512) ? 0 : 1024;
}

/* Fixed-point linear interpolation for colors.
 * Accepts start color, end color, and coef from 0 to 1024. */
static uint8_t lerp_color_fixed(uint8_t c1, uint8_t c2, int32_t coef)
{
    /* Formula: a + (b - a) * coef / 1024
     * Division by 1024 is replaced with a bitwise shift >> 10 */
    return (uint8_t)(c1 + (((int32_t)(c2 - c1) * coef) >> 10));
}

static void perlin_loop(smartled_t *leds, void *raw_data)
{
    perlin_data_t * const data = (perlin_data_t *) raw_data;
    uint32_t i;
    const uint32_t vertical_size = 1048576; /* must be a power of two */
    
    for (i = 0; i < leds->length; i++)
    {
        /* Get noise in the range of [-1024 ... 1024] */
        int32_t coef = perlin(i, data->offset, leds->length, vertical_size, 10);
        
        /* Map the range from [-1024 ... 1024] to [0 ... 1024] using shift instead of / 2 */
        coef = (coef + 1024) >> 1;
        
        /* Run through sigmoid, resulting in [0 ... 1024] */
        coef = sigmoid_fixed(coef);

        /* Calculate new RGB channels entirely using integer arithmetic */
        RGB_t color = (RGB_t)
        {
            lerp_color_fixed(data->col[0].r, data->col[1].r, coef),
            lerp_color_fixed(data->col[0].g, data->col[1].g, coef),
            lerp_color_fixed(data->col[0].b, data->col[1].b, coef)
        };
        
        smartled_set_rgb(leds, i, color);
    }
    
    data->offset = (data->offset + 1) & (vertical_size - 1);
}

/*                                  Running shadow                         */

typedef struct {
    uint32_t counter;
} running_shadow_data_t;

STATIC_ASSERT(sizeof(running_shadow_data_t) <= effects_shared_data_size);

static void running_shadow_setup(void *raw_data, const void *initial)
{
    running_shadow_data_t * const data = (running_shadow_data_t *) raw_data;
    data->counter = 0;
}

static void running_shadow_loop(smartled_t *leds, void *raw_data)
{
    uint32_t i;
    running_shadow_data_t * const data = (running_shadow_data_t *) raw_data;

    for (i = 0; i < leds->length; i++)
    {
        int val = effects_service_abs((int) data->counter - (int) i);
        int hue, value;

        val = val > leds->length / 2 ? leds->length - val : val;

        /*hue = (int) ((float) i / leds->length * full_hue);*/
        hue = ((i << 16) / leds->length * full_hue) >> 16;
        /*value = (int) ((float) val / (leds->length / 2) * max_value);*/
        value = ((val << 16) / (leds->length / 2) * max_value) >> 16;

        smartled_set_hsv(leds, i, hsv(hue, max_whiteness, value));
    }

    data->counter = (data->counter + 1) % leds->length;
}

/*                      Running rainbow from middle                        */

typedef struct {
    uint32_t counter;
} running_rainbow_from_middle_data_t;

STATIC_ASSERT(sizeof(running_rainbow_from_middle_data_t) <= effects_shared_data_size);

static void running_rainbow_from_middle_setup(void *raw_data, const void *initial)
{
    running_rainbow_from_middle_data_t * const data = (running_rainbow_from_middle_data_t *) raw_data;
    data->counter = 0;
}

static void running_rainbow_from_middle_loop(smartled_t *leds, void *raw_data)
{
    running_rainbow_from_middle_data_t * const data = (running_rainbow_from_middle_data_t *) raw_data;
    uint32_t i;

    for (i = 0; i < leds->length; i++)
    {
	    /*int hue = (float) ((ABS((int) i - (int) (leds->length / 2)) + data->counter) % leds->length) / leds->length * full_hue;*/
	    int32_t hue = ((((effects_service_abs((int32_t) i - (int32_t) (leds->length / 2)) + data->counter) % leds->length) << 16) / leds->length * full_hue) >> 16;
	    smartled_set_hsv(leds, i, hsv(hue, max_whiteness, max_value));
    }

    data->counter = (data->counter + 1) % leds->length;
}

/*                             Stepping colors                              */

typedef struct {
    uint16_t counter;
    uint16_t hue;
    uint16_t direction;
} stepping_colors_data_t;

STATIC_ASSERT(sizeof(stepping_colors_data_t) <= effects_shared_data_size);

static void stepping_colors_setup(void *raw_data, const void *initial)
{
    stepping_colors_data_t * const data = (stepping_colors_data_t *) raw_data;

    data->counter = 0;
    data->hue = 0;
    data->direction = 0;
}

static void stepping_colors_loop(smartled_t *leds, void *raw_data)
{
    stepping_colors_data_t * const data = (stepping_colors_data_t *) raw_data;
    int32_t i;
    int32_t length = (int32_t)leds->length;
    int32_t counter = (int32_t)data->counter;

    if (data->direction)
    {
        for (i = length - 1; i >= 0; i--)
        {
            HSV_t color = i > (length - 1 - counter)
                          ? hsv(data->hue, max_whiteness, max_whiteness)
                          : hsv(0, 0, 0);
            smartled_set_hsv(leds, (uint32_t)i, color);
        }
    }
    else
    {
        for (i = 0; i < length; i++)
        {
            HSV_t color = i <= counter
                          ? hsv(data->hue, max_whiteness, max_whiteness)
                          : hsv(0, 0, 0);
            smartled_set_hsv(leds, (uint32_t)i, color);
        }
    }

    data->counter = (data->counter + 1) % leds->length;

    if (data->counter == 0)
    {
        data->direction ^= 1;
        data->hue = (data->hue + full_hue / 12) % full_hue;
    }
}

/*                                  Comet                                  */

typedef struct {
    uint16_t head_pos;
    uint16_t hue;
} comet_data_t;

STATIC_ASSERT(sizeof(comet_data_t) <= effects_shared_data_size);

static void comet_setup(void *raw_data, const void *initial) {
    comet_data_t * const data = (comet_data_t *) raw_data;
    data->head_pos = 0;
    data->hue = 0;
}

static void comet_loop(smartled_t *leds, void *raw_data) {
    comet_data_t * const data = (comet_data_t *) raw_data;
    int32_t i;
    int32_t length = (int32_t)leds->length;

    for (i = 0; i < length; i++) {
        int32_t dist = (int32_t)data->head_pos - i;
        if (dist < 0) dist += length;

        int32_t value;
        if (dist == 0) {
            value = max_value;
        } else if (dist < 8) {
            value = ((8 - dist) * max_value) >> 3;
        } else {
            value = 0;
        }

        smartled_set_hsv(leds, (uint32_t)i, hsv(data->hue, max_whiteness, value));
    }

    data->head_pos = (data->head_pos + 1) % leds->length;
    data->hue = (data->hue + 1) % full_hue;
}

/*                                Heartbeat                                */

typedef struct {
    uint16_t lut_index;
    uint16_t hue;
} heartbeat_data_t;

STATIC_ASSERT(sizeof(heartbeat_data_t) <= effects_shared_data_size);

static void heartbeat_setup(void *raw_data, const void *initial) {
    heartbeat_data_t * const data = (heartbeat_data_t *) raw_data;
    data->lut_index = 0;
    data->hue = 0;
}

static void heartbeat_loop(smartled_t *leds, void *raw_data) {
    heartbeat_data_t * const data = (heartbeat_data_t *) raw_data;
    uint32_t i;

    static const uint8_t heartbeat_lut[32] = {
        0, 40, 255, 150, 80, 200, 255, 180, 120, 70, 40, 20, 10, 5, 2, 0,
        0, 0,  0,   0,   0,  0,   0,   0,   0,   0,  0,  0,  0, 0, 0, 0
    };

    uint8_t current_brightness = heartbeat_lut[data->lut_index];
    int32_t scaled_value = ((int32_t)max_value * current_brightness) >> 8;

    for (i = 0; i < leds->length; i++) {
        smartled_set_hsv(leds, i, hsv(data->hue, max_whiteness, scaled_value));
    }

    data->lut_index = (data->lut_index + 1) & 31;
    if (data->lut_index == 0) {
        data->hue = (data->hue + full_hue / 6) % full_hue;
    }
}

/*                              Blade Spinner                              */

typedef struct {
    uint16_t angle;
    uint16_t hue;
} spinner_data_t;

STATIC_ASSERT(sizeof(spinner_data_t) <= effects_shared_data_size);

static void spinner_setup(void *raw_data, const void *initial) {
    spinner_data_t * const data = (spinner_data_t *) raw_data;
    data->angle = 0;
    data->hue = 0;
}

static void spinner_loop(smartled_t *leds, void *raw_data) {
    spinner_data_t * const data = (spinner_data_t *) raw_data;
    int32_t i;
    int32_t length = (int32_t)leds->length;
    int32_t half_length = length / 2;

    for (i = 0; i < length; i++) {
        int32_t dist = ((int32_t)data->angle - i) % half_length;
        if (dist < 0) dist += half_length;

        int32_t value = 0;
        if (dist == 0) value = max_value;
        else if (dist == 1) value = max_value >> 1;
        else if (dist == 2) value = max_value >> 3;

        smartled_set_hsv(leds, (uint32_t)i, hsv(data->hue, max_whiteness, value));
    }

    data->angle = (data->angle + 1) % leds->length;
    data->hue = (data->hue + 2) % full_hue;
}

/*                            Comets Collision                             */

typedef struct {
    uint16_t hue1;
    uint16_t hue2;
} comets_collision_init_t;

typedef struct {
    uint16_t hue1;
    uint16_t hue2;
    uint16_t pos1;
    uint16_t pos2;
} comets_collision_data_t;

STATIC_ASSERT(sizeof(comets_collision_data_t) <= effects_shared_data_size);

static void comets_collision_setup(void *raw_data, const void *initial)
{
    comets_collision_data_t * const data = (comets_collision_data_t *) raw_data;
    const comets_collision_init_t * const init = (const comets_collision_init_t *) initial;

    if (init != NULL) {
        data->hue1 = init->hue1;
        data->hue2 = init->hue2;
    } else {
        data->hue1 = 0;
        data->hue2 = (full_hue * 2) / 3;
    }

    data->pos1 = 0;
    data->pos2 = 0;
}

static void comets_collision_loop(smartled_t *leds, void *raw_data)
{
    comets_collision_data_t * const data = (comets_collision_data_t *) raw_data;
    int32_t i;
    int32_t length = (int32_t)leds->length;

    for (i = 0; i < length; i++)
    {
        int32_t dist1 = (int32_t)data->pos1 - i;
        if (dist1 < 0) dist1 += length;

        int32_t val1 = 0;
        if (dist1 == 0) val1 = max_value;
        else if (dist1 < 6) val1 = ((6 - dist1) * max_value) >> 3;

        int32_t dist2 = i - (int32_t)data->pos2;
        if (dist2 < 0) dist2 += length;

        int32_t val2 = 0;
        if (dist2 == 0) val2 = max_value;
        else if (dist2 < 6) val2 = ((6 - dist2) * max_value) >> 3;

        if (val1 > 0 && val2 > 0)
        {
            int32_t total_val = val1 + val2;
            int32_t coef = (val1 << 8) / total_val;

            int32_t mixed_hue = data->hue2 + (((data->hue1 - data->hue2) * coef) >> 8);
            mixed_hue = (mixed_hue + full_hue) % full_hue;

            int32_t mixed_val = val1 > val2 ? val1 : val2;

            if (val1 == max_value && val2 == max_value)
            {
                smartled_set_hsv(leds, (uint32_t)i, hsv(mixed_hue, 0, mixed_val));
            }
            else
            {
                smartled_set_hsv(leds, (uint32_t)i, hsv(mixed_hue, max_whiteness, mixed_val));
            }
        }
        else if (val1 > 0)
        {
            smartled_set_hsv(leds, (uint32_t)i, hsv(data->hue1, max_whiteness, val1));
        }
        else if (val2 > 0)
        {
            smartled_set_hsv(leds, (uint32_t)i, hsv(data->hue2, max_whiteness, val2));
        }
        else
        {
            smartled_set_hsv(leds, (uint32_t)i, hsv(0, 0, 0));
        }
    }

    data->pos1 = (data->pos1 + 1) % leds->length;

    if (data->pos2 == 0) {
        data->pos2 = leds->length - 1;
    } else {
        data->pos2--;
    }
}

/*                                  Fire                                   */

typedef struct {
    uint32_t rand_seed;
} fire_data_t;

STATIC_ASSERT(sizeof(fire_data_t) <= effects_shared_data_size);

static void fire_setup(void *raw_data, const void *initial) {
    fire_data_t * const data = (fire_data_t *) raw_data;
    data->rand_seed = 54321;
}

static void fire_loop(smartled_t *leds, void *raw_data) {
    fire_data_t * const data = (fire_data_t *) raw_data;
    int32_t i;
    int32_t length = (int32_t)leds->length;

    /* The bottom of the ring (led 11) is the hearth, the top (led 0) is the tip of the flame */
    for (i = 0; i < length; i++) {
        /* Calculate distance from the bottom hearth */
        int32_t dist = effects_service_abs(i - 11);

        /* Fast Linear Congruential Generator for per-led noise */
        data->rand_seed = data->rand_seed * 1103515245 + 12345;
        int32_t flicker = (data->rand_seed >> 16) & 63; /* Noise range [0..63] */

        /* Base brightness drops moving away from the bottom center + add noise flicker */
        int32_t base_val = max_value - ((dist * max_value) / 11);
        int32_t value = base_val - flicker;
        if (value < 0) value = 0;

        /* Color correction: orange-yellow at the hearth, shifting to pure red at the tips */
        int32_t hue = (dist * 25) / 11; 

        smartled_set_hsv(leds, (uint32_t)i, hsv(hue, max_whiteness, value));
    }
}

/*                                  Radar                                   */

typedef struct {
    uint16_t scan_pos;
} radar_data_t;

STATIC_ASSERT(sizeof(radar_data_t) <= effects_shared_data_size);

static void radar_setup(void *raw_data, const void *initial) {
    radar_data_t * const data = (radar_data_t *) raw_data;
    data->scan_pos = 0;
}

static void radar_loop(smartled_t *leds, void *raw_data) {
    radar_data_t * const data = (radar_data_t *) raw_data;
    int32_t i;
    int32_t length = (int32_t)leds->length;

    /* Basic configuration for green sweep tint and red target alert */
    const int32_t radar_hue = 120; 
    const int32_t alert_hue = 0;   

    for (i = 0; i < length; i++) {
        /* Calculate trailing fade tail behind the radar sweeping head */
        int32_t dist = (int32_t)data->scan_pos - i;
        if (dist < 0) dist += length;

        int32_t radar_val = 0;
        if (dist == 0) radar_val = max_value; 
        else if (dist < 10) radar_val = ((10 - dist) * (max_value >> 1)) / 10; 
        else radar_val = max_value >> 4; /* Ambient dim background illumination */

        /* Check if the current led falls into static target positions (indexes 4 and 15) */
        if (i == 4 || i == 15) {
            /* If the scanning head currently sweeps over or just passed the target */
            if (dist < 4) {
                /* Target ping overrides background radar logic with high intensity alert */
                int32_t alert_val = max_value - (dist * (max_value >> 2));
                smartled_set_hsv(leds, (uint32_t)i, hsv(alert_hue, max_whiteness, alert_val));
                continue;
            }
        }

        /* Default background radar sweep rendering */
        smartled_set_hsv(leds, (uint32_t)i, hsv(radar_hue, max_whiteness, radar_val));
    }

    data->scan_pos = (data->scan_pos + 1) % leds->length;
}

/*                                 VU Meter                                  */

typedef struct {
    uint8_t current_level; /* Dynamic audio level from 0 to 255 */
} vu_meter_init_t;

typedef struct {
    uint32_t rand_seed;
    uint32_t level;        /* Fixed-point smooth level */
    uint16_t hue_base;
} vu_meter_data_t;

STATIC_ASSERT(sizeof(vu_meter_data_t) <= effects_shared_data_size);

static void vu_meter_setup(void *raw_data, const void *initial) {
    vu_meter_data_t * const data = (vu_meter_data_t *) raw_data;
    data->rand_seed = 98765;
    data->level = 0;
    data->hue_base = 0;
}

static void vu_meter_loop(smartled_t *leds, void *raw_data) {
    vu_meter_data_t * const data = (vu_meter_data_t *) raw_data;
    const vu_meter_init_t * const init = (const vu_meter_init_t *) leds; /* Dummy mapping or direct cast if passed via leds wrapper */
    int32_t i;
    int32_t length = (int32_t)leds->length;
    int32_t target_level = 0;

    /* Generate pseudo-random audio bursts if external initial data is absent */
    data->rand_seed = data->rand_seed * 1103515245 + 12345;
    if ((data->rand_seed % 100) < 15) {
        target_level = (data->rand_seed >> 16) & 255;
    }

    /* Smooth the level transitions using low-pass filter approximation */
    data->level = data->level + (((target_level - (int32_t)data->level) * 40) >> 8);

    /* Map 0..255 scale down to half-ring led length (0 to 11 leds max height) */
    int32_t max_active_leds = (int32_t)((data->level * 11) >> 8);

    for (i = 0; i < length; i++) {
        /* Calculate distance from the bottom origin point (led 11) */
        int32_t dist = effects_service_abs(i - 11);

        if (dist <= max_active_leds) {
            /* Active equalizer bars: gradient from green at the bottom to red at the peak */
            int32_t hue = (data->hue_base + (dist * (full_hue / 4)) / 11) % full_hue;
            smartled_set_hsv(leds, (uint32_t)i, hsv(hue, max_whiteness, max_value));
        } else {
            /* Ambient background off-state */
            smartled_set_hsv(leds, (uint32_t)i, hsv(0, 0, 0));
        }
    }
    
    data->hue_base = (data->hue_base + 1) % full_hue;
}

/*                                 Vortex                                  */

typedef struct {
    uint16_t phase;
    uint16_t color_wheel;
} vortex_data_t;

STATIC_ASSERT(sizeof(vortex_data_t) <= effects_shared_data_size);

static void vortex_setup(void *raw_data, const void *initial) {
    vortex_data_t * const data = (vortex_data_t *) raw_data;
    data->phase = 0;
    data->color_wheel = 0;
}

static void vortex_loop(smartled_t *leds, void *raw_data) {
    vortex_data_t * const data = (vortex_data_t *) raw_data;
    int32_t i;
    int32_t length = (int32_t)leds->length;

    for (i = 0; i < length; i++) {
        /* Calculate offset relative to the moving global phase */
        int32_t rel_pos = (i - (int32_t)data->phase) % (length / 3);
        if (rel_pos < 0) rel_pos += (length / 3);

        int32_t value = 0;
        /* Generate sharp peak nodes with exponential tail decay */
        if (rel_pos == 0) value = max_value;
        else if (rel_pos == 1) value = max_value >> 1;
        else if (rel_pos == 2) value = max_value >> 3;

        /* Add a secondary slow breathing global brightness wave to simulate depth */
        int32_t wave = (i + data->phase) & 15;
        value = (value * (16 + wave)) >> 5;

        int32_t hue = (data->color_wheel + (i * 4)) % full_hue;
        smartled_set_hsv(leds, (uint32_t)i, hsv(hue, max_whiteness, value));
    }

    data->phase = (data->phase + 1) % leds->length;
    data->color_wheel = (data->color_wheel + 3) % full_hue;
}

/*                                 Scanner                                  */

typedef struct {
    uint16_t pointer;
    uint16_t bg_hue;
} scanner_data_t;

STATIC_ASSERT(sizeof(scanner_data_t) <= effects_shared_data_size);

static void scanner_setup(void *raw_data, const void *initial) {
    scanner_data_t * const data = (scanner_data_t *) raw_data;
    data->pointer = 0;
    data->bg_hue = 0;
}

static void scanner_loop(smartled_t *leds, void *raw_data) {
    scanner_data_t * const data = (scanner_data_t *) raw_data;
    int32_t i;
    int32_t length = (int32_t)leds->length;

    /* Vibrant scanner dot color versus calm ambient background */
    const int32_t scan_hue = 0; /* Pure red neon pointer */

    for (i = 0; i < length; i++) {
        if (i == (int32_t)data->pointer) {
            /* Render the leading laser dot at maximum intensity */
            smartled_set_hsv(leds, (uint32_t)i, hsv(scan_hue, max_whiteness, max_value));
        } else {
            /* Render uniform dim background layer */
            smartled_set_hsv(leds, (uint32_t)i, hsv(data->bg_hue, max_whiteness, max_value >> 3));
        }
    }

    data->pointer = (data->pointer + 1) % leds->length;
    
    /* Slowly cycle the background environment hue */
    if (data->pointer == 0) {
        data->bg_hue = (data->bg_hue + (full_hue / 24)) % full_hue;
    }
}

/*                              Double Orbit                               */

typedef struct {
    uint16_t pos1;       /* Fast particle index */
    uint16_t subpos2;    /* Slow particle sub-pixel position scaled by 16 */
    uint16_t hue1;
    uint16_t hue2;
} double_orbit_data_t;

STATIC_ASSERT(sizeof(double_orbit_data_t) <= effects_shared_data_size);

static void double_orbit_setup(void *raw_data, const void *initial) {
    double_orbit_data_t * const data = (double_orbit_data_t *) raw_data;
    data->pos1 = 0;
    data->subpos2 = 0;
    data->hue1 = 15;              /* Neon orange */
    data->hue2 = (full_hue * 2) / 3; /* Electric blue */
}

static void double_orbit_loop(smartled_t *leds, void *raw_data) {
    double_orbit_data_t * const data = (double_orbit_data_t *) raw_data;
    int32_t i;
    int32_t length = (int32_t)leds->length;
    int32_t pos2 = (int32_t)(data->subpos2 >> 4); /* Derive actual index from sub-pixel configuration */

    for (i = 0; i < length; i++) {
        /* Compute fade profile for the first fast particle */
        int32_t dist1 = (int32_t)data->pos1 - i;
        if (dist1 < 0) dist1 += length;
        int32_t val1 = (dist1 < 4) ? (max_value - (dist1 * (max_value >> 2))) : 0;

        /* Compute fade profile for the second slow counter-rotating particle */
        int32_t dist2 = i - pos2;
        if (dist2 < 0) dist2 += length;
        int32_t val2 = (dist2 < 5) ? (max_value - (dist2 * (max_value / 5))) : 0;

        if (val1 > 0 && val2 > 0) {
            /* Blend positions using maximum intensity selection and proportional hue blending */
            int32_t total_val = val1 + val2;
            int32_t coef = (val1 << 8) / total_val;
            int32_t mixed_hue = data->hue2 + (((data->hue1 - data->hue2) * coef) >> 8);
            mixed_hue = (mixed_hue + full_hue) % full_hue;
            int32_t mixed_val = val1 > val2 ? val1 : val2;
            smartled_set_hsv(leds, (uint32_t)i, hsv(mixed_hue, max_whiteness, mixed_val));
        } else if (val1 > 0) {
            smartled_set_hsv(leds, (uint32_t)i, hsv(data->hue1, max_whiteness, val1));
        } else if (val2 > 0) {
            smartled_set_hsv(leds, (uint32_t)i, hsv(data->hue2, max_whiteness, val2));
        } else {
            smartled_set_hsv(leds, (uint32_t)i, hsv(0, 0, 0));
        }
    }

    /* Fast particle moves 1 led per step forward */
    data->pos1 = (data->pos1 + 1) % leds->length;

    /* Slow particle moves backwards at a fraction of frame speed (e.g., 0.5 leds per step) */
    if (data->subpos2 < 8) {
        data->subpos2 = (uint16_t)((length << 4) - (8 - data->subpos2));
    } else {
        data->subpos2 -= 8; /* Subtract half an integer index step */
    }
}

/*                                 Plasma                                  */

typedef struct {
    uint16_t phase_time;
    uint16_t phase_space;
} plasma_data_t;

STATIC_ASSERT(sizeof(plasma_data_t) <= effects_shared_data_size);

static void plasma_setup(void *raw_data, const void *initial) {
    plasma_data_t * const data = (plasma_data_t *) raw_data;
    data->phase_time = 0;
    data->phase_space = 0;
}

static void plasma_loop(smartled_t *leds, void *raw_data) {
    plasma_data_t * const data = (plasma_data_t *) raw_data;
    int32_t i;
    int32_t length = (int32_t)leds->length;

    for (i = 0; i < length; i++) {
        /* Approximate sinusoidal behavior using an absolute triangular phase calculation */
        int32_t temp_wave = (i + (int32_t)data->phase_time) % 11;
        int32_t tri_wave = (temp_wave > 5) ? (11 - temp_wave) : temp_wave; /* Range [0..5] */

        /* Dynamically scale value using fixed-point mapping */
        int32_t value = (tri_wave * max_value) / 5;

        /* Apply a shifting spatial color gradient offset by the temporal evolution variable */
        int32_t hue = ((i * (full_hue / length)) + (data->phase_space >> 2)) % full_hue;

        smartled_set_hsv(leds, (uint32_t)i, hsv(hue, max_whiteness, value));
    }

    data->phase_time = (data->phase_time + 1) % 22;
    data->phase_space = (data->phase_space + 3) % (full_hue << 2);
}

/*                              Strobe Blocks                               */

typedef struct {
    uint16_t rotation_counter;
    uint16_t state_timer;
    uint16_t base_hue;
    uint8_t strobe_state;
} strobe_blocks_data_t;

STATIC_ASSERT(sizeof(strobe_blocks_data_t) <= effects_shared_data_size);

static void strobe_blocks_setup(void *raw_data, const void *initial) {
    strobe_blocks_data_t * const data = (strobe_blocks_data_t *) raw_data;
    data->rotation_counter = 0;
    data->state_timer = 0;
    data->base_hue = 0;
    data->strobe_state = 0;
}

static void strobe_blocks_loop(smartled_t *leds, void *raw_data) {
    strobe_blocks_data_t * const data = (strobe_blocks_data_t *) raw_data;
    int32_t i;
    int32_t length = (int32_t)leds->length;

    for (i = 0; i < length; i++) {
        /* Map absolute led index into modular quadrant space to form 4 distinct segments */
        int32_t sector_pos = (i - (int32_t)data->rotation_counter) % 5;
        if (sector_pos < 0) sector_pos += 5;

        int32_t value = 0;
        int32_t hue = data->base_hue;

        if (data->strobe_state == 1) {
            /* Strobe state active: flash the entire ring rapidly with altered color palette */
            value = (data->state_timer & 1) ? max_value : 0;
            hue = (data->base_hue + (full_hue / 2)) % full_hue;
        } else {
            /* Default rotational state: check if current index sits inside a lit segment block */
            if (sector_pos < 3) {
                value = max_value;
            }
        }

        smartled_set_hsv(leds, (uint32_t)i, hsv(hue, max_whiteness, value));
    }

    data->state_timer++;

    if (data->strobe_state == 0) {
        /* Normal rotation operation */
        data->rotation_counter = (data->rotation_counter + 1) % leds->length;
        if (data->state_timer >= 120) { /* Enter flash phase after approximately 2 seconds at 60Hz */
            data->strobe_state = 1;
            data->state_timer = 0;
        }
    } else {
        /* High speed toggle evaluation phase */
        if (data->state_timer >= 12) { /* Finish strobe flash execution loop after 12 frames */
            data->strobe_state = 0;
            data->state_timer = 0;
            data->base_hue = (data->base_hue + (full_hue / 4)) % full_hue; /* Step color node forward */
        }
    }
}

/*                                 Ripple                                  */

typedef struct {
    uint16_t wave_phase;
    uint16_t base_hue;
} ripple_data_t;

STATIC_ASSERT(sizeof(ripple_data_t) <= effects_shared_data_size);

static void ripple_setup(void *raw_data, const void *initial) {
    ripple_data_t * const data = (ripple_data_t *) raw_data;
    data->wave_phase = 0;
    data->base_hue = 0;
}

static void ripple_loop(smartled_t *leds, void *raw_data) {
    ripple_data_t * const data = (ripple_data_t *) raw_data;
    int32_t i;
    int32_t length = (int32_t)leds->length;

    for (i = 0; i < length; i++) {
        /* Map led coordinates to mirror symmetrically relative to two polar centers (0 and 11) */
        int32_t local_coord = i % (length / 2); /* Range [0..10] */
        if (local_coord > (length / 4)) {
            local_coord = (length / 2) - local_coord;
        }

        /* Calculate wavefront propagation logic using temporal phase offset */
        int32_t dist = (local_coord - (int32_t)data->wave_phase) % 6;
        if (dist < 0) dist += 6;

        int32_t value = 0;
        /* Generate sharp pulse ripple with immediate step decay */
        if (dist == 0) value = max_value;
        else if (dist == 1) value = max_value >> 1;
        else if (dist == 2) value = max_value >> 3;

        /* Distribute running hue palette mapped over the ring shape */
        int32_t hue = (data->base_hue + (i * (full_hue / length))) % full_hue;

        smartled_set_hsv(leds, (uint32_t)i, hsv(hue, max_whiteness, value));
    }

    /* Advance the ripple wavefront animation step */
    data->wave_phase = (data->wave_phase + 1) % 6;
    data->base_hue = (data->base_hue + 2) % full_hue;
}

/*                               Neon Glitch                                */

typedef struct {
    uint32_t rand_seed;
    uint8_t led_glitch_timer[22]; /* Tracks remaining glitch duration for each cell */
} neon_glitch_data_t;

STATIC_ASSERT(sizeof(neon_glitch_data_t) <= effects_shared_data_size);

static void neon_glitch_setup(void *raw_data, const void *initial) {
    neon_glitch_data_t * const data = (neon_glitch_data_t *) raw_data;
    data->rand_seed = 77711;
    uint32_t i;
    for (i = 0; i < 22; i++) {
        data->led_glitch_timer[i] = 0;
    }
}

static void neon_glitch_loop(smartled_t *leds, void *raw_data) {
    neon_glitch_data_t * const data = (neon_glitch_data_t *) raw_data;
    int32_t i;
    int32_t length = (int32_t)leds->length;

    /* Base color configuration: Cyberpunk Cyan / Cool Ice Blue */
    const int32_t base_hue = 190; 

    /* Linear Congruential Generator step execution */
    data->rand_seed = data->rand_seed * 1103515245 + 12345;
    uint32_t trigger_chance = data->rand_seed % 100;
    uint32_t target_index = (data->rand_seed >> 16) % leds->length;

    /* With a 4% chance, trigger a rapid structural glitch on a random led node */
    if (trigger_chance < 4 && data->led_glitch_timer[target_index] == 0) {
        data->led_glitch_timer[target_index] = (uint8_t)(5 + (data->rand_seed & 7)); /* Set duration */
    }

    for (i = 0; i < length; i++) {
        int32_t value = max_value;

        if (data->led_glitch_timer[i] > 0) {
            /* Glitch phase active: enforce unstable voltage simulation drop */
            data->led_glitch_timer[i]--;
            
            /* Alternate light states intensely using bit masks over remaining time ticks */
            value = (data->led_glitch_timer[i] & 1) ? (max_value >> 3) : 0;
        }

        smartled_set_hsv(leds, (uint32_t)i, hsv(base_hue, max_whiteness, value));
    }
}

/*                              Dynamic Trail                               */

typedef struct {
    uint16_t head_pos;
    uint16_t dynamic_timer;
} dynamic_trail_data_t;

STATIC_ASSERT(sizeof(dynamic_trail_data_t) <= effects_shared_data_size);

static void dynamic_trail_setup(void *raw_data, const void *initial) {
    dynamic_trail_data_t * const data = (dynamic_trail_data_t *) raw_data;
    data->head_pos = 0;
    data->dynamic_timer = 0;
}

static void dynamic_trail_loop(smartled_t *leds, void *raw_data) {
    dynamic_trail_data_t * const data = (dynamic_trail_data_t *) raw_data;
    int32_t i;
    int32_t length = (int32_t)leds->length;
    int32_t half_ring = length / 2;

    /* Evaluate breathing width index limit using dynamic clock tracking */
    int32_t cycle_step = (int32_t)data->dynamic_timer % 16;
    int32_t max_trail_len = (cycle_step > 8) ? (16 - cycle_step) : cycle_step; /* Range [0..8] */
    max_trail_len = 2 + (max_trail_len >> 1); /* Rescale boundary down to [2..6] leds length */

    /* Fixed hue tone: Amber/Orange light beams */
    const int32_t trail_hue = 30;

    for (i = 0; i < length; i++) {
        /* Map pixel positions into dual symmetric hemisphere configurations */
        int32_t dist = ((int32_t)data->head_pos - i) % half_ring;
        if (dist < 0) dist += half_ring;

        int32_t value = 0;
        if (dist == 0) {
            value = max_value; /* Front leading segment node */
        } else if (dist < max_trail_len) {
            /* Interpolate tail attenuation based on calculated breath range limits */
            value = ((max_trail_len - dist) * max_value) / max_trail_len;
        }

        smartled_set_hsv(leds, (uint32_t)i, hsv(trail_hue, max_whiteness, value));
    }

    data->head_pos = (data->head_pos + 1) % leds->length;
    data->dynamic_timer++;
}

/*                                  Helix                                  */

typedef struct {
    uint16_t rotation_index;
    uint16_t palette_hue;
} helix_data_t;

STATIC_ASSERT(sizeof(helix_data_t) <= effects_shared_data_size);

static void helix_setup(void *raw_data, const void *initial) {
    helix_data_t * const data = (helix_data_t *) raw_data;
    data->rotation_index = 0;
    data->palette_hue = 0;
}

static void helix_loop(smartled_t *leds, void *raw_data) {
    helix_data_t * const data = (helix_data_t *) raw_data;
    int32_t i;
    int32_t length = (int32_t)leds->length;

    for (i = 0; i < length; i++) {
        /* Track phase line 1 rotating clockwise */
        int32_t phase1 = (i - (int32_t)data->rotation_index) % 11;
        if (phase1 < 0) phase1 += 11;
        int32_t tri_wave1 = (phase1 > 5) ? (11 - phase1) : phase1; /* Range [0..5] */

        /* Track phase line 2 rotating counter-clockwise */
        int32_t phase2 = (i + (int32_t)data->rotation_index) % 11;
        if (phase2 < 0) phase2 += 11;
        int32_t tri_wave2 = (phase2 > 5) ? (11 - phase2) : phase2; /* Range [0..5] */

        /* Linear scale mapped down to brightness values */
        int32_t val1 = (tri_wave1 * max_value) / 5;
        int32_t val2 = (tri_wave2 * max_value) / 5;

        /* Additive value blending to emphasize overlap junction peak points */
        int32_t combined_value = val1 + val2;
        if (combined_value > max_value) combined_value = max_value;

        /* Assign two contrasting color nodes depending on dominant local node wave amplitude */
        int32_t current_hue;
        if (val1 >= val2) {
            current_hue = data->palette_hue;
        } else {
            current_hue = (data->palette_hue + (full_hue / 2)) % full_hue; /* Inverse color wheel node */
        }

        smartled_set_hsv(leds, (uint32_t)i, hsv(current_hue, max_whiteness, combined_value));
    }

    data->rotation_index = (data->rotation_index + 1) % leds->length;
    data->palette_hue = (data->palette_hue + 2) % full_hue;
}

/***************************************************************************/
/*                              Used effects                               */
/***************************************************************************/

static const perlin_data_t perlin_initials[] =
{
    { {(RGB_t) {255, 0,   255}, (RGB_t) {0,   255, 255}}, 0 },
    { {(RGB_t) {255, 255, 0  }, (RGB_t) {0,   255, 255}}, 0 },
    { {(RGB_t) {0  , 255, 0  }, (RGB_t) {255, 0,   255}}, 0 },
    { {(RGB_t) {180, 0,   0  }, (RGB_t) {255, 255, 100}}, 0 }, /* flame */
};

static const comets_collision_init_t fire_and_ice = { .hue1 = 0, .hue2 = 180 };
static const comets_collision_init_t cyberpunk = { .hue1 = 320, .hue2 = 180 };
static const comets_collision_init_t storm = { .hue1 = 45, .hue2 = 240 };
static const comets_collision_init_t magma = { .hue1 = 25, .hue2 = 290 };
static const comets_collision_init_t mint_candy = { .hue1 = 140, .hue2 = 340 };

static const effect_t effects[] =
{
    { running_rainbow_setup, running_rainbow_loop, NULL },
    { all_rainbow_setup, all_rainbow_loop, NULL },
    
    { perlin_setup, perlin_loop, &(perlin_initials[0]) },
    { perlin_setup, perlin_loop, &(perlin_initials[1]) },
    { perlin_setup, perlin_loop, &(perlin_initials[2]) },
    { perlin_setup, perlin_loop, &(perlin_initials[3]) },

    { running_shadow_setup, running_shadow_loop, NULL },
    { running_rainbow_from_middle_setup, running_rainbow_from_middle_loop, NULL },
    { stepping_colors_setup, stepping_colors_loop, NULL },
    
    /* AI-generated effects, must be careful */
    
    { comet_setup, comet_loop, NULL },
    { heartbeat_setup, heartbeat_loop, NULL },
    { spinner_setup, spinner_loop, NULL },

    { comets_collision_setup, comets_collision_loop, &fire_and_ice },
    { comets_collision_setup, comets_collision_loop, &cyberpunk },
    { comets_collision_setup, comets_collision_loop, &storm },
    { comets_collision_setup, comets_collision_loop, &magma },
    { comets_collision_setup, comets_collision_loop, &mint_candy },
    
    { fire_setup, fire_loop, NULL },
    { radar_setup, radar_loop, NULL },
    { vu_meter_setup, vu_meter_loop, NULL },
    { vortex_setup, vortex_loop, NULL },
    { scanner_setup, scanner_loop, NULL },
    { double_orbit_setup, double_orbit_loop, NULL },
    { plasma_setup, plasma_loop, NULL },
    { strobe_blocks_setup, strobe_blocks_loop, NULL },
    { ripple_setup, ripple_loop, NULL },
    { neon_glitch_setup, neon_glitch_loop, NULL },
    { dynamic_trail_setup, dynamic_trail_loop, NULL },
    { helix_setup, helix_loop, NULL }
};

/***************************************************************************/
/*                                Interface                                */
/***************************************************************************/

void effects_service_select_effect(uint32_t idx);

void effects_service_init(void)
{
    lfsr_srand(1);
    effects_service_select_effect(0);
}

void effects_service_select_effect(uint32_t idx)
{
    current_effect_idx = idx;
    (effects[idx].setup)(effects_shared_data, effects[idx].initial);
}

void effects_service_next_effect(bool random_choice)
{
    uint32_t idx = random_choice
	           ? lfsr_rand()
		   : current_effect_idx + 1;

    idx %= (sizeof(effects) / sizeof(effects[0]));

    effects_service_select_effect(idx);
}

void effects_service_play_currect_effect(smartled_t *leds)
{
    (effects[current_effect_idx].loop)(leds, effects_shared_data);
}
