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

enum { effects_shared_data_size = 12 };

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
    { comet_setup, comet_loop, NULL },
    { heartbeat_setup, heartbeat_loop, NULL },
    { spinner_setup, spinner_loop, NULL },

    { comets_collision_setup, comets_collision_loop, &fire_and_ice },
    { comets_collision_setup, comets_collision_loop, &cyberpunk },
    { comets_collision_setup, comets_collision_loop, &storm },
    { comets_collision_setup, comets_collision_loop, &magma },
    { comets_collision_setup, comets_collision_loop, &mint_candy }
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
