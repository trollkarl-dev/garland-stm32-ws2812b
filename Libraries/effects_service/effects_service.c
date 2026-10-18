#include "effects_service.h"

#include <string.h>
#include <math.h>

#include "colorspaces.h"
#include "smartled.h"
#include "perlin.h"
#include "utils.h"

#define ABS(A) (((A) < 0) ? (-(A)) : (A))

enum {
    full_hue = 360,
    max_whiteness = 100,
    max_value = 100
};

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

static float sigmoid(float x)
{
    const float threshold = 0.15f;
    
    if (fabs(x - 0.5f) < threshold)
    {
        return 0.5f * (1.0f + (x - 0.5f) / threshold);
    }
    
    return (x < 0.5f) ? 0.0f : 1.0f;
}

static void perlin_loop(smartled_t *leds, void *raw_data)
{
    perlin_data_t * const data = (perlin_data_t *) raw_data;
    uint32_t i;
    const uint32_t vertical_size = 1048576; /* must be a power of two */
    
    for (i = 0; i < leds->length; i++)
    {
        float coef = perlin(i, data->offset, leds->length, vertical_size, 10);
        coef = sigmoid((coef + 1.0f) / 2.0f);

        RGB_t color = (RGB_t)
        {
            (uint8_t) lerp((float) data->col[0].r, (float) data->col[1].r, coef),
            (uint8_t) lerp((float) data->col[0].g, (float) data->col[1].g, coef),
            (uint8_t) lerp((float) data->col[0].b, (float) data->col[1].b, coef)
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
        int val = ABS((int) data->counter - (int) i);
	int hue, value;

	val = val > leds->length / 2 ? leds->length - val : val;

	hue = (int) ((float) i / leds->length * full_hue);
	value = (int) ((float) val / (leds->length / 2) * max_value);

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
    running_shadow_data_t * const data = (running_shadow_data_t *) raw_data;
    data->counter = 0;
}

static void running_rainbow_from_middle_loop(smartled_t *leds, void *raw_data)
{
    running_shadow_data_t * const data = (running_shadow_data_t *) raw_data;
    uint32_t i;

    for (i = 0; i < leds->length; i++)
    {
	    int hue = (float) ((ABS((int) i - (int) (leds->length / 2)) + data->counter) % leds->length) / leds->length * full_hue;
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

    if (data->direction)
    {
        for (i = leds->length; i >= 0; i--)
	{
	    HSV_t color = i > (int32_t) leds->length - (int32_t) data->counter
		          ? hsv(data->hue, max_whiteness, max_whiteness)
			  : hsv(0, 0, 0);
            smartled_set_hsv(leds, i, color);
	}
    }
    else
    {

        for (i = 0; i < leds->length; i++)
	{
	    HSV_t color = i <= (int32_t) data->counter
		          ? hsv(data->hue, max_whiteness, max_whiteness)
			  : hsv(0, 0, 0);
            smartled_set_hsv(leds, i, color);
	}
    }

    data->counter = (data->counter + 1) % leds->length;

    if (data->counter == 0)
    {
        data->direction ^= 1;
	data->hue = (data->hue + full_hue / 12) % full_hue;
    }
}

static const perlin_data_t perlin_initials[] =
{
    { {(RGB_t) {255, 0,   255}, (RGB_t) {0,   255, 255}}, 0 },
    { {(RGB_t) {255, 255, 0  }, (RGB_t) {0,   255, 255}}, 0 },
    { {(RGB_t) {0  , 255, 0  }, (RGB_t) {255, 0,   255}}, 0 },
    { {(RGB_t) {180, 0,   0  }, (RGB_t) {255, 255, 100}}, 0 }, /* flame */
};

static const effect_t effects[] = {
    { running_rainbow_setup, running_rainbow_loop, NULL },
    { all_rainbow_setup, all_rainbow_loop, NULL },
    
    { perlin_setup, perlin_loop, &(perlin_initials[0]) },
    { perlin_setup, perlin_loop, &(perlin_initials[1]) },
    { perlin_setup, perlin_loop, &(perlin_initials[2]) },
    { perlin_setup, perlin_loop, &(perlin_initials[3]) },

    { running_shadow_setup, running_shadow_loop, NULL },
    { running_rainbow_from_middle_setup, running_rainbow_from_middle_loop, NULL },
    { stepping_colors_setup, stepping_colors_loop, NULL }
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
