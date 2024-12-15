#include <stddef.h>

#include "button.h"

enum button_init_result
button_init(struct button *btn,
            uint16_t double_click_pause,
            bool (*read)(void),
            void (*click_callback)(uint8_t))
{
    if (btn == NULL ||
        read == NULL ||
        click_callback == NULL ||
        double_click_pause == 0)
    {
        return button_init_fail;
    }

    btn->debounce_timer = 0;
    btn->is_pressed = false;
    btn->prev_click_timer = 0;
    btn->click_counter = 0;
    btn->double_click_pause = double_click_pause;

    btn->read = read;
    btn->click_callback = click_callback;

    return button_init_success;
}

static bool button_is_rising(struct button const *b)
{
    return !b->is_pressed &&
           b->debounce_timer == button_debounce_timer_top;
}

static bool button_is_falling(struct button const *b)
{
    return b->is_pressed &&
           b->debounce_timer == 0;
}

static bool button_is_released(struct button const *b)
{
    return b->prev_click_timer == 0 &&
           !b->is_pressed &&
           b->click_counter != 0;
}

void button_check(struct button *b)
{
    if (b->read())
    {
        if (b->debounce_timer < button_debounce_timer_top)
        {
            b->debounce_timer++;
        }
    }
    else
    {
        if (b->debounce_timer > 0)
        {
            b->debounce_timer--;
        }
    }

    if (b->prev_click_timer != 0)
    {
        b->prev_click_timer--;
    }

    if (button_is_rising(b))
    {
        b->is_pressed = true;
        b->click_counter++;
    }

    if (button_is_falling(b))
    {
        b->is_pressed = false;
        b->prev_click_timer = b->double_click_pause;
    }

    if (button_is_released(b))
    {
        b->click_callback(b->click_counter);
        b->click_counter = 0;
    }
}