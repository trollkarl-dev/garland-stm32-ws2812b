#ifndef BUTTON_H
#define BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

enum { button_debounce_timer_top = 20 };

enum button_init_result {
    button_init_success = 0,
    button_init_fail
};

struct button {
    uint8_t debounce_timer;
    uint8_t click_counter;

    uint16_t prev_click_timer;
    uint16_t double_click_pause;

    bool is_pressed;

    bool (*read)(void);
    void (*click_callback)(uint8_t clicks);
};

enum button_init_result
button_init(struct button *btn,
            uint16_t double_click_pause,
            bool (*read)(void),
            void (*click_callback)(uint8_t));

void button_check(struct button *); /* must be called */
                                    /* with a certain periodicity */
                                    /* (every 1 ms, for example). */

#ifdef __cplusplus
}
#endif

#endif /* BUTTON_H */
