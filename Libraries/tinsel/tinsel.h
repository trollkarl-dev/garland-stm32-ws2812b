#ifndef TINSEL_H_
#define TINSEL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

enum {
    tinsel_max_tasks = 8,
    tinsel_max_timers = 8
};

typedef void (*tinsel_task_worker)(uint32_t);

void tinsel_init();
uint32_t tinsel_run();
void tinsel_timer_check(); /* must be called in time-based ISR */

void tinsel_add_task(tinsel_task_worker routine, uint32_t data);
uint32_t tinsel_add_task_timer(tinsel_task_worker routine, uint32_t data, uint32_t id, uint32_t period);
void tinsel_del_task(uint32_t id);

#ifdef __cplusplus
}
#endif

#endif /* TINSEL_H_ */