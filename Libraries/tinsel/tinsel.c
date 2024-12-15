#include "tinsel.h"

#include <stdlib.h>
#include <cmsis_gcc.h>

struct tinsel_timer {
    uint32_t counter;
    uint32_t data;
    tinsel_task_worker worker;
};

struct tinsel_task {
    tinsel_task_worker worker;
    uint32_t data;
};

static struct tinsel_task tinsel_tasks_list[tinsel_max_tasks];
static struct tinsel_timer tinsel_timers_list[tinsel_max_timers];

static volatile uint32_t tinsel_rd_idx = 0;
static volatile uint32_t tinsel_wr_idx = 0;

__attribute__((weak)) void tinsel_default_worker(uint32_t data)
{
    __WFI();
}

void tinsel_init()
{
    uint32_t i;
    
    for (i = 0; i < tinsel_max_timers; i++) {
        tinsel_timers_list[i].worker = NULL;
    }
}

void tinsel_add_task(tinsel_task_worker routine, uint32_t data)
{
    __disable_irq();
    
    tinsel_tasks_list[tinsel_wr_idx].worker = routine;
    tinsel_tasks_list[tinsel_wr_idx].data = data;
    tinsel_wr_idx = (tinsel_wr_idx + 1) % tinsel_max_tasks;
    
    __enable_irq();
}

uint32_t tinsel_add_task_timer(tinsel_task_worker routine, uint32_t data, uint32_t period)
{
    uint32_t i;
    uint32_t result = 1;
    
    __disable_irq();
    
    for (i = 0; i < tinsel_max_timers; i++) {
        if (tinsel_timers_list[i].worker == NULL) {
            tinsel_timers_list[i].worker = routine;
            tinsel_timers_list[i].data = data;
            tinsel_timers_list[i].counter = period;
            
            result = 0;
            break;
        }
    }
    
    __enable_irq();
    
    return result;
}

uint32_t tinsel_run()
{
    tinsel_task_worker curr_worker;
    uint32_t data = 0xDEADBEEF;
    
    while (1) {
        curr_worker = tinsel_default_worker;
        
        __disable_irq();
        
        if (tinsel_rd_idx != tinsel_wr_idx) {
            curr_worker = tinsel_tasks_list[tinsel_rd_idx].worker;
            data = tinsel_tasks_list[tinsel_rd_idx].data;
            tinsel_rd_idx = (tinsel_rd_idx + 1) % tinsel_max_tasks;
        }
        
        __enable_irq();
        
        curr_worker(data);
    }
    
    return 0;
}

void tinsel_timer_check()
{
    uint32_t i;
    
    __disable_irq();
    
    for (i = 0; i < tinsel_max_timers; i++) {
        if (NULL == tinsel_timers_list[i].worker) {
            continue;
        }
        
        if (0 == tinsel_timers_list[i].counter) {
            tinsel_add_task(tinsel_timers_list[i].worker, tinsel_timers_list[i].data);
            tinsel_timers_list[i].worker = NULL;
        } else {
            tinsel_timers_list[i].counter--;
        }
    }
    
    __enable_irq();
}