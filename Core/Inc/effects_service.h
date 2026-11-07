#ifndef EFFECTS_SERVICE_H_
#define EFFECTS_SERVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "smartled.h"

void effects_service_init(void);
void effects_service_next_effect(bool random_choice);
void effects_service_play_currect_effect(smartled_t *leds);

#ifdef __cplusplus
}
#endif

#endif  /* EFFECTS_SERVICE_H_ */
