#ifndef NEC_INC_H
#define NEC_INC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

enum nec_timings {
    nec_agc_duration = 13500,
    nec_bit0_duration = 1125,
    nec_bit1_duration = 2250,
    nec_rep_duration = 11250,
    nec_dur_tolerance = 250
};

enum nec_state {
    nec_state_wait_for_agc,
    nec_state_read_payload
};

enum nec_error {
    nec_error_bad_pulse_duration,
    nec_error_invalid_data
};

struct NEC {
    enum nec_state state;
    uint32_t bit_counter;
    
    union {
        uint32_t raw;
        
        struct {
            uint8_t address;
            uint8_t n_address;
            uint8_t command;
            uint8_t n_command;
        } decoded;
    } pulses;
    
    void (*start)(void);
    int32_t (*read_pulse)(void);
    
    void (*dec_callback)(uint8_t address, uint8_t cmd);
    void (*rep_callback)();
    void (*err_callback)(enum nec_error err_code);
};

void NEC_Start(struct NEC *nec);
void NEC_Worker(struct NEC *nec);

#ifdef __cplusplus
}
#endif

#endif /* NEC_INC_H */
