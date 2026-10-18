#include "nec.h"

#include <stdlib.h>

static uint32_t nec_check_pulse_duration(int32_t pulse, int32_t reference)
{
    return abs(pulse - reference) <= nec_dur_tolerance;
}

static uint32_t nec_pulse_is_rep(int32_t pulse)
{
    return nec_check_pulse_duration(pulse, nec_rep_duration);
}

static uint32_t nec_pulse_is_agc(int32_t pulse)
{
    return nec_check_pulse_duration(pulse, nec_agc_duration);
}

static uint32_t nec_pulse_is_bit0(int32_t pulse)
{
    return nec_check_pulse_duration(pulse, nec_bit0_duration);
}

static uint32_t nec_pulse_is_bit1(int32_t pulse)
{
    return nec_check_pulse_duration(pulse, nec_bit1_duration);
}

static uint32_t nec_pulse_is_bit(int32_t pulse)
{
    return nec_pulse_is_bit0(pulse) || nec_pulse_is_bit1(pulse);
}

static uint32_t nec_pulses_is_valid(struct NEC *nec)
{
    return ((nec->pulses.decoded.command ^ nec->pulses.decoded.n_command) == 0xFF) &&
           ((nec->pulses.decoded.address ^ nec->pulses.decoded.n_address) == 0xFF);
}

void NEC_Start(struct NEC *nec)
{
    nec->state = nec_state_wait_for_agc;
    nec->start();
}

void NEC_Worker(struct NEC *nec)
{
    int32_t pulse = nec->read_pulse();
    
    switch (nec->state) {
    case nec_state_wait_for_agc:
        if (nec_pulse_is_rep(pulse)) {
            nec->rep_callback();
            break;
        }
        
        if (nec_pulse_is_agc(pulse)) {
            nec->state = nec_state_read_payload;
            nec->bit_counter = 0;
            break;
        }
        
        break;

    case nec_state_read_payload:
        if (!nec_pulse_is_bit(pulse)) {
            nec->state = nec_state_wait_for_agc;
            nec->err_callback(nec_error_bad_pulse_duration);
            break;
        }
        
        if (nec_pulse_is_bit1(pulse)) {
            nec->pulses.raw |= (1 << nec->bit_counter);
        }
        else
        {
            nec->pulses.raw &= ~(1 << nec->bit_counter);
        }
        
        nec->bit_counter++;
        
        if (nec->bit_counter == 32) {
            nec->state = nec_state_wait_for_agc;
            
            if (nec_pulses_is_valid(nec)) {
                nec->dec_callback(nec->pulses.decoded.address,
                                  nec->pulses.decoded.command);
            } else {
                nec->err_callback(nec_error_invalid_data);
            }
            
            break;
        }
    }
}