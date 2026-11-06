#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32fxxx_hal.h"
#include <stdint.h>

/* ============================================================================
 * Configuration
 * ========================================================================== */

typedef struct
{
    float counts_per_revolution;
    float sample_period_s;

    float blend_lower_rps;
    float blend_upper_rps;

    uint8_t enable_low_pass;
    float low_pass_time_constant_s;

} EncoderConfig_t;

/* ============================================================================
 * Public API
 * ========================================================================== */

void Encoder_Init(TIM_HandleTypeDef *encoder_timer,
                  TIM_HandleTypeDef *timestamp_timer,
                  const EncoderConfig_t *config);

void Encoder_CaptureCallback(TIM_HandleTypeDef *htim);

void Encoder_PeriodicUpdate(void);

void Encoder_UpdateVelocityEstimate(void);

float Encoder_GetVelocity(void);

float Encoder_GetFastVelocity(void);

float Encoder_GetSlowVelocity(void);

int32_t Encoder_GetCount(void);

#ifdef __cplusplus
}
#endif

#endif
