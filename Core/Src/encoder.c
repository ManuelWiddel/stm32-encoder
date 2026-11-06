#include "encoder.h"

/* ============================================================================
 * Constants
 * ========================================================================== */

#define SAMPLE_PERIOD_TICK_S      (1e-6f)
#define STOP_TIMEOUT_US           (60000U)
#define SLOW_EST_JITTER_REDUCTION (4U)

/* ============================================================================
 * Private types
 * ========================================================================== */

typedef struct
{
    volatile float fast_rps;
    volatile float slow_rps;
    volatile float blended_rps;

} EncoderVelocity_t;

/* ============================================================================
 * Private variables
 * ========================================================================== */

static TIM_HandleTypeDef *encoder_timer = NULL;
static TIM_HandleTypeDef *timestamp_timer = NULL;

static EncoderConfig_t config;

static volatile int32_t encoder_counter = 0;
static volatile uint32_t prev_encoder_cnt = 0;
static volatile int32_t counts_last_sample = 0;

static volatile uint32_t slow_dt = 0;
static volatile uint32_t last_change_time = 0;
static volatile uint32_t slow_prev_count = 0;

static volatile uint8_t slow_flag = 0;
static volatile uint8_t slow_est_active = 0;

static uint8_t capture_event_count = 0;

static float fast_gain = 0.0f;

static float velocity_filtered = 0.0f;

static EncoderVelocity_t velocity;

/* ============================================================================
 * Private functions
 * ========================================================================== */

/**
 * @brief Clamp a value to the given limits.
 *
 * @param value Input value.
 * @param min Lower limit.
 * @param max Upper limit.
 *
 * @return Clamped value.
 */
static inline float Encoder_Clamp(float value,
                                  float min,
                                  float max)
{
    if (value < min)
    {
        return min;
    }

    if (value > max)
    {
        return max;
    }

    return value;
}

/**
 * @brief Detect a stopped encoder.
 *
 * Forces the velocity estimate to zero when no encoder
 * transition has been detected for STOP_TIMEOUT_US.
 */
static void Encoder_BackgroundTask(void)
{
    uint32_t now =
        __HAL_TIM_GET_COUNTER(timestamp_timer);

    if ((uint32_t)(now - last_change_time) >
        STOP_TIMEOUT_US)
    {
        velocity.blended_rps = 0.0f;
    }
}

/**
 * @brief Calculate signed encoder increment.
 *
 * Handles encoder timer rollover for 16-bit and 32-bit
 * timers and returns the shortest signed distance.
 *
 * @param current Current counter value.
 * @param previous Previous counter value.
 *
 * @return Signed count increment.
 */
static int32_t Encoder_GetDelta(uint32_t current,
                                uint32_t previous)
{
    uint64_t modulus =
        (uint64_t)__HAL_TIM_GET_AUTORELOAD(encoder_timer) + 1ULL;

    int64_t delta =
        ((uint64_t)current + modulus - previous) % modulus;

    if (delta >= (int64_t)(modulus / 2ULL))
    {
        delta -= (int64_t)modulus;
    }

    return (int32_t)delta;
}

/**
 * @brief First-order low-pass filter.
 *
 * @param input Unfiltered velocity.
 * @param dt Sample period.
 *
 * @return Filtered velocity.
 */
static float Encoder_LowPassFilter(float input,
                                   float dt)
{
    if ((config.low_pass_time_constant_s <= 0.0f) ||
        (dt <= 0.0f))
    {
        velocity_filtered = input;
        return velocity_filtered;
    }

    float alpha =
        dt / (config.low_pass_time_constant_s + dt);

    velocity_filtered +=
        alpha * (input - velocity_filtered);

    return velocity_filtered;
}

/* ============================================================================
 * Public API
 * ========================================================================== */

/**
 * @brief Initialize encoder module.
 *
 * @param encoder_timer_ Timer running in encoder mode.
 * @param timestamp_timer_ Free-running timestamp timer.
 * @param config_ Configuration structure.
 */
void Encoder_Init(TIM_HandleTypeDef *encoder_timer_,
                  TIM_HandleTypeDef *timestamp_timer_,
                  const EncoderConfig_t *config_)
{
    encoder_timer = encoder_timer_;
    timestamp_timer = timestamp_timer_;

    config = *config_;

    encoder_counter = 0;
    prev_encoder_cnt =
        __HAL_TIM_GET_COUNTER(encoder_timer);

    counts_last_sample = 0;

    slow_dt = 0;
    last_change_time = 0;
    slow_prev_count = 0;

    slow_flag = 0;
    slow_est_active = 0;

    capture_event_count = 0;

    velocity.fast_rps = 0.0f;
    velocity.slow_rps = 0.0f;
    velocity.blended_rps = 0.0f;

    velocity_filtered = 0.0f;

    fast_gain =
        1.0f /
        (config.counts_per_revolution *
         config.sample_period_s);
}

/**
 * @brief Update blended velocity estimate.
 *
 * Combines slow and fast estimators using a smooth
 * transition function.
 */
void Encoder_UpdateVelocityEstimate(void)
{
    float speed_mag =
        (velocity.fast_rps >= 0.0f)
            ? velocity.fast_rps
            : -velocity.fast_rps;

    float range =
        config.blend_upper_rps -
        config.blend_lower_rps;

    float alpha = 1.0f;

    if (range > 0.0f)
    {
        alpha =
            (speed_mag - config.blend_lower_rps) /
            range;

        alpha =
            Encoder_Clamp(alpha, 0.0f, 1.0f);

        alpha =
            alpha * alpha *
            (3.0f - 2.0f * alpha);
    }

    velocity.blended_rps =
        (1.0f - alpha) * velocity.slow_rps +
        alpha * velocity.fast_rps;

    if (config.enable_low_pass)
    {
        velocity.blended_rps =
            Encoder_LowPassFilter(
                velocity.blended_rps,
                config.sample_period_s);
    }

    if ((velocity.blended_rps < config.blend_lower_rps) &&
        (velocity.blended_rps > -config.blend_lower_rps))
    {
        Encoder_BackgroundTask();
    }
}

/**
 * @brief Process encoder capture event.
 *
 * Must be called from the STM32 input capture callback.
 *
 * @param htim Timer handle generating the event.
 */
void Encoder_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim != encoder_timer)
    {
        return;
    }

    if (!slow_flag)
    {
        return;
    }

    capture_event_count++;

    if (capture_event_count < SLOW_EST_JITTER_REDUCTION)
    {
        return;
    }

    if (!slow_est_active)
    {
        slow_est_active = 1;

        slow_prev_count =
            __HAL_TIM_GET_COUNTER(encoder_timer);
    }

    uint32_t cnt =
        __HAL_TIM_GET_COUNTER(encoder_timer);

    int32_t delta =
        Encoder_GetDelta(cnt,
                         slow_prev_count);

    slow_prev_count = cnt;

    int32_t counts =
        (delta >= 0) ? delta : -delta;

    uint32_t now =
        __HAL_TIM_GET_COUNTER(timestamp_timer);

    slow_dt =
        now - last_change_time;

    if (slow_dt > 0U)
    {
        velocity.slow_rps =
            (float)counts /
            (config.counts_per_revolution *
             (slow_dt * SAMPLE_PERIOD_TICK_S));

        if (delta < 0)
        {
            velocity.slow_rps =
                -velocity.slow_rps;
        }
    }
    else
    {
        velocity.slow_rps = 0.0f;
    }

    last_change_time = now;
    capture_event_count = 0;
}

/**
 * @brief Execute periodic fast estimator update.
 *
 * Must be called with a fixed sample period.
 */
void Encoder_PeriodicUpdate(void)
{
    uint32_t cnt =
        __HAL_TIM_GET_COUNTER(encoder_timer);

    int32_t delta =
        Encoder_GetDelta(cnt,
                         prev_encoder_cnt);

    prev_encoder_cnt = cnt;

    encoder_counter += delta;

    counts_last_sample =
        (delta >= 0) ? delta : -delta;

    velocity.fast_rps =
        ((float)delta) * fast_gain;

    float speed_mag =
        (velocity.fast_rps >= 0.0f)
            ? velocity.fast_rps
            : -velocity.fast_rps;

    if (speed_mag >
        config.blend_upper_rps)
    {
        slow_flag = 0;
        slow_est_active = 0;
    }
    else
    {
        slow_flag = 1;
    }
}

float Encoder_GetVelocity(void)
{
    return velocity.blended_rps;
}

float Encoder_GetFastVelocity(void)
{
    return velocity.fast_rps;
}

float Encoder_GetSlowVelocity(void)
{
    return velocity.slow_rps;
}

int32_t Encoder_GetCount(void)
{
    return encoder_counter;
}
