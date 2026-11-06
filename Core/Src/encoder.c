#include "encoder.h"

/* ----------------------------------------------------------
 * Configuration
 * ---------------------------------------------------------- */

#define ENCODER_CPR                 400.0f

#define SAMPLE_PERIOD_TICK_S        0.000001f

#define STOP_TIMEOUT_US             60000U

#define SLOW_EST_JITTER_REDUCTION   4U



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

static volatile int32_t encoder_counter = 0;
static volatile uint32_t prev_encoder_cnt = 0;
static volatile int32_t counts_last_sample = 0;

static volatile uint32_t slow_dt = 0;
static volatile uint32_t last_change_time = 0;
static volatile uint32_t slow_prev_count = 0;

static volatile int8_t slow_flag = 0;
static volatile int8_t slow_est_active = 0;

static EncoderVelocity_t velocity;

static float vel_smooth_lower_bnd = 0.0f;
static float vel_smooth_upper_bnd = 0.0f;

static uint8_t low_pass = 0;
static float velocity_filtered = 0.0f;
static float filter_time_constant_s = 0.0f;

static uint8_t capture_event_count = 0;

static float sample_period_s;
static float fast_gain;


/* ============================================================================
 * Private functions
 * ========================================================================== */

/**
 * @brief Limits a value to a specified range.
 *
 * @param value Input value.
 * @param min Lower bound.
 * @param max Upper bound.
 *
 * @return Clamped value.
 */
static inline float Encoder_Clamp(float value,
                                  float min,
                                  float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}


/**
@brief Detects when the encoder has stopped moving.

The timestamp of the last encoder transition is compared with
the current timestamp. If the elapsed time exceeds
STOP_TIMEOUT_US, the final velocity estimate is set to zero.
This function is called when the calculated velocity is close
to zero.
*/
void VelocityBackgroundTask(void) {
	uint16_t now = __HAL_TIM_GET_COUNTER(timestamp_timer);


	if((uint16_t)(now - last_change_time) > STOP_TIMEOUT_US)
	{
		vel_rps = 0.0f;
	}


}


/**
@brief Detects when the encoder has stopped moving.

Handles rollovers for different implementaions of encoder_timer.
This allows for the use of 32-bit and 16-bit timers.
*/
static int32_t Encoder_GetDelta(uint32_t current,
                                uint32_t previous)
{
    uint64_t modulus = (uint64_t)__HAL_TIM_GET_AUTORELOAD(encoder_timer) + 1ULL;

    int64_t delta = ((uint64_t)current + modulus - previous) % modulus;

    // Convert to the shortest signed difference
    if (delta >= (int64_t)(modulus / 2ULL))
    {
        delta -= (int64_t)modulus;
    }

    return (int32_t)delta;
}


/**
@brief 1st order Low-pass filter for speed estimate.

Applied after blending slow and fast estimation.
Cutt-off frequency determined by the filter_time_constant_s.
*/
static float LowPassFilter(float input, float dt)
{
    if (filter_time_constant_s <= 0.0f || dt <= 0.0f)
    {
        velocity_filtered = input;
        return velocity_filtered;
    }

    float alpha = dt / (filter_time_constant_s + dt);

    velocity_filtered += alpha * (input - velocity_filtered);

    return velocity_filtered;
}

/* ----------------------------------------------------------
 * Public functions
 * ---------------------------------------------------------- */


/**
@brief Calculates the final encoder velocity estimate.

The fast and slow velocity estimates are blended according to
the current magnitude of the fast velocity.
At low speeds, the slow estimator receives more weight because
measuring the time between encoder transitions provides better
resolution.
At high speeds, the fast estimator receives more weight because
encoder counts are changing frequently enough for the fixed-period
estimator to provide a stable result.
*/
void UpdateVelocityEstimate(void) {
	float speed_mag = (vel_fast_rps >= 0) ? vel_fast_rps : -vel_fast_rps;

	float alpha = (speed_mag - vel_smooth_lower_bnd) / (vel_smooth_upper_bnd - vel_smooth_lower_bnd);
	alpha = Clamp(alpha, 0.0f, 1.0f);
	alpha = alpha * alpha * (3.0f - 2.0f * alpha);
	vel_rps = (1.0f - alpha) * vel_slow_rps + alpha * vel_fast_rps;
	if (low_pass == 1) {
		vel_rps = LowPassFilter(vel_rps, sample_period_s);
		}
	if(vel_rps < vel_smooth_lower_bnd) {
		if(vel_rps > -vel_smooth_lower_bnd) {
			VelocityBackgroundTask();
		}
	}

}



/**
@brief Initializes the encoder measurement module.

Stores the timer handles, resets all encoder/velocity state,
and stores the velocity blending and filtering configuration.
*/
void Encoder_Init(TIM_HandleTypeDef *encoder_timer_,
                  TIM_HandleTypeDef *timestamp_timer_,
				  float vel_smooth_lower_bnd_,
				  float vel_smooth_upper_bnd_,
				  float sample_period_s_,
				  uint8_t low_pass_,
				  float filter_time_constant_s_) {

    encoder_timer = encoder_timer_;
    timestamp_timer = timestamp_timer_;

    encoder_counter = 0;
    prev_encoder_cnt = __HAL_TIM_GET_COUNTER(encoder_timer);
    counts_last_sample = 0;

    slow_dt = 0;
    slow_delta = 0;
    last_change_time = 0;

    slow_flag = 0;
    slow_est_active = 0;
    capture_event_count = 0;

    sample_period_s = sample_period_s_;
    fast_gain = 1.0f / (ENCODER_CPR * sample_period_s);

    vel_fast_rps = 0.0f;
    vel_fast_rps_old = 0.0f;
    vel_slow_rps = 0.0f;
    vel_rps = 0.0f;

    vel_smooth_lower_bnd = vel_smooth_lower_bnd_;
    vel_smooth_upper_bnd = vel_smooth_upper_bnd_;

    low_pass = low_pass_;
    filter_time_constant_s = filter_time_constant_s_;
}


/**
@brief Processes an encoder capture event.

When the slow estimator is enabled, this function measures the
number of encoder counts and the elapsed time since the previous
encoder transition. These values are then used to calculate the
slow velocity estimate.
The function should be called from the appropriate STM32 timer
callback.
@param htim Pointer to the timer handle that generated the callback.
*/
void Encoder_CaptureCallback(TIM_HandleTypeDef *htim) {

	if (htim != encoder_timer)
	        return;

	if (slow_flag == 1) {
		capture_event_count++;
		if(capture_event_count >= SLOW_EST_JITTER_REDUCTION) {
			static uint32_t prev_cnt = 0;

			if (slow_est_active == 0)
			{
			    slow_est_active = 1;
			    prev_cnt = __HAL_TIM_GET_COUNTER(encoder_timer);
			}

			uint32_t cnt = __HAL_TIM_GET_COUNTER(encoder_timer);

			int32_t delta = Encoder_GetDelta(cnt, prev_cnt);

			prev_cnt = cnt;

			int32_t counts = (delta >= 0) ? delta : -delta;

			uint32_t now = __HAL_TIM_GET_COUNTER(timestamp_timer);
			slow_dt = now - last_change_time;
			if(slow_dt > 0) {
				vel_slow_rps = (float)counts / (ENCODER_CPR * (slow_dt * SAMPLE_PERIOD_TICK_S));
				vel_slow_rps = (delta >= 0) ? vel_slow_rps : -vel_slow_rps;
			}
			else {
				vel_slow_rps = 0.0f;
			}
			last_change_time = now;
			capture_event_count = 0;
		}


	}
}


/**
@brief Performs the periodic fast velocity estimation.

The encoder count difference over the fixed estimator period is
converted directly into revolutions per second.
This estimator is particularly useful at higher encoder speeds,
where enough encoder counts occur during each sample period.
@param htim Pointer to the timer handle that generated the callback.
*/
void Encoder_PeriodicUpdate(void) {

	// fast estimator:
	uint32_t cnt = __HAL_TIM_GET_COUNTER(encoder_timer);

	int32_t fast_delta = Encoder_GetDelta(cnt, prev_encoder_cnt);

	prev_encoder_cnt = cnt;

	// Accumulate signed encoder position
	encoder_counter += fast_delta;

	counts_last_sample = (fast_delta >= 0) ? fast_delta : -fast_delta;

	vel_fast_rps = ((float)fast_delta) * fast_gain ;  // Make precalculation to save time in ISR

	// turn on slow estimator if vel_fast_rps is below threshold, turn off if it is above threshold
	float speed_mag = (vel_fast_rps >= 0) ? vel_fast_rps : -vel_fast_rps;

	if (speed_mag > vel_smooth_upper_bnd) {
		slow_flag = 0;
		slow_est_active = 0;
	}

	else {
		slow_flag = 1;
	}



}



float Encoder_GetVelocity(void)
{
    return vel_rps;
}


float Encoder_GetFastVelocity(void)
{
    return vel_fast_rps;
}


float Encoder_GetSlowVelocity(void)
{
    return vel_slow_rps;
}


int32_t Encoder_GetCount(void)
{
    return encoder_counter;
}
