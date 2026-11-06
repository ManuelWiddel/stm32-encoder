#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

/**

@brief Initializes the quadrature encoder measurement module.
@param encoder_timer Timer configured in encoder/quadrature mode.
@param timestamp_timer Timer used to measure the time between encoder events.
@param estimator_timer Timer used to periodically calculate the fast velocity estimate.
@param vel_smooth_lower_bnd Lower velocity boundary used when blending the slow and fast velocity estimates.
@param vel_smooth_upper_bnd  Upper velocity boundary used when switching between the slow and fast velocity estimators.
@param low_pass Enables the low-pass filter for the fast velocity estimate when set to 1.

*/
void Encoder_Init(TIM_HandleTypeDef *encoder_timer,
				  TIM_HandleTypeDef *timestamp_timer,
				  float vel_smooth_lower_bnd,
				  float vel_smooth_upper_bnd,
				  float sample_period_s,
				  uint8_t low_pass,
				  float filter_time_constant_s);

/**

@brief Handles an encoder timer capture/update event.

This function is intended to be called from the STM32 timer callback.
When the encoder is operating in the low-speed region, the time between
encoder changes is used to calculate a slow but more accurate velocity.


@param htim Pointer to the timer handle that generated the callback.
*/
void Encoder_CaptureCallback(TIM_HandleTypeDef *htim);

/**

@brief Performs the periodic fast velocity calculation.

This function is intended to be called periodically using the estimator
timer. It calculates velocity from the number of encoder counts measured
during the fixed sample period.


@param htim Pointer to the timer handle that generated the callback.
*/
void Encoder_PeriodicUpdate(void);



/**
@brief Checks whether the encoder has stopped moving.
If no encoder transition has occurred for longer than the configured
timeout, the output velocity is set to zero.
*/
void VelocityBackgroundTask(void);

/**


@brief Combines the fast and slow velocity estimates.
The fast estimator is used at higher speeds, while the slow estimator
becomes more influential at low speeds. A smooth interpolation is used
between the two estimates.
*/
void UpdateVelocityEstimate(void);

/**


@brief Returns the currently selected encoder velocity estimate.
@return Encoder velocity in revolutions per second (RPS).
*/
float Encoder_GetVelocity(void);

/**


@brief Returns the fast encoder velocity estimate.
@return Fast velocity estimate in revolutions per second (RPS).
*/
float Encoder_GetFastVelocity(void);

/**


@brief Returns the slow encoder velocity estimate.
@return Slow velocity estimate in revolutions per second (RPS).
*/
float Encoder_GetSlowVelocity(void);

/**


@brief Returns the current encoder counter value.
@return Current encoder timer counter value.
*/
int32_t Encoder_GetCount(void);

#endif /* ENCODER_H */
