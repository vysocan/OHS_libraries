/**
 * Emit a pulse of precise length on a pin, using the RMT peripheral.
 */

#ifndef _RMT_PULSE_H_
#define _RMT_PULSE_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/***        include files                                                   ***/
/******************************************************************************/

#include <stdint.h>

#include <driver/gpio.h>
#include <esp_attr.h>

/******************************************************************************/
/***        macro definitions                                               ***/
/******************************************************************************/

/******************************************************************************/
/***        type definitions                                                ***/
/******************************************************************************/

/******************************************************************************/
/***        exported variables                                              ***/
/******************************************************************************/

/******************************************************************************/
/***        exported functions                                              ***/
/******************************************************************************/

/**
 * @brief Initializes RMT Channel 0 with a pin for RMT pulsing.
 *
 * @note The pin will have to be re-initialized if subsequently used as GPIO.
 */
void rmt_pulse_init(gpio_num_t pin);

/**
 * @brief Outputs a single pulse (high -> low) on the configured pin.
 *
 * @note This function will always wait for a previous call to finish.
 *
 * @param high_time_us Pulse high time in us.
 * @param low_time_us  Pulse low time in us.
 * @param wait         Block until the pulse is finished.
 */
void IRAM_ATTR pulse_ckv_us(uint16_t high_time_us, uint16_t low_time_us, bool wait);

/**
 * @brief Indicates if the rmt is currently sending a pulse.
 */
bool IRAM_ATTR rmt_busy();

/**
 * @brief Outputs a single pulse (high -> low) on the configured pin.
 *
 * @note This function will always wait for a previous call to finish.
 *
 * @param high_time_us Pulse high time clock ticks.
 * @param low_time_us  Pulse low time in clock ticks.
 * @param wait         Block until the pulse is finished.
 */
void IRAM_ATTR pulse_ckv_ticks(uint16_t high_time_us, uint16_t low_time_us, bool wait);

#ifdef __cplusplus
}
#endif

#endif
/******************************************************************************/
/***        END OF FILE                                                     ***/
/******************************************************************************/