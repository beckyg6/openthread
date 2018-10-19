/*
 *  Copyright (c) 2016, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * @brief
 *   This file defines the platform-specific functions needed by OpenThread's example applications.
 */

#ifndef OPENTHREAD_SYSTEM_H_
#define OPENTHREAD_SYSTEM_H_

#include <stdint.h>

#include <openthread/instance.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * This function performs all platform-specific initialization of OpenThread's drivers.
 *
 * @note This function is not called by the OpenThread library. Instead, the system/RTOS should call this function
 *       when initialization of OpenThread's drivers is most appropriate.
 *
 * @param[in]  argc  Number of arguments in @p argv.
 * @param[in]  argv  Argument vector.
 *
 */
void otSysInit(int argc, char *argv[]);

/**
 * This function performs all platform-specific deinitialization for OpenThread's drivers.
 *
 * @note This function is not called by the OpenThread library. Instead, the system/RTOS should call this function
 *       when deinitialization of OpenThread's drivers is most appropriate.
 *
 */
void otSysDeinit(void);

/**
 * This function returns true if a pseudo-reset was requested.
 *
 * In such a case, the main loop should shut down and re-initialize the OpenThread instance.
 *
 * @note This function is not called by the OpenThread library. Instead, the system/RTOS should call this function
 *       in the main loop to determine when to shut down and re-initialize the OpenThread instance.
 *
 */
bool otSysPseudoResetWasRequested(void);

/**
 * This function performs all platform-specific processing for OpenThread's example applications.
 *
 * @note This function is not called by the OpenThread library. Instead, the system/RTOS should call this function
 *       in the main loop when processing OpenThread's drivers is most appropriate.
 *
 * @param[in]  aInstance  The OpenThread instance structure.
 *
 */
void otSysProcessDrivers(otInstance *aInstance);

/**
 * This function is called whenever platform drivers needs processing.
 *
 * @note This function is not handled by the OpenThread library. Instead, the system/RTOS should handle this function
 *       and schedule a call to `otSysProcessDrivers()`.
 *
 */
extern void otSysEventSignalPending(void);

/**
 * @def OPENTHREAD_GPIO_LOGIC_LEVEL
 *
 * logic 1: LEDs are turned on by outputing a high GPIO level(other side is GND);
 * logic 0: LEDs are turned on by outputing a low GPIO level(other side is VDD);
 *
 * defining this macro is to configure which kind of method will be used.
 */
#ifndef OPENTHREAD_GPIO_LOGIC_LEVEL
#define OPENTHREAD_GPIO_LOGIC_LEVEL 0
#endif

/**
 * @def OPENTHREAD_EXAMPLES_PLATFROMS
 */

#ifdef OPENTHREAD_EXAMPLES_NRF52840
#undef PACKAGE

/**
 *  OPENTHREAD_GPIO_LOGIC_LEVEL == 1
 *
 *  original demo values:
 *  @mapping gpio pins with RGB control
 *
 *  Red:   P1.13  < - > D11
 *  Green: P1.14  < - > D12
 *  Blue:  P1.15  < - > D13
 *         GDN    // choose any available
 */

/**
 *  OPENTHREAD_GPIO_LOGIC_LEVEL == 0
 *
 *  @mapping gpio pins with onboard LEDs and buttons
 *  To be used initially for new c/l
 *
 *  LED1 (green) = P0.13
 *  LED2 (green) = P0.14
 *  LED3 (green) = P0.15
 *  LED4 (green) = P0.16
 *  BUTTON1 = SW1 = P0.11
 *  BUTTON2 = SW2 = P0.12
 *  BUTTON3 = SW3 = P0.24
 *  BUTTON4 = SW4 = P0.25
 *  BOOT = SW5 = boot/reset
 *
 *  original demo values:
 *  @mapping gpio pins with RGB control
 *
 *  Red:   P1.13  < - > D11
 *  Green: P1.14  < - > D12
 *  Blue:  P1.15  < - > D13
 *         VDD    // choose any available
 */

#define LED_GPIO_PORT 0x50000300UL
#define BUTTON_GPIO_PORT 0x50000300UL

#define BUTTON_1_PIN 11 // 11 + 0 (NRF_P0) // button #1
#define BUTTON_2_PIN 12 // 12 + 0 (NRF_P0) // button #2
#define LED_1_PIN 13    // 13 + 0 (NRF_P0) // leader role
#define LED_2_PIN 14    // 14 + 0 (NRF_P0) // router role
#define LED_3_PIN 15    // 15 + 0 (NRF_P0) // child role
#define LED_4_PIN 16    // 16 + 0 (NRF_P0) // icmp6 ping

#endif // OPENTHREAD_EXAMPLES_NRF52840

#if (OPENTHREAD_GPIO_LOGIC_LEVEL == 1)
enum
{
    GPIO_LOGIC_HIGH = 1,
    GPIO_LOGIC_LOW  = 0,
};
#else
enum
{
    GPIO_LOGIC_HIGH = 0,
    GPIO_LOGIC_LOW  = 1,
};
#endif

/**
 * @defgroup gpio GPIO
 * @ingroup platform
 *
 * @brief
 *   This module includes the platform abstraction to GPIO.
 *
 * @{
 *
 */

/**
 * Init GPIO module.
 *
 */
void otSysGpioInit(void);

/**
 * Set logic high for output pin.
 *
 */
void otSysGpioOutSet(uint32_t port, uint8_t pin);

/**
 * Set logic low for output pin.
 *
 */
void otSysGpioOutClear(uint32_t port, uint8_t pin);

/**
 * Blink(toggle) output pin for a specified count (num_blinks).
 *
 */
void otSysGpioOutBlink(uint32_t port, uint8_t pin, uint8_t num_blinks);

/**
 * Toggle output pin.
 *
 */
void otSysGpioOutToggle(uint32_t port, uint8_t pin);

/**
 * Read the value of output pin.
 *
 */
uint8_t otSysGpioOutGet(uint32_t port, uint8_t pin);

/**
 * A callback will be called when GPIO interrupt occurs.
 *
 */
typedef void (*otSysGpioIntCallback)(otInstance *aInstance);

/**
 * Register a callback for GPIO interrupt.
 *
 */
void otSysGpioRegisterCallback(uint32_t port, uint8_t pin, otSysGpioIntCallback aCallback, otInstance *aInstance);

/**
 * Enable GPIO interrupt.
 *
 */
void otSysGpioIntEnable(uint32_t port, uint8_t pin);

/**
 * Clear GPIO interrupt.
 *
 */
void otSysGpioIntClear(uint32_t port, uint8_t pin);

/**
 * @}
 *
 */

#ifdef __cplusplus
} // end of extern "C"
#endif

#endif // OPENTHREAD_SYSTEM_H_
