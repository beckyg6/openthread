/*
 *  Copyright (c) 2017, The OpenThread Authors.
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
 * @file:This file implements the OpenThread platform abstraction for GPIO
 * and GPIOTE.
 *
 */

#include "openthread-system.h"

#include <string.h>

#include <openthread/instance.h>

#include "platform-nrf5.h"
#include "hal/nrf_gpio.h"
#include "hal/nrf_gpiote.h"
#include "libraries/delay/nrf_delay.h"

#if defined(__GNUC__)
_Pragma("GCC diagnostic push")
_Pragma("GCC diagnostic ignored \"-Wreturn-type\"")
_Pragma("GCC diagnostic ignored \"-Wunused-parameter\"")
_Pragma("GCC diagnostic ignored \"-Wunused-but-set-parameter\"")
#endif
#include "nrfx/drivers/include/nrfx_gpiote.h"
#if defined(__GNUC__)
_Pragma("GCC diagnostic pop")
#endif

/* Declaring callback functions for buttons 1 and 2 only. */
otSysGpioIntCallback mHandler1;
otSysGpioIntCallback mHandler2;

/**
 * @brief Function to receive interrupt and call back function
 * set by the application for button 1.
 *
 */
void in_pin1_handler(uint32_t pin, nrf_gpiote_polarity_t action)
{
    (void)pin;
    (void)action;

    /* Clear button interrupt */
    otSysGpioIntClear(NRF_GPIOTE_EVENTS_PORT, pin);

    /* Call the Button 1 handler registered by the application */
    mHandler1(sInstance);
}

/**
 * @brief Function to receive interrupt and call back function
 * set by the application for button 2.
 *
 */
void in_pin2_handler(uint32_t pin, nrf_gpiote_polarity_t action)
{
    (void)pin;
    (void)action;

    /* Clear button interrupt */
    otSysGpioIntClear(NRF_GPIOTE_EVENTS_PORT, pin);

    /* Call the Button 2 handler registered by the application */
    mHandler2(sInstance);
}

/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
void otSysGpioInit(void)
{
    /* Configure GPIO mode: output */
    nrf_gpio_cfg_output(LED_1_PIN);
    nrf_gpio_cfg_output(LED_2_PIN);
    nrf_gpio_cfg_output(LED_3_PIN);
    nrf_gpio_cfg_output(LED_4_PIN);

    /* Clear all output first */
    nrf_gpio_pin_write(LED_1_PIN, GPIO_LOGIC_LOW);
    nrf_gpio_pin_write(LED_2_PIN, GPIO_LOGIC_LOW);
    nrf_gpio_pin_write(LED_3_PIN, GPIO_LOGIC_LOW);
    nrf_gpio_pin_write(LED_4_PIN, GPIO_LOGIC_LOW);

    /* Initialize gpiote for button(s) input.
     Button event handlers are registered in the application (main.c) */
    ret_code_t err_code;
    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Set the pin value. Turns the LED on.
 *
 */
void otSysGpioOutSet(uint32_t port, uint8_t pin)
{
    (void)port;

    nrf_gpio_pin_write(pin, GPIO_LOGIC_HIGH);
}

/**
 *  @brief Blink the LED a specified number of times.
 *  NOTE: to leave it with the previous value, use an even number.
 *
 */
void otSysGpioOutBlink(uint32_t port, uint8_t pin, uint8_t num_blinks)
{
    (void)port;

    for (int i = 0; i < num_blinks; i++)
    {
        nrf_gpio_pin_toggle((uint32_t)pin);
        nrf_delay_ms(300);
    }
}

/**
 *  @brief Clear the pin value. Turns the LED off.
 *
 */
void otSysGpioOutClear(uint32_t port, uint8_t pin)
{
    (void)port;

    nrf_gpio_pin_write(pin, GPIO_LOGIC_LOW);
}

/**
 * @brief Toggle the pin value.
 *
 */
void otSysGpioOutToggle(uint32_t port, uint8_t pin)
{
    (void)port;

    nrf_gpio_pin_toggle((uint32_t)pin);
}

/**
 * @brief Read the value of the pin.
 *
 */
uint8_t otSysGpioOutGet(uint32_t port, uint8_t pin)
{
    (void)port;

    uint8_t rval = nrf_gpio_pin_out_read((uint32_t)pin);

#if (OPENTHREAD_GPIO_LOGIC_LEVEL == 1)
    return rval > 0 ? GPIO_LOGIC_HIGH : GPIO_LOGIC_LOW;
#else
    return rval > 0 ? GPIO_LOGIC_LOW : GPIO_LOGIC_HIGH;
#endif
}

/**
 * @brief Register a callback for GPIO interrupt.
 *
 */
void otSysGpioRegisterCallback(uint32_t port, uint8_t pin, otSysGpioIntCallback aCallback, otInstance *aInstance)
{
    (void)port;
    (void)aInstance;

    nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull                    = NRF_GPIO_PIN_PULLUP;

    /* Check if button 1 or 2 is getting registered - not using 3 and 4 */
    if (pin == BUTTON_1_PIN)
    {
        ret_code_t err_code;
        mHandler1 = aCallback;
        err_code  = nrfx_gpiote_in_init(pin, &in_config, in_pin1_handler);
        APP_ERROR_CHECK(err_code);
    }
    else if (pin == BUTTON_2_PIN)
    {
        ret_code_t err_code;
        mHandler2 = aCallback;
        err_code  = nrfx_gpiote_in_init(pin, &in_config, in_pin2_handler);
        APP_ERROR_CHECK(err_code);
    }
}

/**
 * @brief Enable GPIO interrupt.
 *
 */
void otSysGpioIntEnable(uint32_t port, uint8_t pin)
{
    (void)port;

    nrfx_gpiote_in_event_enable((uint32_t)pin, true);
}

/**
 * @brief Clear GPIO interrupt.
 *
 */
void otSysGpioIntClear(uint32_t port, uint8_t pin)
{
    (void)port;
    (void)pin;

    nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_PORT);
}
