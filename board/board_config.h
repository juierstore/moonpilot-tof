/****************************************************************************
 *
 *   Copyright (c) 2025 MoonPilot / PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * MoonPilot flight controller (STM32H743VIH6, TFBGA100) - board internal definitions.
 *
 * Pin assignment is taken from the MoonPilot netlist; see docs/moonpilot_px4_pinmap.md.
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* LEDs
 *
 * Both LEDs are wired with the cathode to GND and the anode through a 1k resistor
 * to the MCU pin, so they are active HIGH (opposite of the MatekH743 reference design).
 */

#define GPIO_nLED_BLUE          /* PE4 (blue)  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)
#define GPIO_nLED_GREEN         /* PE3 (white) */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN3)

#define BOARD_LED_ON            1
#define BOARD_LED_OFF           0

#define BOARD_HAS_CONTROL_STATUS_LEDS   1
#define BOARD_ARMED_STATE_LED           1 // Green (white) LED
#define BOARD_OVERLOAD_LED              0 // Blue LED

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the PX4 Firmware in the adc driver
 */

#define ADC1_CH(n)                  (n)

/* Define GPIO pins used as ADC N.B. Channel numbers must match below  */
#define PX4_ADC_GPIO  \
	/* PC0  */  GPIO_ADC123_INP10, \
	/* PC1  */  GPIO_ADC123_INP11, \
	/* PA4  */  GPIO_ADC12_INP18, \
	/* PA7  */  GPIO_ADC12_INP7, \
	/* PC4  */  GPIO_ADC12_INP4, \
	/* PC5  */  GPIO_ADC12_INP8

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_BATTERY_VOLTAGE_CHANNEL     /* PC0  */  ADC1_CH(10)
#define ADC_BATTERY_CURRENT_CHANNEL     /* PC1  */  ADC1_CH(11)
#define ADC_BATTERY2_VOLTAGE_CHANNEL    /* PA4  */  ADC1_CH(18)
#define ADC_BATTERY2_CURRENT_CHANNEL    /* PA7  */  ADC1_CH(7)
#define ADC_AIRSPEED_IN_CHANNEL         /* PC4  */  ADC1_CH(4)
#define ADC_RSSI_IN_CHANNEL             /* PC5  */  ADC1_CH(8)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL) | \
	 (1 << ADC_BATTERY2_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY2_CURRENT_CHANNEL) | \
	 (1 << ADC_AIRSPEED_IN_CHANNEL) | \
	 (1 << ADC_RSSI_IN_CHANNEL))


/* Battery voltage divider: R22 10k / R25 1k (bat1), R23 20k / R26 1k (bat2) */
#define BOARD_BATTERY1_V_DIV         (11.0f)
#define BOARD_BATTERY1_A_PER_V       (40.0f)   /* depends on the 4in1 ESC current output */
#define BOARD_BATTERY2_V_DIV         (21.0f)
#define BOARD_BATTERY2_A_PER_V       (40.0f)   /* depends on the 4in1 ESC current output */


/* CAN Silence
 *
 * Silent mode control \ ESC Mux select
 */

#define GPIO_CAN1_SILENT_S0  /* PD3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN3)


/* PWM: 8 outputs
 *
 *   channel 1  PB0  TIM3_CH3    U17.1 (S1)
 *   channel 2  PB1  TIM3_CH4    U17.2 (S2)
 *   channel 3  PA0  TIM5_CH1    U17.3 (S3)
 *   channel 4  PA1  TIM5_CH2    U17.4 (S4)
 *   channel 5  PA2  TIM5_CH3    U18.1 (S5)
 *   channel 6  PA3  TIM5_CH4    U18.2 (S6)
 *   channel 7  PD12 TIM4_CH1    U18.3 (S7)
 *   channel 8  PD13 TIM4_CH2    U18.4 (S8)
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   8
#define DIRECT_INPUT_TIMER_CHANNELS  8

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_NUM_IO_TIMERS 3


/* Tone alarm output (PA15 -> R52 -> Q1 NPN -> buzzer, active HIGH) */

#define GPIO_TONE_ALARM_IDLE    /* PA15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)
#define GPIO_TONE_ALARM_GPIO    /* PA15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)


/* USB OTG FS
 *
 * This board has no VBUS sense pin (VBUS is only used to feed the 5V rail),
 * so GPIO_OTGFS_VBUS must not be defined.
 */


/* High-resolution timer */
#define HRT_TIMER               2  /* use timer2 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */


/* RC Serial port (USART6 -> U15, /dev/ttyS4) */
#define RC_SERIAL_PORT          "/dev/ttyS4"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT


/* SD Card (SDMMC1, no card detect pin) */
#define SDIO_SLOTNO             0  /* Only one slot */
#define SDIO_MINOR              0

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1


#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO, \
		GPIO_CAN1_TX, \
		GPIO_CAN1_RX, \
		GPIO_CAN1_SILENT_S0, \
		GPIO_nLED_BLUE, \
		GPIO_nLED_GREEN, \
		GPIO_TONE_ALARM_IDLE, \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER


__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
