/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

#define IOCON_PIO_CLKDIV0 0x00u       /*!<@brief IOCONCLKDIV0 */
#define IOCON_PIO_HYS_EN 0x20u        /*!<@brief Enable hysteresis */
#define IOCON_PIO_INV_DI 0x00u        /*!<@brief Input not invert */
#define IOCON_PIO_MODE_PULLDOWN 0x08u /*!<@brief Selects pull-down function */
#define IOCON_PIO_MODE_PULLUP 0x10u   /*!<@brief Selects pull-up function */
#define IOCON_PIO_OD_DI 0x00u         /*!<@brief Disables Open-drain function */
#define IOCON_PIO_SMODE_BYPASS 0x00u  /*!<@brief Bypass input filter */

/*! @name PIO0_24 (number 28), LED_BLUE
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_LED_BLUE_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_LED_BLUE_GPIO_PIN_MASK (1U << 24U) /*!<@brief GPIO pin mask */
#define BOARD_LED_BLUE_PORT 0U                   /*!<@brief PORT device index: 0 */
#define BOARD_LED_BLUE_PIN 24U                   /*!<@brief PORT pin number */
#define BOARD_LED_BLUE_PIN_MASK (1U << 24U)      /*!<@brief PORT pin mask */
                                                 /* @} */

/*! @name PIO0_25 (number 27), LED_GREEN
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_LED_GREEN_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_LED_GREEN_GPIO_PIN_MASK (1U << 25U) /*!<@brief GPIO pin mask */
#define BOARD_LED_GREEN_PORT 0U                   /*!<@brief PORT device index: 0 */
#define BOARD_LED_GREEN_PIN 25U                   /*!<@brief PORT pin number */
#define BOARD_LED_GREEN_PIN_MASK (1U << 25U)      /*!<@brief PORT pin mask */
                                                  /* @} */

/*! @name PIO0_26 (number 23), LED_RED
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_LED_RED_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_LED_RED_GPIO_PIN_MASK (1U << 26U) /*!<@brief GPIO pin mask */
#define BOARD_LED_RED_PORT 0U                   /*!<@brief PORT device index: 0 */
#define BOARD_LED_RED_PIN 26U                   /*!<@brief PORT pin number */
#define BOARD_LED_RED_PIN_MASK (1U << 26U)      /*!<@brief PORT pin mask */
                                                /* @} */

/*! @name PIO1_19 (number 44), NTC_POWER
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_NTC_POWER_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_NTC_POWER_GPIO_PIN_MASK (1U << 19U) /*!<@brief GPIO pin mask */
#define BOARD_NTC_POWER_PORT 1U                   /*!<@brief PORT device index: 1 */
#define BOARD_NTC_POWER_PIN 19U                   /*!<@brief PORT pin number */
#define BOARD_NTC_POWER_PIN_MASK (1U << 19U)      /*!<@brief PORT pin mask */
                                                  /* @} */

/*! @name PIO1_20 (number 56), NTC_GND
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_NTC_GND_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_NTC_GND_GPIO_PIN_MASK (1U << 20U) /*!<@brief GPIO pin mask */
#define BOARD_NTC_GND_PORT 1U                   /*!<@brief PORT device index: 1 */
#define BOARD_NTC_GND_PIN 20U                   /*!<@brief PORT pin number */
#define BOARD_NTC_GND_PIN_MASK (1U << 20U)      /*!<@brief PORT pin mask */
                                                /* @} */

/*! @name PIO0_21 (number 57), SPI_CLK
  @{ */
#define BOARD_SPI_CLK_PORT 0U                   /*!<@brief PORT device index: 0 */
#define BOARD_SPI_CLK_PIN 21U                   /*!<@brief PORT pin number */
#define BOARD_SPI_CLK_PIN_MASK (1U << 21U)      /*!<@brief PORT pin mask */
                                                /* @} */

/*! @name PIO0_22 (number 55), SPI_MOSI
  @{ */
#define BOARD_SPI_MOSI_PORT 0U                   /*!<@brief PORT device index: 0 */
#define BOARD_SPI_MOSI_PIN 22U                   /*!<@brief PORT pin number */
#define BOARD_SPI_MOSI_PIN_MASK (1U << 22U)      /*!<@brief PORT pin mask */
                                                 /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void); /* Function assigned for the Cortex-M0P */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
