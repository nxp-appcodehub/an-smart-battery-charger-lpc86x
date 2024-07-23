/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v12.0
processor: LPC865
package_id: LPC865M201JBD64
mcu_data: ksdk2_0
processor_version: 0.12.3
pin_labels:
- {pin_num: '36', pin_signal: PIO1_16, label: I2C1_SDA, identifier: DEBUG_UART_RX}
- {pin_num: '37', pin_signal: PIO1_17, label: I2C1_SCL, identifier: DEBUG_UART_TX}
- {pin_num: '45', pin_signal: PIO0_7/ADC_0, label: ADC_CHN0}
- {pin_num: '21', pin_signal: PIO0_27, label: USART0_RXD}
- {pin_num: '19', pin_signal: PIO0_16, label: USART0_TXD}
- {pin_num: '63', pin_signal: PIO0_17/ADC_9, label: FTM0_CH0}
- {pin_num: '28', pin_signal: PIO0_24, label: LED_BLUE, identifier: LED_BLUE}
- {pin_num: '27', pin_signal: PIO0_25, label: LED_GREEN, identifier: LED_RED;LED_GREEN}
- {pin_num: '23', pin_signal: PIO0_26, label: LED_RED, identifier: LED_RED}
- {pin_num: '44', pin_signal: PIO1_19, label: NTC_POWER, identifier: NTC_POWER}
- {pin_num: '56', pin_signal: PIO1_20, label: NTC_GND, identifier: NTC_GND}
- {pin_num: '57', pin_signal: PIO0_21/ADC_5, label: SPI_CLK, identifier: SPI_CLK}
- {pin_num: '55', pin_signal: PIO0_22/ADC_4, label: SPI_MOSI, identifier: SPI_MOSI}
- {pin_num: '60', pin_signal: PIO0_19/ADC_7, label: LCD_DC, identifier: LCD_DC}
- {pin_num: '61', pin_signal: PIO0_18/ADC_8, label: LCD_BL, identifier: LCD_BL}
- {pin_num: '58', pin_signal: PIO0_20/ADC_6, label: LCD_CS, identifier: LCD_CS}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_iocon.h"
#include "fsl_swm.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '37', peripheral: I2C0, signal: SCL, pin_signal: PIO1_17, identifier: ''}
  - {pin_num: '36', peripheral: I2C0, signal: SDA, pin_signal: PIO1_16, identifier: ''}
  - {pin_num: '21', peripheral: USART0, signal: RXD, pin_signal: PIO0_27}
  - {pin_num: '19', peripheral: USART0, signal: TXD, pin_signal: PIO0_16}
  - {pin_num: '45', peripheral: ADC0, signal: 'CH, 0', pin_signal: PIO0_7/ADC_0}
  - {pin_num: '63', peripheral: FTM0, signal: 'CH, 0', pin_signal: PIO0_17/ADC_9}
  - {pin_num: '28', peripheral: GPIO, signal: 'PIO0, 24', pin_signal: PIO0_24, direction: OUTPUT, gpio_init_state: 'true', mode: pullUp, invert: disabled, hysteresis: enabled,
    opendrain: disabled, smode: bypass, clkdiv: div0}
  - {pin_num: '27', peripheral: GPIO, signal: 'PIO0, 25', pin_signal: PIO0_25, identifier: LED_GREEN, direction: OUTPUT, gpio_init_state: 'true', mode: pullUp, invert: disabled,
    hysteresis: enabled, opendrain: disabled, smode: bypass, clkdiv: div0}
  - {pin_num: '23', peripheral: GPIO, signal: 'PIO0, 26', pin_signal: PIO0_26, direction: OUTPUT, gpio_init_state: 'true', mode: pullUp, invert: disabled, hysteresis: enabled,
    opendrain: disabled, smode: bypass, clkdiv: div0}
  - {pin_num: '44', peripheral: GPIO, signal: 'PIO1, 19', pin_signal: PIO1_19, direction: OUTPUT, gpio_init_state: 'true', mode: pullUp, invert: disabled, hysteresis: enabled,
    opendrain: disabled, smode: bypass, clkdiv: div0}
  - {pin_num: '56', peripheral: GPIO, signal: 'PIO1, 20', pin_signal: PIO1_20, direction: OUTPUT, gpio_init_state: 'false', mode: pullDown, invert: disabled, hysteresis: enabled,
    opendrain: disabled, smode: bypass, clkdiv: div0}
  - {pin_num: '57', peripheral: SPI0, signal: SCK, pin_signal: PIO0_21/ADC_5}
  - {pin_num: '55', peripheral: SPI0, signal: MOSI, pin_signal: PIO0_22/ADC_4}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M0P */
void BOARD_InitPins(void)
{
    /* Enables clock for IOCON.: enable */
    CLOCK_EnableClock(kCLOCK_Iocon);
    /* Enables clock for switch matrix.: enable */
    CLOCK_EnableClock(kCLOCK_Swm);
    /* Enables the clock for the GPIO0 module */
    CLOCK_EnableClock(kCLOCK_Gpio0);
    /* Enables the clock for the GPIO1 module */
    CLOCK_EnableClock(kCLOCK_Gpio1);

    gpio_pin_config_t LED_BLUE_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U,
    };
    /* Initialize GPIO functionality on pin PIO0_24 (pin 28)  */
    GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PORT, BOARD_LED_BLUE_PIN, &LED_BLUE_config);

    gpio_pin_config_t LED_GREEN_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U,
    };
    /* Initialize GPIO functionality on pin PIO0_25 (pin 27)  */
    GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PORT, BOARD_LED_GREEN_PIN, &LED_GREEN_config);

    gpio_pin_config_t LED_RED_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U,
    };
    /* Initialize GPIO functionality on pin PIO0_26 (pin 23)  */
    GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_PORT, BOARD_LED_RED_PIN, &LED_RED_config);

    gpio_pin_config_t NTC_POWER_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U,
    };
    /* Initialize GPIO functionality on pin PIO1_19 (pin 44)  */
    GPIO_PinInit(BOARD_NTC_POWER_GPIO, BOARD_NTC_POWER_PORT, BOARD_NTC_POWER_PIN, &NTC_POWER_config);

    gpio_pin_config_t NTC_GND_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U,
    };
    /* Initialize GPIO functionality on pin PIO1_20 (pin 56)  */
    GPIO_PinInit(BOARD_NTC_GND_GPIO, BOARD_NTC_GND_PORT, BOARD_NTC_GND_PIN, &NTC_GND_config);

    const uint32_t LED_BLUE = (/* Selects pull-up function */
                               IOCON_PIO_MODE_PULLUP |
                               /* Enable hysteresis */
                               IOCON_PIO_HYS_EN |
                               /* Input not invert */
                               IOCON_PIO_INV_DI |
                               /* Disables Open-drain function */
                               IOCON_PIO_OD_DI |
                               /* Bypass input filter */
                               IOCON_PIO_SMODE_BYPASS |
                               /* IOCONCLKDIV0 */
                               IOCON_PIO_CLKDIV0);
    /* PIO0 PIN24 (coords: 28) is configured as GPIO, PIO0, 24. */
    IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO0_24, LED_BLUE);

    const uint32_t LED_GREEN = (/* Selects pull-up function */
                                IOCON_PIO_MODE_PULLUP |
                                /* Enable hysteresis */
                                IOCON_PIO_HYS_EN |
                                /* Input not invert */
                                IOCON_PIO_INV_DI |
                                /* Disables Open-drain function */
                                IOCON_PIO_OD_DI |
                                /* Bypass input filter */
                                IOCON_PIO_SMODE_BYPASS |
                                /* IOCONCLKDIV0 */
                                IOCON_PIO_CLKDIV0);
    /* PIO0 PIN25 (coords: 27) is configured as GPIO, PIO0, 25. */
    IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO0_25, LED_GREEN);

    const uint32_t LED_RED = (/* Selects pull-up function */
                              IOCON_PIO_MODE_PULLUP |
                              /* Enable hysteresis */
                              IOCON_PIO_HYS_EN |
                              /* Input not invert */
                              IOCON_PIO_INV_DI |
                              /* Disables Open-drain function */
                              IOCON_PIO_OD_DI |
                              /* Bypass input filter */
                              IOCON_PIO_SMODE_BYPASS |
                              /* IOCONCLKDIV0 */
                              IOCON_PIO_CLKDIV0);
    /* PIO0 PIN26 (coords: 23) is configured as GPIO, PIO0, 26. */
    IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO0_26, LED_RED);

    const uint32_t NTC_POWER = (/* Selects pull-up function */
                                IOCON_PIO_MODE_PULLUP |
                                /* Enable hysteresis */
                                IOCON_PIO_HYS_EN |
                                /* Input not invert */
                                IOCON_PIO_INV_DI |
                                /* Disables Open-drain function */
                                IOCON_PIO_OD_DI |
                                /* Bypass input filter */
                                IOCON_PIO_SMODE_BYPASS |
                                /* IOCONCLKDIV0 */
                                IOCON_PIO_CLKDIV0);
    /* PIO1 PIN19 (coords: 44) is configured as GPIO, PIO1, 19. */
    IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO1_19, NTC_POWER);

    const uint32_t NTC_GND = (/* Selects pull-down function */
                              IOCON_PIO_MODE_PULLDOWN |
                              /* Enable hysteresis */
                              IOCON_PIO_HYS_EN |
                              /* Input not invert */
                              IOCON_PIO_INV_DI |
                              /* Disables Open-drain function */
                              IOCON_PIO_OD_DI |
                              /* Bypass input filter */
                              IOCON_PIO_SMODE_BYPASS |
                              /* IOCONCLKDIV0 */
                              IOCON_PIO_CLKDIV0);
    /* PIO1 PIN20 (coords: 56) is configured as GPIO, PIO1, 20. */
    IOCON_PinMuxSet(IOCON, IOCON_INDEX_PIO1_20, NTC_GND);

    /* FTM0_CH0 connect to kSWM_FTM_Selection0 */
    SWM_SetFlextimerPinSelect(SWM0, kSWM_FTM0_CH0, kSWM_FTM_Selection0);

    /* USART0_TXD connect to P0_16 */
    SWM_SetMovablePinSelect(SWM0, kSWM_USART0_TXD, kSWM_PortPin_P0_16);

    /* USART0_RXD connect to P0_27 */
    SWM_SetMovablePinSelect(SWM0, kSWM_USART0_RXD, kSWM_PortPin_P0_27);

    /* SPI0_SCK connect to P0_21 */
    SWM_SetMovablePinSelect(SWM0, kSWM_SPI0_SCK, kSWM_PortPin_P0_21);

    /* SPI0_MOSI connect to P0_22 */
    SWM_SetMovablePinSelect(SWM0, kSWM_SPI0_MOSI, kSWM_PortPin_P0_22);

    /* I2C0_SDA connect to P1_16 */
    SWM_SetMovablePinSelect(SWM0, kSWM_I2C0_SDA, kSWM_PortPin_P1_16);

    /* I2C0_SCL connect to P1_17 */
    SWM_SetMovablePinSelect(SWM0, kSWM_I2C0_SCL, kSWM_PortPin_P1_17);

    /* ADC_CHN0 connect to P0_7 */
    SWM_SetFixedPinSelect(SWM0, kSWM_ADC_CHN0, true);

    /* Disable clock for switch matrix. */
    CLOCK_DisableClock(kCLOCK_Swm);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
