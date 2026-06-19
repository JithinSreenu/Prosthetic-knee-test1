/**
 * @file    project_config.h
 * @brief   Global project configuration for the STM32H585 Prosthetic Knee
 *          Controller. Centralizes pin assignments, peripheral handles,
 *          and system-wide constants so every module agrees on the same
 *          hardware map.
 *
 * @note    ASSUMPTION: This file assumes a CubeMX-generated project where
 *          MX_GPIO_Init(), MX_USART1_UART_Init(), MX_USART2_UART_Init(),
 *          MX_TIM3_Init() (motor PWM), MX_GPDMA1_Init(), MX_ADC1_Init(),
 *          MX_WWDG_Init() (or IWDG) and MX_FREERTOS_Init() have already
 *          been generated in main.c. Adjust handle names below to match
 *          your actual .ioc if they differ.
 */

#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ======================================================================
 *  PERIPHERAL HANDLES (declared extern; defined in main.c by CubeMX)
 * ====================================================================== */
extern UART_HandleTypeDef huart1;   /* Bluetooth link            */
extern UART_HandleTypeDef huart2;   /* Main controller link      */
extern TIM_HandleTypeDef  htim3;    /* Motor PWM (CH1 = Motor1, CH2 = Motor2) */
extern ADC_HandleTypeDef  hadc1;    /* Battery voltage monitor   */
extern WWDG_HandleTypeDef hwwdg1;   /* Watchdog                  */

/* ======================================================================
 *  GPIO PIN MAP
 *  Adjust these to match your schematic / CubeMX pinout. Defined here
 *  so no .c file hardcodes a port/pin literal.
 * ====================================================================== */

/* --- HX711 Load Cell amplifier (bit-banged 2-wire protocol) --- */
#define HX711_SCK_PORT          GPIOA
#define HX711_SCK_PIN           GPIO_PIN_0
#define HX711_DOUT_PORT         GPIOA
#define HX711_DOUT_PIN          GPIO_PIN_1

/* --- Motor 1: Hydraulic valve OPEN driver (e.g. DRV8871) --- */
#define MOTOR1_DIR_PORT         GPIOB
#define MOTOR1_DIR_PIN          GPIO_PIN_0
#define MOTOR1_PWM_TIM_CHANNEL  TIM_CHANNEL_1

/* --- Motor 2: Hydraulic valve CLOSE driver --- */
#define MOTOR2_DIR_PORT         GPIOB
#define MOTOR2_DIR_PIN          GPIO_PIN_1
#define MOTOR2_PWM_TIM_CHANNEL  TIM_CHANNEL_2

/* --- Motor driver fault/diagnostic feedback (active low, typical DRV887x) --- */
#define MOTOR_FAULT_PORT        GPIOB
#define MOTOR_FAULT_PIN         GPIO_PIN_2

/* --- Warning indicator (LED / buzzer driver transistor) --- */
#define WARNING_LED_PORT        GPIOC
#define WARNING_LED_PIN         GPIO_PIN_13

/* ======================================================================
 *  SYSTEM CONSTANTS
 * ====================================================================== */
#define UART_RX_BUFFER_SIZE     64U     /* per-port DMA circular RX buffer */
#define PACKET_MAX_SIZE         5U      /* Header,Command,Value(2),CRC     */

#define COMM_TIMEOUT_MS         100U    /* Safety: max silence from controller */
#define WATCHDOG_REFRESH_MS     100U
#define LOADCELL_TASK_PERIOD_MS 5U
#define MOTOR_TASK_PERIOD_MS    1U
#define COMM_TASK_PERIOD_MS     10U
#define BLE_TASK_PERIOD_MS      50U
#define SAFETY_TASK_PERIOD_MS   1U

#define BATTERY_LOW_VOLTAGE_MV  6000U   /* example threshold, tune to pack */

#ifdef __cplusplus
}
#endif

#endif /* PROJECT_CONFIG_H */
