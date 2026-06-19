/**
 * @file    motor.h
 * @brief   Hydraulic valve motor control: Motor 1 opens the valve (used
 *          during swing/leg-lift), Motor 2 closes it (used during stance/
 *          standing). Each motor is driven through a PWM speed signal
 *          plus a direction GPIO, suitable for a brushed-DC driver stage
 *          such as a DRV8871 or similar H-bridge IC.
 *
 * ASSUMPTION: TIM3 is configured in CubeMX with CH1 and CH2 as PWM
 * Generation outputs, sharing a period register, at a frequency suitable
 * for the chosen motor driver (commonly 10-20kHz to stay above the
 * audible range and within the driver IC's PWM input spec).
 */

#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "project_config.h"

typedef enum
{
    MOTOR_1_OPEN_VALVE  = 0,
    MOTOR_2_CLOSE_VALVE = 1
} MotorId_t;

/* Used by Safety's EnterSafeModeActions(): a short, bounded full-ish
 * duty pulse on the close-valve motor to drive the valve fully shut
 * before motor power is cut entirely. Bounded duration so a stuck valve
 * or jammed motor cannot be driven indefinitely even inside the safety
 * path itself. Tune pulse duration to your actual valve's travel time. */
#define MAX_DUTY_PCT_SAFE_CLOSE   80U
#define SAFE_CLOSE_VALVE_PULSE_MS 150U

/**
 * @brief Configure TIM3 PWM channels and direction GPIOs for both motors.
 *        Call once at startup, after MX_TIM3_Init().
 */
void Motor_Init(void);

/**
 * @brief Drive a motor at a given duty cycle and direction.
 *
 * @param motor    Which motor (valve-open or valve-close actuator).
 * @param dutyPct  0-100. 0 stops the motor (coasts; driver-dependent).
 * @param forward  Direction; mapping to "open more" vs "close more" is
 *                 driver/mechanism-specific -- wire up forward=true to
 *                 mean "increasing valve opening" for Motor 1 and
 *                 "increasing valve closure" for Motor 2 during bring-up.
 */
void Motor_Drive(MotorId_t motor, uint8_t dutyPct, bool forward);

/**
 * @brief Immediately stops both motors (duty = 0) and disables the PWM
 *        outputs. This is the function the Safety task calls when
 *        entering Safe Mode -- it does not merely set duty to zero, it
 *        also forces the direction pins low so a wedged H-bridge fault
 *        can't keep driving current through a single motor winding.
 */
void Motor_EmergencyStop(void);

/**
 * @brief Reads the motor driver's fault/diagnostic feedback pin.
 *        Polarity is driver-dependent; this assumes an active-low FAULT
 *        output (common on DRV887x-family ICs), so the function returns
 *        true (fault present) when the pin reads low.
 */
bool Motor_HasDriverFault(void);

/**
 * @brief High-level helper used by the Motor Control task: given a
 *        current and target knee angle, compute and apply the
 *        appropriate valve motor drive to move toward the target,
 *        implementing basic damping control rather than a hard on/off.
 *
 * This is a simple proportional control as a starting point -- a real
 * implementation would likely add velocity feedback and tunable gains
 * exposed via the calibration/config command path.
 */
void Motor_UpdateValveControl(int16_t currentAngleDeciDeg, int16_t targetAngleDeciDeg);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
