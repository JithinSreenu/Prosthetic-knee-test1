/**
 * @file    motor.c
 * @brief   Implementation of hydraulic valve motor control via TIM3 PWM.
 */

#include "motor.h"

/* Proportional control gain for angle-error -> duty-cycle mapping.
 * Tune on the bench: too high -> oscillation/overshoot, too low ->
 * sluggish tracking. Expressed as duty-percent per degree of error. */
#define ANGLE_CONTROL_KP            2.0f
#define ANGLE_DEADBAND_DECIDEG      20   /* +/-2.0 degrees: don't chatter the valve for tiny errors */
#define MAX_DUTY_PCT                100U

void Motor_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = MOTOR1_DIR_PIN;
    HAL_GPIO_Init(MOTOR1_DIR_PORT, &gpio);
    HAL_GPIO_WritePin(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN, GPIO_PIN_RESET);

    gpio.Pin = MOTOR2_DIR_PIN;
    HAL_GPIO_Init(MOTOR2_DIR_PORT, &gpio);
    HAL_GPIO_WritePin(MOTOR2_DIR_PORT, MOTOR2_DIR_PIN, GPIO_PIN_RESET);

    /* Fault feedback input, assumed active-low open-drain output from the
     * driver IC, hence pulled up here. */
    gpio.Pin  = MOTOR_FAULT_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MOTOR_FAULT_PORT, &gpio);

    /* Start both PWM channels at 0% duty, but running, so the Motor task
     * only needs to update the compare register thereafter rather than
     * repeatedly starting/stopping the timer channel. */
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR1_PWM_TIM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim3, MOTOR2_PWM_TIM_CHANNEL, 0);
    HAL_TIM_PWM_Start(&htim3, MOTOR1_PWM_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&htim3, MOTOR2_PWM_TIM_CHANNEL);
}

static void SetDuty(uint32_t channel, uint8_t dutyPct)
{
    if (dutyPct > MAX_DUTY_PCT)
    {
        dutyPct = MAX_DUTY_PCT;
    }

    /* ARR (auto-reload register) defines the PWM period in timer ticks;
     * the compare register value for a given duty% is ARR * duty/100.
     * Reading htim3.Init.Period here (rather than hardcoding it) keeps
     * this correct regardless of the exact PWM frequency chosen in
     * CubeMX. */
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t compare = (arr * dutyPct) / 100U;
    __HAL_TIM_SET_COMPARE(&htim3, channel, compare);
}

void Motor_Drive(MotorId_t motor, uint8_t dutyPct, bool forward)
{
    if (motor == MOTOR_1_OPEN_VALVE)
    {
        HAL_GPIO_WritePin(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
        SetDuty(MOTOR1_PWM_TIM_CHANNEL, dutyPct);
    }
    else if (motor == MOTOR_2_CLOSE_VALVE)
    {
        HAL_GPIO_WritePin(MOTOR2_DIR_PORT, MOTOR2_DIR_PIN, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
        SetDuty(MOTOR2_PWM_TIM_CHANNEL, dutyPct);
    }
}

void Motor_EmergencyStop(void)
{
    SetDuty(MOTOR1_PWM_TIM_CHANNEL, 0);
    SetDuty(MOTOR2_PWM_TIM_CHANNEL, 0);
    HAL_GPIO_WritePin(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR2_DIR_PORT, MOTOR2_DIR_PIN, GPIO_PIN_RESET);
}

bool Motor_HasDriverFault(void)
{
    /* Active-low fault line (DRV887x-style): pin reads low when a fault
     * (overcurrent, overtemp, undervoltage) is latched by the driver. */
    return HAL_GPIO_ReadPin(MOTOR_FAULT_PORT, MOTOR_FAULT_PIN) == GPIO_PIN_RESET;
}

void Motor_UpdateValveControl(int16_t currentAngleDeciDeg, int16_t targetAngleDeciDeg)
{
    int16_t error = targetAngleDeciDeg - currentAngleDeciDeg;

    if (error > -ANGLE_DEADBAND_DECIDEG && error < ANGLE_DEADBAND_DECIDEG)
    {
        /* Within deadband: hold position, don't actuate either valve
         * motor. This avoids constant micro-corrections that would wear
         * the actuators and waste battery for no functional benefit. */
        Motor_Drive(MOTOR_1_OPEN_VALVE, 0, true);
        Motor_Drive(MOTOR_2_CLOSE_VALVE, 0, true);
        return;
    }

    float errorDeg = (float)error / 10.0f;
    float rawDuty = errorDeg * ANGLE_CONTROL_KP;
    uint8_t duty = (uint8_t)(rawDuty < 0 ? -rawDuty : rawDuty);
    if (duty > MAX_DUTY_PCT)
    {
        duty = MAX_DUTY_PCT;
    }

    if (error > 0)
    {
        /* Need more flexion/angle -> open valve to let the joint move. */
        Motor_Drive(MOTOR_1_OPEN_VALVE, duty, true);
        Motor_Drive(MOTOR_2_CLOSE_VALVE, 0, true);
    }
    else
    {
        /* Need less angle -> close valve to resist/restrict motion. */
        Motor_Drive(MOTOR_2_CLOSE_VALVE, duty, true);
        Motor_Drive(MOTOR_1_OPEN_VALVE, 0, true);
    }
}
