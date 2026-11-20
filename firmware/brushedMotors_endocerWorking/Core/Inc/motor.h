/*
 * motor.h - L298N dual motor driver for the Bipedal Wheeled Robot board
 *
 * Board connections (from robot_v2 schematic, connector P6):
 *   Motor A: ENA = PA8 (TIM1_CH1 PWM), IN1 = PA3, IN2 = PA4
 *   Motor B: ENB = PA9 (TIM1_CH2 PWM), IN3 = PB11, IN4 = PB12
 *
 * Speed is given in per-mille: -1000 (full reverse) .. 0 .. +1000 (full forward).
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_A = 0,   /* L298N OUT1/OUT2, controlled by ENA/IN1/IN2 */
    MOTOR_B = 1    /* L298N OUT3/OUT4, controlled by ENB/IN3/IN4 */
} motor_id_t;

#define MOTOR_SPEED_MAX 1000

/*
 * Safety cap on PWM duty, in per-mille of full scale.
 * The L298N is fed from the board's +24V rail. If your motors are rated
 * for 12 V, keep this at 500 (about 12 V average). Raise it only if
 * your motors are rated for 24 V.
 */
#define MOTOR_DUTY_LIMIT 1000

void Motor_Init(void);                              /* call once after MX_TIM1_Init() */
void Motor_SetSpeed(motor_id_t m, int16_t speed);   /* -1000..+1000 */
void Motor_Brake(motor_id_t m);                     /* active stop (motor shorted) */
void Motor_Coast(motor_id_t m);                     /* free-wheel (driver disabled) */
void Motor_StopAll(void);                           /* coast both motors */

#endif /* MOTOR_H */
