/*
 * motor.c - L298N dual motor driver for the Bipedal Wheeled Robot board
 *
 * Requires these CubeMX settings (see setup instructions):
 *   TIM1 CH1 on PA8 and CH2 on PA9 in "PWM Generation" mode
 *   PA3, PA4, PB11, PB12 as GPIO_Output with user labels IN1, IN2, IN3, IN4
 *
 * L298N truth table (per motor):
 *   EN = PWM, IN_a = 1, IN_b = 0  -> forward
 *   EN = PWM, IN_a = 0, IN_b = 1  -> reverse
 *   EN = 1,   IN_a = IN_b         -> brake (fast stop)
 *   EN = 0                        -> coast (free-wheel)
 */
#include "motor.h"
#include "main.h"   /* HAL, plus the IN1_Pin / IN1_GPIO_Port etc. macros from CubeMX */

extern TIM_HandleTypeDef htim1;

typedef struct {
    GPIO_TypeDef *in_a_port;
    uint16_t      in_a_pin;
    GPIO_TypeDef *in_b_port;
    uint16_t      in_b_pin;
    uint32_t      pwm_channel;
} motor_hw_t;

static const motor_hw_t motors[2] = {
    /* MOTOR_A: ENA = PA8 / TIM1_CH1 */
    { IN1_GPIO_Port, IN1_Pin, IN2_GPIO_Port, IN2_Pin, TIM_CHANNEL_1 },
    /* MOTOR_B: ENB = PA9 / TIM1_CH2 */
    { IN3_GPIO_Port, IN3_Pin, IN4_GPIO_Port, IN4_Pin, TIM_CHANNEL_2 },
};

/* Convert per-mille (0..1000) into a timer compare value. */
static uint32_t duty_to_compare(uint32_t permille)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
    return (period * permille) / 1000U;
}

static void set_duty(motor_id_t m, uint32_t permille)
{
    __HAL_TIM_SET_COMPARE(&htim1, motors[m].pwm_channel, duty_to_compare(permille));
}

static void set_dir_pins(motor_id_t m, GPIO_PinState a, GPIO_PinState b)
{
    HAL_GPIO_WritePin(motors[m].in_a_port, motors[m].in_a_pin, a);
    HAL_GPIO_WritePin(motors[m].in_b_port, motors[m].in_b_pin, b);
}

void Motor_Init(void)
{
    for (int i = 0; i < 2; i++) {
        set_dir_pins((motor_id_t)i, GPIO_PIN_RESET, GPIO_PIN_RESET);
        set_duty((motor_id_t)i, 0);
    }
    /* For TIM1 (an advanced timer) HAL also sets the main output enable (MOE). */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void Motor_SetSpeed(motor_id_t m, int16_t speed)
{
    if (m > MOTOR_B) return;

    if (speed >  MOTOR_SPEED_MAX) speed =  MOTOR_SPEED_MAX;
    if (speed < -MOTOR_SPEED_MAX) speed = -MOTOR_SPEED_MAX;

    uint32_t mag = (uint32_t)(speed < 0 ? -speed : speed);
    /* Scale 0..1000 down to 0..MOTOR_DUTY_LIMIT */
    uint32_t duty = (mag * MOTOR_DUTY_LIMIT) / MOTOR_SPEED_MAX;

    if (speed > 0) {
        set_dir_pins(m, GPIO_PIN_SET, GPIO_PIN_RESET);
    } else if (speed < 0) {
        set_dir_pins(m, GPIO_PIN_RESET, GPIO_PIN_SET);
    } else {
        set_dir_pins(m, GPIO_PIN_RESET, GPIO_PIN_RESET);
    }
    set_duty(m, duty);
}

void Motor_Brake(motor_id_t m)
{
    if (m > MOTOR_B) return;
    set_dir_pins(m, GPIO_PIN_RESET, GPIO_PIN_RESET);
    set_duty(m, 1000);   /* EN fully on with IN_a == IN_b shorts the motor */
}

void Motor_Coast(motor_id_t m)
{
    if (m > MOTOR_B) return;
    set_duty(m, 0);      /* EN low: driver outputs off */
    set_dir_pins(m, GPIO_PIN_RESET, GPIO_PIN_RESET);
}

void Motor_StopAll(void)
{
    Motor_Coast(MOTOR_A);
    Motor_Coast(MOTOR_B);
}
