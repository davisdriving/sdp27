/* robstride.h - minimal driver for RobStride 01 actuators (private CAN protocol).
 *
 * Protocol source: RobStride 01 product manual (English part),
 *   section 4      "Driver Communication Protocol" (printed p.54): CAN 2.0, 1 Mbps,
 *                  29-bit extended ID = [28:24] type | [23:8] data area 2 | [7:0] target
 *   4.1.2  type 1  operation control (MIT-style) command   (p.55-56)
 *   4.1.3  type 2  motor feedback frame                     (p.56-57)
 *   4.1.4  type 3  enable                                   (p.57)
 *   4.1.5  type 4  stop (back to Reset mode)                (p.58)
 *   4.1.8  type 17 read one parameter                       (p.59-60)
 *   4.1.9  type 18 write one parameter (not saved)          (p.60)
 *   4.1.11 parameter list: 0x7005 run_mode, 0x7019 MechPos  (p.61-62)
 */
#ifndef ROBSTRIDE_H
#define ROBSTRIDE_H

#include "main.h"      /* pulls in stm32g4xx_hal.h, which has the FDCAN HAL */
#include <stdbool.h>
#include <stdint.h>

#define RS_HOST_ID        0xFDu    /* our node ID; the manual's examples use 0xFD */
#define RS_PARAM_RUN_MODE 0x7005u  /* 0 = operation control mode */
#define RS_PARAM_MECHPOS  0x7019u  /* output-shaft angle, float, radians */

/* Motor feedback (type 2) */
typedef struct {
  uint8_t id;          /* which motor sent it */
  uint8_t faults;      /* bits 21-16 of the ID; 0 = no fault */
  uint8_t mode;        /* 0 Reset, 1 Calibration, 2 Run */
  float   pos_rad;     /* output angle, -4pi..4pi */
  float   vel_rad_s;   /* -44..44 rad/s */
  float   torque_nm;   /* -17..17 Nm */
  float   temp_c;
} RS_Feedback;

/* --- reading --- */
HAL_StatusTypeDef RS_RequestParam(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                                  uint16_t index);
bool RS_GetParamReply(FDCAN_HandleTypeDef *hfdcan, uint8_t *motor_id,
                      uint16_t *index, float *value);
/* Pop frames until a type-2 feedback frame is found; false if FIFO empties. */
bool RS_GetFeedback(FDCAN_HandleTypeDef *hfdcan, RS_Feedback *fb);

/* --- commanding (each one makes the motor answer with a type-2 frame) --- */
HAL_StatusTypeDef RS_Stop(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
HAL_StatusTypeDef RS_Enable(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
HAL_StatusTypeDef RS_SetRunMode(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                                uint8_t mode);
/* torque = kp*(pos - actual) + kd*(vel - actual_vel) + torque_ff */
HAL_StatusTypeDef RS_Control(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                             float pos_rad, float vel_rad_s, float kp, float kd,
                             float torque_ff_nm);

#define RS_PARAM_LIMIT_TORQUE 0x700Bu
HAL_StatusTypeDef RS_WriteFloatParam(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                                    uint16_t index, float value);

#endif /* ROBSTRIDE_H */
