/* robstride.c - minimal driver for RobStride 01 actuators.
 * See robstride.h for the manual sections this follows. */
#include "robstride.h"
#include <string.h>

#define RS_TYPE_CONTROL     1u
#define RS_TYPE_FEEDBACK    2u
#define RS_TYPE_ENABLE      3u
#define RS_TYPE_STOP        4u
#define RS_TYPE_PARAM_READ  17u
#define RS_TYPE_PARAM_WRITE 18u

/* Value ranges of the 16-bit scaled fields (manual 4.1.2 / 4.1.3) */
#define RS_P_MAX   12.566371f   /* 4*pi rad */
#define RS_V_MAX   44.0f        /* rad/s */
#define RS_T_MAX   17.0f        /* Nm */
#define RS_KP_MAX  500.0f
#define RS_KD_MAX  5.0f

/* Map x in [lo, hi] to 0..65535 (clamped), as in the manual's example 4.2.1 */
static uint16_t f2u(float x, float lo, float hi)
{
  if (x > hi) x = hi;
  if (x < lo) x = lo;
  return (uint16_t)((x - lo) * 65535.0f / (hi - lo));
}

static float u2f(uint16_t u, float lo, float hi)
{
  return lo + (float)u * (hi - lo) / 65535.0f;
}

/* Send one extended frame: type in bits 28-24, 'mid' in bits 23-8, motor in 7-0 */
static HAL_StatusTypeDef rs_send(FDCAN_HandleTypeDef *hfdcan, uint32_t type,
                                 uint16_t mid, uint8_t motor_id, const uint8_t d[8])
{
  FDCAN_TxHeaderTypeDef h = {0};
  h.Identifier          = (type << 24) | ((uint32_t)mid << 8) | motor_id;
  h.IdType              = FDCAN_EXTENDED_ID;     /* 29-bit ID */
  h.TxFrameType         = FDCAN_DATA_FRAME;
  h.DataLength          = FDCAN_DLC_BYTES_8;
  h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  h.BitRateSwitch       = FDCAN_BRS_OFF;         /* classic CAN, no FD */
  h.FDFormat            = FDCAN_CLASSIC_CAN;
  h.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &h, (uint8_t *)d);
}

HAL_StatusTypeDef RS_RequestParam(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                                  uint16_t index)
{
  /* bytes 0-1 = parameter index, low byte first; rest 0 */
  uint8_t d[8] = { (uint8_t)(index & 0xFF), (uint8_t)(index >> 8) };
  return rs_send(hfdcan, RS_TYPE_PARAM_READ, RS_HOST_ID, motor_id, d);
}

bool RS_GetParamReply(FDCAN_HandleTypeDef *hfdcan, uint8_t *motor_id,
                      uint16_t *index, float *value)
{
  FDCAN_RxHeaderTypeDef h;
  uint8_t d[8];

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &h, d) != HAL_OK) {
      return false;
    }
    uint32_t id = h.Identifier;
    if (h.IdType != FDCAN_EXTENDED_ID)        continue;  /* not RobStride */
    if ((id >> 24) != RS_TYPE_PARAM_READ)     continue;  /* other frame type */
    if ((id & 0xFFu) != RS_HOST_ID)           continue;  /* not addressed to us */
    if (((id >> 16) & 0xFFu) != 0)            continue;  /* 1 = read failed */

    /* Reply ID: bits 15-8 = motor ID. Data: bytes 0-1 index (low first),
     * bytes 4-7 the value as an IEEE-754 float, low byte first. The
     * Cortex-M4 is little-endian too, so a plain byte copy gives the float. */
    *motor_id = (uint8_t)((id >> 8) & 0xFFu);
    *index    = (uint16_t)(d[0] | (d[1] << 8));
    memcpy(value, &d[4], sizeof *value);
    return true;
  }
  return false;
}

bool RS_GetFeedback(FDCAN_HandleTypeDef *hfdcan, RS_Feedback *fb)
{
  FDCAN_RxHeaderTypeDef h;
  uint8_t d[8];

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &h, d) != HAL_OK) {
      return false;
    }
    uint32_t id = h.Identifier;
    if (h.IdType != FDCAN_EXTENDED_ID)    continue;
    if ((id >> 24) != RS_TYPE_FEEDBACK)   continue;
    if ((id & 0xFFu) != RS_HOST_ID)       continue;

    /* ID bits 15-8 motor ID, 21-16 fault flags, 23-22 mode.
     * Data: big-endian 16-bit angle, speed, torque, then temperature*10. */
    fb->id        = (uint8_t)((id >> 8) & 0xFFu);
    fb->faults    = (uint8_t)((id >> 16) & 0x3Fu);
    fb->mode      = (uint8_t)((id >> 22) & 0x03u);
    fb->pos_rad   = u2f((uint16_t)(d[0] << 8 | d[1]), -RS_P_MAX, RS_P_MAX);
    fb->vel_rad_s = u2f((uint16_t)(d[2] << 8 | d[3]), -RS_V_MAX, RS_V_MAX);
    fb->torque_nm = u2f((uint16_t)(d[4] << 8 | d[5]), -RS_T_MAX, RS_T_MAX);
    fb->temp_c    = (float)(uint16_t)(d[6] << 8 | d[7]) / 10.0f;
    return true;
  }
  return false;
}

HAL_StatusTypeDef RS_Stop(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
  uint8_t d[8] = {0};            /* byte 0 = 1 would also clear faults */
  return rs_send(hfdcan, RS_TYPE_STOP, RS_HOST_ID, motor_id, d);
}

HAL_StatusTypeDef RS_Enable(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
  uint8_t d[8] = {0};
  return rs_send(hfdcan, RS_TYPE_ENABLE, RS_HOST_ID, motor_id, d);
}

HAL_StatusTypeDef RS_SetRunMode(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                                uint8_t mode)
{
  /* bytes 0-1 index (low first), bytes 4-7 value; run_mode is 1 byte in byte 4 */
  uint8_t d[8] = { (uint8_t)(RS_PARAM_RUN_MODE & 0xFF), (uint8_t)(RS_PARAM_RUN_MODE >> 8),
                   0, 0, mode, 0, 0, 0 };
  return rs_send(hfdcan, RS_TYPE_PARAM_WRITE, RS_HOST_ID, motor_id, d);
}

HAL_StatusTypeDef RS_Control(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                             float pos_rad, float vel_rad_s, float kp, float kd,
                             float torque_ff_nm)
{
  /* Type 1 puts the feed-forward torque in ID bits 23-8 (not our host ID).
   * Data fields are 16-bit, high byte first (manual 4.1.2). */
  uint16_t t = f2u(torque_ff_nm, -RS_T_MAX, RS_T_MAX);
  uint16_t p = f2u(pos_rad,      -RS_P_MAX, RS_P_MAX);
  uint16_t v = f2u(vel_rad_s,    -RS_V_MAX, RS_V_MAX);
  uint16_t a = f2u(kp,           0.0f,      RS_KP_MAX);
  uint16_t b = f2u(kd,           0.0f,      RS_KD_MAX);
  uint8_t d[8] = { (uint8_t)(p >> 8), (uint8_t)p, (uint8_t)(v >> 8), (uint8_t)v,
                   (uint8_t)(a >> 8), (uint8_t)a, (uint8_t)(b >> 8), (uint8_t)b };
  return rs_send(hfdcan, RS_TYPE_CONTROL, t, motor_id, d);
}
