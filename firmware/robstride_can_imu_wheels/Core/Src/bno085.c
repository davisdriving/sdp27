/* bno085.c - minimal BNO085 driver over SPI (SHTP / SH-2).
 *
 * References (CEVA/Hillcrest docs; BNO085 uses the BNO080 interface):
 *   BNO080 datasheet v1.3   1.3.4 SPI, 1.4.1 SHTP header, 5.2 startup, 6.5.2/6.5.3 timing
 *   SH-2 Reference Manual v1.2   6.5.4 Set Feature, 6.5.13 gyro, 6.5.19 game RV, 7.2 timestamps
 */
#include "bno085.h"
#include "main.h"   /* HAL + the BNO_CS / BNO_INT / BNO_RST pin labels from CubeMX */
#include <string.h>

extern SPI_HandleTypeDef hspi2;

#define CS_LOW()   HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET)
#define INT_LOW()  (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port, BNO_INT_Pin) == GPIO_PIN_RESET)

#define CH_CONTROL  2     /* SHTP channel: sensor hub control (commands)  */
#define CH_REPORTS  3     /* SHTP channel: input sensor reports           */
#define ID_GYRO     0x02  /* Gyroscope Calibrated, Q9, rad/s              */
#define ID_GRV      0x08  /* Game Rotation Vector, Q14 quaternion         */
#define CMD_LEN     21    /* 4-byte SHTP header + 17-byte Set Feature     */

static uint8_t rx[300];             /* holds every report packet we care about */
static uint8_t tx[sizeof rx];
static uint8_t cmd[2][CMD_LEN];     /* Set Feature for game RV and gyro        */
static uint8_t next_cmd;
static uint8_t seq_control;         /* SHTP sequence number, our channel 2     */
static bool seen_grv, seen_gyro;    /* true once that report has arrived       */

/* One SPI transaction. Sends 'out' (0 or CMD_LEN bytes) and, at the same time,
 * reads the packet the BNO085 has ready. Both sides clock the longer of the two
 * lengths. Returns the number of valid bytes in rx (0 if none). */
static uint16_t transfer(const uint8_t *out, uint16_t out_len)
{
  memset(tx, 0, sizeof tx);
  if (out_len) memcpy(tx, out, out_len);

  CS_LOW();
  if (HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4, 10) != HAL_OK) {
    CS_HIGH();
    return 0;
  }

  uint16_t in_len = 0;
  if (!(rx[0] == 0xFF && rx[1] == 0xFF))                   /* 0xFFFF = no device */
    in_len = (uint16_t)(rx[0] | (rx[1] << 8)) & 0x7FFF;    /* bit 15 = continuation */

  uint16_t total = in_len > out_len ? in_len : out_len;
  if (total > sizeof rx) {
    CS_HIGH();
    return 0;
  }
  if (total > 4 && HAL_SPI_TransmitReceive(&hspi2, tx + 4, rx + 4, total - 4, 10) != HAL_OK) {
    CS_HIGH();
    return 0;
  }
  CS_HIGH();
  return in_len;
}

static float q_to_float(const uint8_t *p, int q)
{
  return (float)(int16_t)(p[0] | (p[1] << 8)) / (float)(1 << q);
}

/* Pull our two reports out of a channel-3 packet. One packet can hold several
 * reports back to back, starting with a 5-byte timestamp record. */
static void parse(uint16_t len, BNO085_Data *d)
{
  if (len < 5 || rx[2] != CH_REPORTS) return;

  uint16_t i = 4;
  while (i < len) {
    uint8_t id = rx[i];
    if (id == 0xFB || id == 0xFA) {                  /* timestamp records */
      i += 5;
    } else if (id == ID_GRV && i + 12 <= len) {
      d->qi = q_to_float(&rx[i + 4], 14);
      d->qj = q_to_float(&rx[i + 6], 14);
      d->qk = q_to_float(&rx[i + 8], 14);
      d->qr = q_to_float(&rx[i + 10], 14);
      d->new_quat = true;
      seen_grv = true;
      i += 12;
    } else if (id == ID_GYRO && i + 10 <= len) {
      d->gx = q_to_float(&rx[i + 4], 9);
      d->gy = q_to_float(&rx[i + 6], 9);
      d->gz = q_to_float(&rx[i + 8], 9);
      d->new_gyro = true;
      seen_gyro = true;
      i += 10;
    } else {
      break;   /* a report we did not enable: skip the rest of this packet */
    }
  }
}

static void make_set_feature(uint8_t *p, uint8_t report_id, uint32_t interval_us)
{
  memset(p, 0, CMD_LEN);
  p[0] = CMD_LEN;            /* SHTP length LSB (header included) */
  p[2] = CH_CONTROL;         /* p[3] = sequence number, filled when sent */
  p[4] = 0xFD;               /* Set Feature command */
  p[5] = report_id;
  p[9]  = (uint8_t)(interval_us);
  p[10] = (uint8_t)(interval_us >> 8);
  p[11] = (uint8_t)(interval_us >> 16);
  p[12] = (uint8_t)(interval_us >> 24);
}

bool BNO085_Init(uint32_t interval_us)
{
  make_set_feature(cmd[0], ID_GRV,  interval_us);
  make_set_feature(cmd[1], ID_GYRO, interval_us);
  seen_grv = seen_gyro = false;

  /* Enable SPI now so SCK settles at its idle-high level (mode 3) before CS
   * ever goes low. Otherwise the first transfer can start with a false edge. */
  __HAL_SPI_ENABLE(&hspi2);
  CS_HIGH();

  HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_SET);

  /* After reset the BNO085 pulls INT low when it is ready (datasheet 5.2.1). */
  uint32_t start = HAL_GetTick();
  while (!INT_LOW()) {
    if (HAL_GetTick() - start > 1000) return false;
  }
  return true;
}

void BNO085_Service(BNO085_Data *d)
{
  if (!INT_LOW()) return;    /* nothing waiting */

  /* PS0/WAKE is tied high on this board, so we can only talk while INT is low.
   * Until both reports are flowing, piggyback a Set Feature command on each
   * read. Every command makes the BNO085 reply, which pulls INT low again,
   * so this keeps going until both reports have arrived. */
  if (!(seen_grv && seen_gyro)) {
    uint8_t *c = cmd[next_cmd];
    next_cmd ^= 1;
    c[3] = seq_control++;
    parse(transfer(c, CMD_LEN), d);
  } else {
    parse(transfer(NULL, 0), d);
  }
}
