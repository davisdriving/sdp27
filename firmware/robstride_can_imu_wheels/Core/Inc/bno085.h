/* bno085.h - minimal BNO085 driver over SPI (SHTP / SH-2).
 * Gives the game rotation vector (quaternion) and calibrated gyro.
 * Needs CubeMX: SPI2 (PB13/PB14/PB15), GPIO BNO_CS (PB2, output, high),
 * BNO_INT (PA10, input), BNO_RST (PB1, output, high). See the setup guide. */
#ifndef BNO085_H
#define BNO085_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float qi, qj, qk, qr;  /* game rotation vector, unit quaternion       */
  float gx, gy, gz;      /* calibrated gyroscope, rad/s                  */
  bool new_quat;         /* set when qi..qr update; your code clears it  */
  bool new_gyro;         /* set when gx..gz update; your code clears it  */
} BNO085_Data;

/* Resets the BNO085 and waits for it to answer.
 * interval_us: report period for both reports, e.g. 10000 = 100 Hz.
 * Returns false if the sensor never signals ready (wiring, P0/P1, power). */
bool BNO085_Init(uint32_t interval_us);

/* Call as often as possible from the main loop. Does nothing unless the
 * BNO085 has data waiting (INT low), so it is cheap to call. */
void BNO085_Service(BNO085_Data *d);

#endif /* BNO085_H */
