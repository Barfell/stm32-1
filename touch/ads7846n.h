/*--------------------------------------------------------------------------
 * ads7846n.c -
 *
 * Author: 185275258 (QQ Group)
 *
 *--------------------------------------------------------------------------*/

#ifndef TOUCH_ADS7846N_H_
#define TOUCH_ADS7846N_H_

uint8_t ADS7846N_Init(void);
uint8_t ADS7846N_IsDetected(void);
uint8_t ADS7846N_ReadXY(uint16_t *px, uint16_t *py, uint16_t *pressure);
uint8_t ADS7846N_ReadX(uint16_t *px);
uint8_t ADS7846N_ReadY(uint16_t *py);

#endif /* TOUCH_ADS7846N_H_ */
