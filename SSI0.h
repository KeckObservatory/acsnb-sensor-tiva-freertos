/*
 * SSI0.h
 */

#ifndef SSI0_H_
#define SSI0_H_

#include <stdint.h>

/* Function prototypes */
extern void SSI0Init(bool *received, bool *sent, uint8_t RxBuffer[], uint8_t TxBuffer[], uint16_t length);
extern void SSI0Interrupt(void);
extern void SSI0SendMessage(void);
extern void InitSPITransfer(void);
extern void uDMAErrorHandler(void);

#endif /* SSI0_H_ */
