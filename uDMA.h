/*
 * uDMA.h
 */

#ifndef UDMA_H_
#define UDMA_H_

#include <stdint.h>

/* Function prototypes */
extern void uDMAInit(void);
extern void uDMAInitSSI0(void);
extern void uDMAReceiveSSI0(char RxMessageBuffer[], uint16_t dataLength);
extern void uDMASendSSI0(char TxMessageBuffer[], uint16_t dataLength);
extern void uDMAErrorHandler(void);

#endif /* UDMA_H_ */
