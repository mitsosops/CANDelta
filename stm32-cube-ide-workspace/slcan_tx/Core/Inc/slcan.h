#ifndef INC_SLCAN_H_
#define INC_SLCAN_H_

#include "main.h"

void SLCAN_Init(CAN_HandleTypeDef *hcan);
void SLCAN_ProcessCommand(uint8_t *buf, uint16_t len);
uint8_t SLCAN_IsOpen(void);
void SLCAN_CheckErrors(void);  // Call periodically to track CAN errors

#endif /* INC_SLCAN_H_ */
