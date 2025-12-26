#include "slcan.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

static CAN_HandleTypeDef *slcan_hcan;
static uint8_t can_open = 0;

void SLCAN_Init(CAN_HandleTypeDef *hcan) {
  slcan_hcan = hcan;
  can_open = 0;
}

uint8_t SLCAN_IsOpen(void) {
  return can_open;
}

static void SLCAN_SendResponse(const char *resp) {
  CDC_Transmit_FS((uint8_t*)resp, strlen(resp));
}

static void SLCAN_TransmitFrame(uint8_t *buf, uint8_t extended) {
  CAN_TxHeaderTypeDef txHeader;
  uint8_t txData[8];
  uint32_t txMailbox;
  uint32_t id;
  uint8_t dlc;
  uint8_t idx;

  if (!can_open) {
	  SLCAN_SendResponse("\x07"); // BEL = error
	  return;
  }

  if (extended) {
	  // Extended frame: Tiiiiiiiildd...
	  // Parse 8 hex chars for ID
	  id = 0;
	  for (int i = 0; i < 8; i++) {
		  id <<= 4;
		  if (buf[1+i] >= '0' && buf[1+i] <= '9') id |= buf[1+i] - '0';
		  else if (buf[1+i] >= 'A' && buf[1+i] <= 'F') id |= buf[1+i] - 'A' + 10;
		  else if (buf[1+i] >= 'a' && buf[1+i] <= 'f') id |= buf[1+i] - 'a' + 10;
	  }
	  dlc = buf[9] - '0';
	  idx = 10;
	  txHeader.IDE = CAN_ID_EXT;
	  txHeader.ExtId = id;
  } else {
	  // Standard frame: tiiildd...
	  // Parse 3 hex chars for ID
	  id = 0;
	  for (int i = 0; i < 3; i++) {
		  id <<= 4;
		  if (buf[1+i] >= '0' && buf[1+i] <= '9') id |= buf[1+i] - '0';
		  else if (buf[1+i] >= 'A' && buf[1+i] <= 'F') id |= buf[1+i] - 'A' + 10;
		  else if (buf[1+i] >= 'a' && buf[1+i] <= 'f') id |= buf[1+i] - 'a' + 10;
	  }
	  dlc = buf[4] - '0';
	  idx = 5;
	  txHeader.IDE = CAN_ID_STD;
	  txHeader.StdId = id;
  }

  if (dlc > 8) dlc = 8;
  txHeader.DLC = dlc;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.TransmitGlobalTime = DISABLE;

  // Parse data bytes
  for (int i = 0; i < dlc; i++) {
	  uint8_t hi = buf[idx + i*2];
	  uint8_t lo = buf[idx + i*2 + 1];
	  uint8_t val = 0;

	  if (hi >= '0' && hi <= '9') val = (hi - '0') << 4;
	  else if (hi >= 'A' && hi <= 'F') val = (hi - 'A' + 10) << 4;
	  else if (hi >= 'a' && hi <= 'f') val = (hi - 'a' + 10) << 4;

	  if (lo >= '0' && lo <= '9') val |= lo - '0';
	  else if (lo >= 'A' && lo <= 'F') val |= lo - 'A' + 10;
	  else if (lo >= 'a' && lo <= 'f') val |= lo - 'a' + 10;

	  txData[i] = val;
  }

  // Send frame
  if (HAL_CAN_AddTxMessage(slcan_hcan, &txHeader, txData, &txMailbox) == HAL_OK) {
	  SLCAN_SendResponse("\r"); // OK
  } else {
	  SLCAN_SendResponse("\x07"); // Error
  }
}

void SLCAN_ProcessCommand(uint8_t *buf, uint16_t len) {
  if (len == 0) return;

  switch (buf[0]) {
	  case 'V': // Version
	  case 'v':
		  SLCAN_SendResponse("V1010\r");
		  break;

	  case 'N': // Serial number
		  SLCAN_SendResponse("NSTM32\r");
		  break;

	  case 'O': // Open CAN
		  if (HAL_CAN_Start(slcan_hcan) == HAL_OK) {
			  can_open = 1;
			  SLCAN_SendResponse("\r");
		  } else {
			  SLCAN_SendResponse("\x07");
		  }
		  break;

	  case 'C': // Close CAN
		  HAL_CAN_Stop(slcan_hcan);
		  can_open = 0;
		  SLCAN_SendResponse("\r");
		  break;

	  case 'S': // Set baud (ignored, we're fixed at 500k)
		  SLCAN_SendResponse("\r");
		  break;

	  case 'F': // Read status flags
		  SLCAN_SendResponse("F00\r");
		  break;

	  case 't': // Transmit standard frame
		  SLCAN_TransmitFrame(buf, 0);
		  break;

	  case 'T': // Transmit extended frame
		  SLCAN_TransmitFrame(buf, 1);
		  break;

	  default:
		  SLCAN_SendResponse("\x07"); // Unknown command
		  break;
  }
}
