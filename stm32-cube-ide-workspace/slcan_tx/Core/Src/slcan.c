#include "slcan.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

static CAN_HandleTypeDef *slcan_hcan;
static uint8_t can_open = 0;

// Error tracking (async - doesn't slow down TX)
static volatile uint32_t tx_success_count = 0;
static volatile uint32_t tx_error_count = 0;
static volatile uint32_t last_can_error = 0;

void SLCAN_Init(CAN_HandleTypeDef *hcan) {
  slcan_hcan = hcan;
  can_open = 0;
  tx_success_count = 0;
  tx_error_count = 0;
  last_can_error = 0;
}

// Call this periodically from main loop to check for TX errors
void SLCAN_CheckErrors(void) {
  if (slcan_hcan == NULL) return;

  uint32_t error = HAL_CAN_GetError(slcan_hcan);
  if (error != HAL_CAN_ERROR_NONE) {
    last_can_error = error;
    tx_error_count++;
    // Clear error flags
    __HAL_CAN_CLEAR_FLAG(slcan_hcan, CAN_FLAG_ERRI);
  }
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

  // Send frame (fire-and-forget, errors tracked async)
  if (HAL_CAN_AddTxMessage(slcan_hcan, &txHeader, txData, &txMailbox) == HAL_OK) {
	  tx_success_count++;  // Optimistic - actual errors tracked via SLCAN_CheckErrors()
	  SLCAN_SendResponse("\r"); // OK - queued
  } else {
	  tx_error_count++;
	  SLCAN_SendResponse("\x07"); // Error - couldn't queue
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

	  case 'F': // Read status flags (returns actual CAN error state)
		  {
			  // Check for current errors
			  SLCAN_CheckErrors();

			  // Build status: TEC in high nibble approximation, error flags in low
			  uint32_t esr = slcan_hcan->Instance->ESR;
			  uint8_t tec = (esr >> 16) & 0xFF;  // TEC is bits 23:16
			  uint8_t rec = (esr >> 24) & 0xFF;  // REC is bits 31:24
			  uint8_t status = 0;
			  if (tec > 96 || rec > 96) status |= 0x01;  // Warning
			  if (tec > 127 || rec > 127) status |= 0x02;  // Error passive
			  if (esr & CAN_ESR_BOFF) status |= 0x04;  // Bus-off

			  static char resp[16];
			  snprintf(resp, sizeof(resp), "F%02X\r", status);
			  SLCAN_SendResponse(resp);
		  }
		  break;

	  case 'E': // Extended status (custom command): returns TEC,REC,tx_ok,tx_err
		  {
			  SLCAN_CheckErrors();
			  uint32_t esr = slcan_hcan->Instance->ESR;
			  uint8_t tec = (esr >> 16) & 0xFF;
			  uint8_t rec = (esr >> 24) & 0xFF;
			  static char resp[32];
			  snprintf(resp, sizeof(resp), "E%02X%02X%04lX%04lX\r",
					   tec, rec,
					   tx_success_count & 0xFFFF,
					   tx_error_count & 0xFFFF);
			  SLCAN_SendResponse(resp);
		  }
		  break;

	  case 'R': // Reset CAN controller (recover from bus-off, clear TEC/REC)
		  {
			  HAL_CAN_Stop(slcan_hcan);

			  // Full peripheral reset via RCC to clear TEC/REC hardware counters
			  __HAL_RCC_CAN1_FORCE_RESET();
			  __HAL_RCC_CAN1_RELEASE_RESET();

			  // Re-initialize CAN with same settings
			  HAL_CAN_Init(slcan_hcan);

			  tx_success_count = 0;
			  tx_error_count = 0;
			  last_can_error = 0;

			  if (can_open) {
				  if (HAL_CAN_Start(slcan_hcan) == HAL_OK) {
					  SLCAN_SendResponse("\r");
				  } else {
					  can_open = 0;
					  SLCAN_SendResponse("\x07");
				  }
			  } else {
				  SLCAN_SendResponse("\r");
			  }
		  }
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
