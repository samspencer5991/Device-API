#ifndef DEVICE_API_HANDLER_H_
#define DEVICE_API_HANDLER_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

// Transmit functions
extern void sendCheckResponse(uint8_t transport);
extern void sendGlobalSettings(uint8_t transport);
extern void sendBankSettings(int bankNum, uint8_t transport);
extern void sendBankId(int bankNum, uint8_t transport);
extern void sendCurrentBank(uint8_t transport);
extern void sendOkPacket(uint8_t transport);

// Parsing functions
extern void parseGlobalSettings(char* appData, uint8_t transport);
extern void parseBankSettings(char* appData, uint16_t bankNum, uint8_t transport);
extern void ctrlCommandHandler(char* appData, uint8_t transport);

#ifdef __cplusplus
}
#endif

#endif /*DEVICE_API_HANDLER_H_*/