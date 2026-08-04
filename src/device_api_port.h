#ifndef DEVICE_API_PORT_H_
#define DEVICE_API_PORT_H_

// Platform seams for the ESP-IDF build of Device-API.
//
// On ESP-IDF the library owns no USB I/O: the application feeds received,
// '~'-terminated packets into deviceApi_Handler(), and provides the transmit
// seams below (implemented in the app, e.g. CDC -> midiCore_CdcSend, MIDI ->
// a SysEx send over the MIDI core). This keeps Device-API free of any
// dependency on the specific connectivity component.

#include "stdint.h"
#include "stddef.h"

#if defined(ESP_PLATFORM) && !defined(ARDUINO)

#ifdef __cplusplus
extern "C" {
#endif

// Transmit raw bytes over the USB-CDC transport (application-provided).
void deviceApi_TransmitCdc(const uint8_t *buffer, size_t length);

// Transmit a Device-API SysEx string over the MIDI transport
// (application-provided). containsFraming != 0 means the buffer is a framed
// opening/termination packet (F0..F7); 0 means a raw payload data chunk.
void midi_SendDeviceApiSysExString(const char *buffer, uint16_t length, uint8_t containsFraming);

#ifdef __cplusplus
}
#endif

#endif // ESP_PLATFORM && !ARDUINO

#endif // DEVICE_API_PORT_H_
