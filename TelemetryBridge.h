/*
 * TelemetryBridge.h
 *
 *  Created on: Jan 22, 2014
 *      Author: Kevin
 */

#ifndef TELEMETRYBRIDGE_H_

#define BAUD 57600
#define BAUD_PRESCALLER (((F_CPU / (BAUD * 16UL))) - 1)

#define LOW_BATTERY 0
#define RX_BUFFER_SIZE 16

void TWISlaveInit(void);
void USART_Init(void);
uint8_t usartGetChar(void);

uint16_t divideBy10(uint16_t dividend);
uint32_t divideBy100(int32_t dividend);
uint16_t divideBy1E7(uint32_t dividend);
uint8_t bcd(uint8_t num);

void setCurBuf(int16_t cur);
void setAltBuf(int32_t alt);
void setPwrBuf(uint16_t voltage,int8_t battery_remaining);
void setAirBuf(uint16_t air_cm_s);
void setGPSBuf(int32_t lat, int32_t lon, int32_t alt, uint16_t vel, uint16_t cog, uint8_t sats);
uint8_t parseCoord(uint8_t *deg, uint8_t *min, uint8_t *sec, uint8_t *subSec, int32_t coord);

#define TELEMETRYBRIDGE_H_



#endif /* TELEMETRYBRIDGE_H_ */
