/*
 * main.c

 *
 *  Created on: Dec 10, 2013
 *      Author: Kevin
 */

// Save memory, MAVLINK by default allocates 4 256 byte buffers
#define MAVLINK_COMM_NUM_BUFFERS 1

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "mavlink_bridge_adapter.h"
#include <common/mavlink.h>
#include "TelemetryBridge.h"

//Enable timing of the GPS calculations and display them in Capacity 2 of the Power Box
//#define TIMEIT

#ifdef TIMEIT
void TimerInit(void);
#endif

const char const *capName="BATT_CAPACITY";
const char const *lowBattery="Low Battery!";

uint8_t cur_buf[16];	// Current Buffer
uint8_t alt_buf[16];	// Altitude Buffer
uint8_t pwr_buf[16];	// PowerBox Buffer
uint8_t air_buf[16];	// Airspeed Buffer
uint8_t gps_buf16[16];  // GPS Buffer 0x16
uint8_t gps_buf17[16];  // GPS Buffer 0x17

uint16_t batteryCapacity=0;
uint8_t alarm=0;

uint16_t timeVal=0;

volatile uint8_t rTop=0;
volatile uint8_t rBot=0;
uint8_t rxBuf[RX_BUFFER_SIZE];

int main (void)
{
	char textBuffer[50];
	uint8_t batReqs=0;
	uint8_t heartbeats=0;
	uint8_t statusFlag=0;
	uint8_t gpsFlag=0;
	uint8_t batFlag=0;

	setCurBuf(0);
	setPwrBuf(0,0);
	setAltBuf(0);
	setAirBuf(0);
	setGPSBuf(0,0,0,0,0,0);

	TWISlaveInit();

#ifdef TIMEIT
	TimerInit();
#endif

	mavlink_system.sysid = 1;
	mavlink_system.compid = 50;

	USART_Init();

	mavlink_message_t msg;
	mavlink_status_t status;


	while(1)
	{
		uint8_t byte=usartGetChar();


		if(mavlink_parse_char(MAVLINK_COMM_0, byte,&msg,&status))
		{

			switch(msg.msgid)
			{
			case MAVLINK_MSG_ID_HEARTBEAT:
				heartbeats++;

				for (int n=1;n<10;n++)
					PORTB ^= (1<<PB5);

				// After a couple of heartbeats, try up to 3 times to get the battery capacity
				if((heartbeats>2)&&(!batFlag)&&(batReqs<3))
				{
					mavlink_msg_param_request_read_send(MAVLINK_COMM_0,0x01,0x00,capName,-1);
					batReqs++;
				}

				// if you go 2 heartbeats without getting the status or GPS message, re-request the telemetry stream
				if(((!statusFlag)||(!gpsFlag))&&(heartbeats>2))
				{

					mavlink_msg_request_data_stream_send(MAVLINK_COMM_0, 0x01, 0x00, 0x01, 0x200, 0x01);
					heartbeats=0;
				}
				else
				{
					statusFlag=0;
					gpsFlag=0;

				}

				break;
			case MAVLINK_MSG_ID_PARAM_VALUE:
				mavlink_msg_param_value_get_param_id(&msg,textBuffer);
				float value=mavlink_msg_param_value_get_param_value(&msg);
				if(strncmp(textBuffer,capName,16)==0)
				{
					batteryCapacity=(uint16_t)value;
					batFlag=1;
				}
				break;
			case MAVLINK_MSG_ID_SYS_STATUS:
				setPwrBuf(mavlink_msg_sys_status_get_voltage_battery(&msg),mavlink_msg_sys_status_get_battery_remaining(&msg));
				setCurBuf(mavlink_msg_sys_status_get_current_battery(&msg));
				statusFlag=1;
				break;
			case MAVLINK_MSG_ID_GPS_RAW_INT:
				setAltBuf(mavlink_msg_gps_raw_int_get_alt(&msg));

#ifdef TIMEIT
				TCNT1=0;
#endif
				setGPSBuf(mavlink_msg_gps_raw_int_get_lat(&msg),mavlink_msg_gps_raw_int_get_lon(&msg),mavlink_msg_gps_raw_int_get_alt(&msg),
						mavlink_msg_gps_raw_int_get_vel(&msg),mavlink_msg_gps_raw_int_get_cog(&msg),
						mavlink_msg_gps_raw_int_get_satellites_visible(&msg));
#ifdef TIMEIT
				timeVal=TCNT1;
#endif

				setAirBuf(mavlink_msg_gps_raw_int_get_vel(&msg));
				gpsFlag=1;
				break;
			case MAVLINK_MSG_ID_STATUSTEXT:
				mavlink_msg_statustext_get_text(&msg,textBuffer);

				// Set status for Powerbox when the low battery message is received
				if(strncmp(textBuffer,lowBattery,strlen(lowBattery))==0)
					alarm |= (1<<LOW_BATTERY);

				break;
			default:
				break;
			}
		}
	}

}

void setCurBuf(int16_t cur)
{
	for(int n=4;n<16;n++)
		cur_buf[n]=0;

	cur_buf[0]=0x03;
	cur_buf[1]=0x00;

	// Each unit is .1967 A or 2603 / 512 within 0.002 %.  Add 512/2 to round up.  Current input unit is 10mA
	uint32_t units32=(uint32_t)(cur)*2603+256;
	// Divide by 512
	uint16_t units=(units32 >> 9);
	//Divide by 100 for Amps
	units=divideBy100(units);

	cur_buf[2]=((units&0xff00)>>8);
	cur_buf[3]=(units&0x00ff);
}

void setAltBuf(int32_t alt_mm)
{
    int16_t alt=(int16_t)divideBy100(alt_mm);

    for(int n=5;n<16;n++)
		alt_buf[n]=0;

	alt_buf[0]=0x12;
	alt_buf[1]=0x00;
	alt_buf[4]=0xff;

	alt_buf[2]=(alt&0xff00)>>8;
	alt_buf[3]=(alt&0x00ff);

}

void setAirBuf(uint16_t air_cm_s)
{

	// Convert from cm/s to m/h, then divide by 1000
	uint16_t spd=(uint16_t)((((((uint32_t)(air_cm_s*36) * (uint32_t)(0x0625)) >> 16) + (air_cm_s*36)) >> 1) >> (9));

    for(int n=6;n<16;n++)
		air_buf[n]=0;

	air_buf[0]=0x11;
	air_buf[1]=0x00;

	air_buf[2]=(spd&0xff00)>>8;
	air_buf[3]=(spd&0x00ff);

	air_buf[4]=0x01;
	air_buf[5]=0xF9;

}

void setPwrBuf(uint16_t voltage,int8_t battery_remaining)
{
	pwr_buf[0]=0x0a;
	pwr_buf[1]=0x00;

	// Fixed point /10
	uint16_t v=divideBy10(voltage);

	//Voltage 1
	pwr_buf[2]=(v&0xff00)>>8;
	pwr_buf[3]=(v&0x00ff);

	//Voltage 1
	pwr_buf[4]=pwr_buf[5]=0;


	uint16_t capDisp=battery_remaining;
	if(batteryCapacity!=0)
	{
		// Fixed point divide by 100
		uint32_t totCap=(uint32_t)battery_remaining*(uint32_t)batteryCapacity;
		capDisp=divideBy100(totCap);
	}
	//capacity 1
	pwr_buf[6]=(capDisp&0xff00)>>8;
	pwr_buf[7]=(capDisp&0xff);

#ifdef TIMEIT
	pwr_buf[8]=(timeVal&0xff00)>>8;
	pwr_buf[9]=(timeVal&0xff);
#else
	pwr_buf[8]=pwr_buf[9]=0;
#endif

	for (uint8_t n=10;n<15;n++)
		pwr_buf[n]=0;

	pwr_buf[15]=alarm;

}



void setGPSBuf(int32_t lat, int32_t lon, int32_t alt_mm, uint16_t vel, uint16_t cog, uint8_t sats)
{
	uint16_t alt=divideBy100(alt_mm);
	uint8_t superAlt=0;
	uint8_t cordSign=0;


	if(alt>10000)
	{
		superAlt=divideBy100(divideBy100(alt));
		alt=alt-superAlt*10000;
	}

	gps_buf16[0]=0x16;
	gps_buf16[1]=0;
	gps_buf16[2]=bcd((alt & 0x00ff));
	gps_buf16[3]=bcd((alt & 0xff00) >> 8);

	uint8_t deg;
	uint8_t min;
	uint8_t sec;
	uint8_t subSec;

	if(parseCoord(&deg,&min,&sec,&subSec,lat))
		cordSign|=0x01;

	gps_buf16[4]=bcd(subSec);
	gps_buf16[5]=bcd(sec);
	gps_buf16[6]=bcd(min);
	gps_buf16[7]=bcd(deg);

	if(parseCoord(&deg,&min,&sec,&subSec,lon))
		cordSign|=0x01;
	gps_buf16[8]=bcd(subSec);
	gps_buf16[9]=bcd(sec);
	gps_buf16[10]=bcd(min);

	if(deg > 99)
	{
		cordSign |=0x04;
		deg-=100;
	}
	gps_buf16[11]=bcd(deg);

	uint16_t heading=divideBy10(cog);
	uint8_t headingMSB=divideBy100(heading);
	gps_buf16[12]=bcd(heading-headingMSB*100);
	gps_buf16[13]=bcd(headingMSB);

	gps_buf16[14]=0x0;
	gps_buf16[15]=cordSign;


	gps_buf17[0]=0x17;
	gps_buf17[1]=0;

	uint16_t speed=divideBy1E7((uint32_t)vel * 1943844LU);
	uint8_t speedMSB=divideBy100(speed);
	gps_buf17[2]=bcd(speed - speedMSB * 100);
	gps_buf17[3]=bcd(speedMSB);

	for (int n=4;n<16;n++)
		gps_buf17[n]=0;

	gps_buf17[8]=sats;
	gps_buf17[9]=superAlt;
}

uint8_t bcd(uint8_t num)
{
	uint8_t ten=divideBy10(num);
	return (ten<<4)+(num-ten*10);
}

uint16_t divideBy10(uint16_t dividend)
{
	return (uint16_t)(((uint32_t)dividend * (uint32_t)0xCCCD) >> 16) >> 3;
	//return (uint16_t)dividend/10;
}

uint32_t divideBy100(int32_t dividend)
{
	// Divide by 100 the fixed-point way

	return (uint32_t)((((((int64_t)dividend*(int64_t)0x47AF) >> 16)+dividend) >> 1)>>6);
	//return (uint32_t)dividend/100;
}

uint16_t divideBy1E7(uint32_t dividend)
{
	return (uint16_t)((((int64_t)dividend * 0xD6E0LU) >> 16) >> 23);
	//return (uint16_t)(dividend / 1E7);
}

uint8_t parseCoord(uint8_t *deg, uint8_t *min, uint8_t *sec, uint8_t *subSec, int32_t coord)
{
	uint8_t retval=0;
	if(coord < 0)
	{
		coord=-coord;
		retval=1;
	}

	*deg=divideBy1E7(coord);
	uint32_t minPart=coord - *deg * 1E7;
	*min=divideBy1E7(minPart * 60);
	uint32_t secPart=minPart*60 - *min * 1E7;
	*sec=divideBy1E7(secPart * 100);
	uint32_t subSecPart=secPart*100 - *sec * 1E7;
	*subSec=divideBy1E7(subSecPart * 100);

	return retval;

}

uint8_t usartGetChar()
{
	while(rTop==rBot);

	uint8_t retval=rxBuf[rBot];
	rBot=(rBot+1) & (RX_BUFFER_SIZE -1);

	return retval;

}

void USART_Init(void)
{
	// Set 57600 Baud
	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);

	// Enabled RX Interrupt, RX and TX. 8 Bit
	UCSR0B = (1 << RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	// No parity, 1 stop bit
	UCSR0C = ((1<<UCSZ00)|(1<<UCSZ01));

	rBot=rTop=0;
}

ISR(USART_RX_vect)
{
	rxBuf[rTop]=UDR0;
	rTop=(rTop+1) & (RX_BUFFER_SIZE -1);
}

#ifdef TIMEIT
void TimerInit()
{
	TCCR1A=0;
	TCCR1B|=(1 << CS10);
}
#endif



