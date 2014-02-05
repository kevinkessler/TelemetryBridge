/*
 * Spektrum.c
 *
 *  Created on: Jan 15, 2014
 *      Author: Kevin
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/twi.h>

#define MSG_LENGTH 16

void TWISlaveInit();


extern uint8_t alt_buf[16];
extern uint8_t cur_buf[16];
extern uint8_t pwr_buf[16];
extern uint8_t air_buf[16];
extern uint8_t gps_buf16[16];
extern uint8_t gps_buf17[16];

uint8_t addresses[]={0x03,0x0A,0x11,0x12,0x16,0x17,0x00};
uint8_t initializing=1;
uint8_t addrIdx=0;
uint8_t twi_buf[17];
volatile uint8_t twi_buf_ptr=0;
volatile uint8_t addrCount=0;

void TWISlaveInit()
{
	TWAR=addresses[0]<<1;

	TWSR=0;
	TWBR=0;

	TWCR=(1<<TWEN)|(1<<TWEA)|(1<<TWIE);


	for(int n=0;n<16;n++)
		twi_buf[n]=alt_buf[n];
	sei();


}

ISR(TWI_vect)
{

	uint8_t dummy;
	switch (TW_STATUS)
	{
		case TW_SR_SLA_ACK:
			dummy=TWDR;
			TWCR |= (1<<TWINT);
			break;
		case TW_ST_SLA_ACK:
			if(initializing)
			{
				addrCount++;
				if(addrCount==2)
				{
					addrCount=0;
					if(addresses[++addrIdx]==0)
					{
						TWAR=addresses[0]<<1;
						initializing=0;
					}
					else
						TWAR=addresses[addrIdx]<<1;
				}
			}
			else
				TWAMR=0xFF;

			uint8_t funct=TWDR>>1;
			switch(funct)
			{
			case 0x03:
				memcpy(twi_buf,cur_buf,16);
				break;
			case 0x0A:
				memcpy(twi_buf,pwr_buf,16);
				break;
			case 0x11:
				memcpy(twi_buf,air_buf,16);
				break;
			case 0x12:
				memcpy(twi_buf,alt_buf,16);
				break;
			case 0x16:
				memcpy(twi_buf,gps_buf16,16);
				break;
			case 0x17:
				memcpy(twi_buf,gps_buf17,16);
				break;
			default:
				for (int n=0;n<16;n++)
					twi_buf[n]=0;
				break;
			}
		case TW_ST_DATA_ACK:
			TWDR=twi_buf[twi_buf_ptr++];
			if(twi_buf_ptr==MSG_LENGTH)
			{
				twi_buf_ptr=0;
				TWCR = (1<<TWINT) | (1<<TWEN) | (0<<TWEA) | (1<<TWIE);
			}
			else
			{
				TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
			}
			break;
		case TW_ST_LAST_DATA:
		case TW_ST_DATA_NACK:
				TWCR=(1<<TWEN)|(1<<TWEA)|(1<<TWINT)| (1<<TWIE);
			break;
		default:
			TWCR|=(1<<TWINT);

			break;
	}

	sei();

}
