/*
 * mavlink_bridge_adapter.h
 *
 *  Created on: Dec 11, 2013
 *      Author: Kevin
 */

#ifndef MAVLINK_BRIDGE_ADAPTER_H_
#define MAVLINK_BRIDGE_ADAPTER_H_


#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include <avr/io.h>
#include "mavlink_types.h"

mavlink_system_t mavlink_system;


static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    if (chan == MAVLINK_COMM_0)
    {
    	while(!(UCSR0A & (1<<UDRE0)));
    	UDR0 = ch;
    }
    if (chan == MAVLINK_COMM_1)
    {
    	while(!(UCSR0A & (1<<UDRE0)));
    	UDR0 = ch;
    }
}


#endif /* MAVLINK_BRIDGE_ADAPTER_H_ */
