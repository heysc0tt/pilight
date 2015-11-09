/*
	Copyright (C) 2015 CurlyMo & Rafael

	This file is part of pilight.

	pilight is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software
	Foundation, either version 3 of the License, or (at your option) any later
	version.

	pilight is distributed in the hope that it will be useful, but WITHOUT ANY
	WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
	A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with pilight. If not, see	<http://www.gnu.org/licenses/>
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../../core/pilight.h"
#include "../../core/common.h"
#include "../../core/dso.h"
#include "../../core/log.h"
#include "../../core/binary.h"
#include "../../core/gc.h"
#include "../protocol.h"
#include "maverick.h"

#define PULSE_MULTIPLIER        4
#define MIN_PULSE_LENGTH        100
#define MAX_PULSE_LENGTH        350
#define AVG_PULSE_LENGTH        250
#define MIN_LONG_PULSE			3000
#define MAX_LONG_PULSE			6000
#define RAW_LENGTH              50
#define MIN_RAW_LENGTH			90
#define MAX_RAW_LENGTH			175

#define NUM_BYTES       13
#define NUM_NIBBLES     NUM_BYTES * 2
#define NUM_BITS        NUM_BYTES * 8	

static int validate(void) {
	if(maverick->rawlen >= MIN_RAW_LENGTH && maverick->rawlen <= MAX_RAW_LENGTH) {
		if(maverick->raw[maverick->rawlen-1] >= (MIN_PULSE_LENGTH*PULSE_DIV) &&
			maverick->raw[maverick->rawlen-1] <= (MAX_PULSE_LENGTH*PULSE_DIV)) {
			return 0;
		}
	}

	return -1;
}
static void createMessage(int foodTemp, int bbqTemp) {
	maverick->message = json_mkobject();
	// json_append_member(maverick->message, "id", json_mknumber(1, 0));
	json_append_member(maverick->message, "temperature", json_mknumber(foodTemp, 0));
	json_append_member(maverick->message, "bbq", json_mknumber(bbqTemp, 0));
}

static void parse_binary_data(char *binary_in, char *hex_out)
{
    int i,j;
    unsigned char temp;
    // Parse binary data into hex nibbles (stored inefficiently in bytes)
    for(i=0;i<NUM_NIBBLES;i++){
        hex_out[i]=0; //initialize to 0
        for(j=0;j<4;j++){
            hex_out[i] <<= 1;
            temp = binary_in[(i*4)+j];
            hex_out[i] = hex_out[i] | temp;
        }
    }
}

//Calculate probe temperature in celsius
static signed int calc_probe_temp(char which_probe, char *rx_parsed)
{
    int i, offset, probe_temp;
    unsigned char   rx_quart[5];
    
    if(which_probe == 1)
        offset = 8;
    else
        offset = 13;

    //Parse data to quaternary
    for(i=0;i<5;i++){
        switch(rx_parsed[i+offset]){
            case 0x05:
                rx_quart[i] = 0;
            break;
            case 0x06:
                rx_quart[i] = 1;
            break;
            case 0x09:
                rx_quart[i] = 2;
            break;
            case 0x0A:
                rx_quart[i] = 3;
            break;
        }
    }
    probe_temp = 0;
    probe_temp += rx_quart[0] * 256;
    probe_temp += rx_quart[1] * 64;
    probe_temp += rx_quart[2] * 16;
    probe_temp += rx_quart[3] * 4;
    probe_temp += rx_quart[4] * 1;
    probe_temp -= 532;
    return probe_temp;
}

static void parseCode(void) {
//	int binary[RAW_LENGTH/2], x = 0, i = 0;
//	int id = -1, state = -1, unit = -1, systemcode = -1;
	int x=0;
	int values[maverick->rawlen];

	for(x=0;x<maverick->rawlen;x++) {
		if(maverick->raw[x] > MIN_LONG_PULSE) {
//			printf("Long: %d\n", maverick->raw[x]);
			values[x] = -1;
		} else if(maverick->raw[x] > MAX_PULSE_LENGTH) {
//			printf("Medium: %d\n", maverick->raw[x]);
			values[x] = 1;
		} else {
//			printf("Short: %d\n", maverick->raw[x]);
			values[x] = 0;
		}
	}

	int previous_period_was_short = 0;
	char bits[NUM_BITS]; // shouldnt need all these
	unsigned int bit_index=1;
	unsigned int current_bit = 1;
	bits[0] = current_bit;
	for(x=0;x<maverick->rawlen;x++) {
	    if (values[x] == 0) { // short pulse
	      if (previous_period_was_short == 1) {
	        // previous bit was short, add the current_bit value to the stream and continue to next incoming bit
			bits[bit_index++] = current_bit;
	        previous_period_was_short = 0;
	      }
	      else {
	        // previous bit was long, remember that and continue to next incoming bit
	        previous_period_was_short = 1;
	      }
	    }
	    else if (values[x] == 1) { // medium pulse
          if (previous_period_was_short == 1) { //cannot have a long pulse if previous_period was short
			logprintf(LOG_DEBUG, "Oh, shit! Recieved medium after a single short.");
			return;
          }

	      // swap the current_bit
	      current_bit = !current_bit;

	      // add current_bit value to the stream and continue to next incoming bit
		  bits[bit_index++] = current_bit;
	    }
	}		

	// for(x=0;x<bit_index;x++) {
	// 	printf("Bits[%d]=%d\n",x,bits[x]);
	// }
    logprintf(LOG_DEBUG, "Parsing bits into nibbles.");
    char nibbles[NUM_NIBBLES];
	parse_binary_data(bits, nibbles);

	//Validate? Checksum?

	// for(x=0;x<NUM_NIBBLES;x++) {
	// 	printf("Nibble[%d]=%#02x\n", x,nibbles[x]);
	// }

	signed int probe_1,probe_2;
	probe_1 = calc_probe_temp(1, nibbles);
	probe_2 = calc_probe_temp(2, nibbles);

	logprintf(LOG_DEBUG, "Probe 1: %d\n", probe_1);
	logprintf(LOG_DEBUG, "Probe 2: %d\n", probe_2);

	createMessage(probe_1, probe_2);
}

static void printHelp(void) {
	printf("Sorry, no help\n");
	// printf("\t -i  --id=id\t\t\tcontrol a device with this id\n");
	// printf("\t -s --systemcode=systemcode\t\t\tcontrol a device with this systemcode\n");
	// printf("\t -u --unitcode=unitcode\t\t\tcontrol a device with this unitcode\n");
	// printf("\t -t --on\t\t\tsend an on signal\n");
	// printf("\t -f --off\t\t\tsend an off signal\n");
}

#if !defined(MODULE) && !defined(_WIN32)
__attribute__((weak))
#endif

void maverickInit(void) {
	protocol_register(&maverick);
	protocol_set_id(maverick, "maverick");
	protocol_device_add(maverick, "maverick", "maverick thermometer");
	maverick->devtype = WEATHER;
	maverick->hwtype = RF433;
	maverick->minrawlen = MIN_RAW_LENGTH;
	maverick->maxrawlen = MAX_RAW_LENGTH;
	maverick->maxgaplen = MAX_PULSE_LENGTH*PULSE_DIV;
	maverick->mingaplen = MIN_PULSE_LENGTH*PULSE_DIV;
	maverick->rxrpt = 4;


	// options_add(&maverick->options, 't', "on", OPTION_NO_VALUE, DEVICES_STATE, JSON_STRING, NULL, NULL);
	// options_add(&maverick->options, 'f', "off", OPTION_NO_VALUE, DEVICES_STATE, JSON_STRING, NULL, NULL);
	// options_add(&maverick->options, 'u', "unit", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "^([0-7])$");
	// options_add(&maverick->options, 's', "systemcode", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "^([0-9]{1,4}|1[0-6][0-9]{3})$");
	// options_add(&maverick->options, 'i', "id", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "^([0-3])$");
	options_add(&maverick->options, 't', "temperature", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, NULL);
	options_add(&maverick->options, 'b', "bbq", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, NULL);

	options_add(&maverick->options, 0, "show-temperature", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *)1, "^[10]{1}$");

	// options_add(&maverick->options, 0, "readonly", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *)0, "^[10]{1}$");

	maverick->parseCode=&parseCode;
	maverick->printHelp=&printHelp;
	maverick->validate=&validate;
}
#if defined(MODULE) && !defined(_WIN32)
void compatibility(struct module_t *module) {
	module->name = "maverick";
	module->version = "1.0";
	module->reqversion = "6.0";
	module->reqcommit = "187";
}

void init(void) {
	maverickInit();
}
#endif
