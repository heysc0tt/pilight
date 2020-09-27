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
#include <bson.h>
#include <bcon.h>
#include <mongoc.h>

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

static void createMessage(int foodTemp, int bbqTemp, int id) {
	maverick->message = json_mkobject();
	json_append_member(maverick->message, "id", json_mknumber(id, 0));
	json_append_member(maverick->message, "temperature", json_mknumber(foodTemp, 0));
	json_append_member(maverick->message, "bbq", json_mknumber(bbqTemp, 0));
}

static void storeMessage() {
	mongoc_client_t     *client;
	mongoc_database_t   *database;
	mongoc_collection_t *collection;
	bson_t              *bson;
	bson_error_t        error;


	/*
    * Required to initialize libmongoc's internals
    */
    mongoc_init ();

	/*
	* Create a new client instance
	*/
	client = mongoc_client_new ("mongodb://192.168.1.142:27017");

	/*
	* Get a handle on the database "maverick" and collection "temperatures"
	*/
	database = mongoc_client_get_database (client, "maverick");
	collection = mongoc_client_get_collection (client, "maverick", "temperatures");

	// create bson from json
	uint8_t *jsonStr = json_stringify(maverick->message, NULL);
	bson = bson_new_from_json (jsonStr, -1, &error);
	json_free(jsonStr);

	if (!bson) {
		logprintf(LOG_ERR, "%s\n", error.message);
		return;
	}

	// insert
	if (!mongoc_collection_insert (collection, MONGOC_INSERT_NONE, bson, NULL, &error)) {
		logprintf(LOG_ERR, "%s\n", error.message);
	}

	bson_destroy(bson);

	/*
	* Release our handles and clean up libmongoc
	*/
	mongoc_collection_destroy (collection);
	mongoc_database_destroy (database);
	mongoc_client_destroy (client);
	mongoc_cleanup ();
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

//validate header looks like 0xAA9995
static int validate_header(char *rx_parsed)
{    
    if(rx_parsed[0] != 0x0A) {
    	return -1;
    }
    if(rx_parsed[1] != 0x0A) {
    	return -1;
    }
    if(rx_parsed[2] != 0x09) {
    	return -1;
    }
    if(rx_parsed[3] != 0x09) {
    	return -1;
    }
    if(rx_parsed[4] != 0x09) {
    	return -1;
    }
    if(rx_parsed[5] != 0x05) {
    	return -1;
    }
    if(rx_parsed[6] != 0x05 &&
       rx_parsed[6] != 0x06) {
       	logprintf(LOG_INFO, "index 6 wasnt 05 or 06, was: 0x%01x", rx_parsed[6]);
    	return -1;
    }
    if(rx_parsed[6] == 0x05 &&
       rx_parsed[7] != 0x09) {
       	logprintf(LOG_INFO, "index 6 was 05 but 7 was not 09, was: 0x%01x", rx_parsed[7]);
    	return -1;
    }
    if(rx_parsed[6] == 0x06 &&
       rx_parsed[7] != 0x0A) {
       	logprintf(LOG_INFO, "index 6 was 06 but 7 was not 0A, was: 0x%01x", rx_parsed[7]);
    	return -1;
    }

    return 0;
}

uint16_t shiftreg(uint16_t currentValue) {
    uint8_t msb = (currentValue >> 15) & 1;
    currentValue <<= 1;
    if (msb == 1) {
        // Toggle pattern for feedback bits
        // Toggle, if MSB is 1
        currentValue ^= 0x1021;
    }
    return currentValue;
}

static unsigned char quart(char rx_val) {
	switch(rx_val){
        case 0x05:
            return 0;
        case 0x06:
            return 1;
        case 0x09:
            return 2;
        case 0x0A:
            return 3;
        default:
        	return 4; // hopefully this will not happen
    }
}

uint16_t calculate_checksum(char *nibbles, uint32_t probe1, uint32_t probe2) {
	uint32_t check_data;
    uint16_t mask = 0x3331; //initial value of linear feedback shift register
    uint16_t csum = 0x0;

    check_data = quart(nibbles[6]) << 22;
    check_data |= quart(nibbles[7]) << 20;
    check_data |= (uint32_t) probe1 << 10;
    check_data |= (uint32_t) probe2;

    int i = 0;
    for(i = 0; i < 24; ++i) {
        if((check_data >> i) & 0x01) {
           //data bit at current position is "1"
           //do XOR with mask
          csum ^= mask; 
        }
        mask = shiftreg(mask);
    }
    return csum;
}

static void pulsesToValues(int *values) {
	int i;
	for(i=0;i<maverick->rawlen;i++) {
		if(maverick->raw[i] > MIN_LONG_PULSE) {
			values[i] = -1;
		} else if(maverick->raw[i] > MAX_PULSE_LENGTH) {
			values[i] = 1;
		} else {
			values[i] = 0;
		}
	}
}

static void manchesterDecode(char *bitsArr) {
	int values[maverick->rawlen];
    pulsesToValues(values);

	int previous_period_was_short = 0;
	unsigned int bit_index=1; // start at index 1 because the first bit is always 1 
	unsigned int current_bit = bitsArr[0] = 1;
	int x;

	for(x=0;x<maverick->rawlen;x++) {
	    if (values[x] == 0) { // short pulse
	      if (previous_period_was_short == 1) {
	        // previous bit was short, add the current_bit value to the stream and continue to next incoming bit
			bitsArr[bit_index++] = current_bit;
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
		  bitsArr[bit_index++] = current_bit;
	    }
	}		
}

static uint16_t getCheksum(char *nibbles) {
	uint16_t chksum_sent;
    chksum_sent =  (uint16_t) quart(nibbles[18]) << 14;
    chksum_sent |= (uint16_t) quart(nibbles[19]) << 12;
    chksum_sent |= (uint16_t) quart(nibbles[20]) << 10;
    chksum_sent |= (uint16_t) quart(nibbles[21]) << 8;
    chksum_sent |= (uint16_t) quart(nibbles[22]) << 6;
    chksum_sent |= (uint16_t) quart(nibbles[23]) << 4;
    chksum_sent |= (uint16_t) quart(nibbles[24]) << 2;
    chksum_sent |= (uint16_t) quart(nibbles[25]);
    return chksum_sent;
}

static void parseCode(void) {
	int x=0;
    uint32_t probe1=0, probe2=0;
    uint16_t calculatedChecksum, messageChecksum, checksumXOR;
	char bits[NUM_BITS]; // shouldnt need all these

	manchesterDecode(bits);

    logprintf(LOG_DEBUG, "Parsing bits into nibbles.");
    char nibbles[NUM_NIBBLES];
	parse_binary_data(bits, nibbles);
	
	//Validate
	if(validate_header(nibbles) != 0) {
		logprintf(LOG_DEBUG, "The header doesn't match, skipping this message.");
		return;
	}

    for(x=0; x<=4; x++)
    {
        probe1 += quart(nibbles[12-x]) * (1<<(2*x));
        probe2 += quart(nibbles[17-x]) * (1<<(2*x));
    }

    calculatedChecksum = calculate_checksum(nibbles, probe1, probe2);
    messageChecksum = getCheksum(nibbles);

    checksumXOR = calculatedChecksum ^ messageChecksum;

    if(nibbles[6] == 0x06 &&
       nibbles[7] != 0x0A) {
       	logprintf(LOG_INFO, "Updating checksumXOR to: %x", checksumXOR);
       checksumXOR_expected = checksumXOR;
    }

    logprintf(LOG_DEBUG, "checksumXOR: %x (%x %x)\n",checksumXOR, calculatedChecksum, messageChecksum);

	probe1 -= 532;
	probe2 -= 532;

	logprintf(LOG_INFO, "Probe 1: %d\n", probe1);
	logprintf(LOG_INFO, "Probe 2: %d\n", probe2);

	createMessage(probe1, probe2, checksumXOR);
	storeMessage();
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
	options_add(&maverick->options, 'i', "id",          OPTION_HAS_VALUE, DEVICES_ID,    JSON_NUMBER, NULL, "^([0-9]+)$");
	options_add(&maverick->options, 't', "temperature", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, NULL);
	options_add(&maverick->options, 'b', "bbq",         OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, NULL);

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
