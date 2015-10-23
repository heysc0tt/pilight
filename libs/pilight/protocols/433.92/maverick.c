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
#define MIN_LONG_PULSE			4000
#define MAX_LONG_PULSE			6000
#define RAW_LENGTH              50

static int validate(void) {
	return 0;
	// if(maverick->rawlen == RAW_LENGTH) {
	// 	if(maverick->raw[maverick->rawlen-1] >= (MIN_PULSE_LENGTH*PULSE_DIV) &&
	// 		maverick->raw[maverick->rawlen-1] <= (MAX_PULSE_LENGTH*PULSE_DIV)) {
	// 		return 0;
	// 	}
	// }

	// return -1;
}

static void createMessage(int id, int systemcode, int unit, int state) {
	maverick->message = json_mkobject();
	json_append_member(maverick->message, "id", json_mknumber(id, 0));
	json_append_member(maverick->message, "systemcode", json_mknumber(systemcode, 0));
	json_append_member(maverick->message, "unit", json_mknumber(unit, 0));
	if(state == 0) {
		json_append_member(maverick->message, "state", json_mkstring("on"));
	} else {
		json_append_member(maverick->message, "state", json_mkstring("off"));
	}
}

static void parseCode(void) {
//	int binary[RAW_LENGTH/2], x = 0, i = 0;
//	int id = -1, state = -1, unit = -1, systemcode = -1;
	int x=0,i=0;

	for(x=0;x<100;x+=2) {
		if(maverick->raw[x] > MIN_LONG_PULSE) {
			printf("Long: %d\n", maverick->raw[x]);
		} else if(maverick->raw[x] > MAX_PULSE_LENGTH) {
			printf("Medium: %d\n", maverick->raw[x]);
		} else {
			printf("Short: %d\n", maverick->raw[x]);
		}
	}

	// id = binToDecRev(binary, 0, 5);
	// systemcode = binToDecRev(binary, 6, 19);
	// unit = binToDecRev(binary, 21, 23 );
	// state = binary[20];
	// createMessage(id, systemcode, unit, state);
}

static void createLow(int s, int e) {
	int i;
	for(i=s;i<=e;i+=2) {
		maverick->raw[i]=(PULSE_MULTIPLIER*AVG_PULSE_LENGTH);
		maverick->raw[i+1]=(AVG_PULSE_LENGTH);
	}
}
static void createHigh(int s, int e) {
	int i;
	for(i=s;i<=e;i+=2) {
		maverick->raw[i]=(AVG_PULSE_LENGTH);
		maverick->raw[i+1]=(PULSE_MULTIPLIER*AVG_PULSE_LENGTH);
	}
}

static void clearCode(void) {
	createHigh(0,47);
}

static void createId(int id) {
	int binary[255];
	int length = 0;
	int i=0, x=0;

	length = decToBinRev(id, binary);
	for(i=0;i<=length;i++) {
		if(binary[i]==1) {
			x=i*2;
			createLow(11-(x+1), 11-x);
		}
	}
}


static void createSystemCode(int systemcode) {
	int binary[255];
	int length = 0;
	int i=0, x=0;

	length = decToBinRev(systemcode, binary);
	for(i=0;i<=length;i++) {
		if(binary[i]==1) {
			x=i*2;
			createLow(39-(x+1), 39-x);
		}
	}
}

static void createUnit(int unit) {
	int binary[255];
	int length = 0;
	int i=0, x=0;

	length = decToBinRev(unit, binary);
	for(i=0;i<=length;i++) {
		if(binary[i]==1) {
			x=i*2;
			createLow(47-(x+1), 47-x);
		}
	}
}

static void createState(int state) {
	if(state == 0) {
		createLow(40, 41);
	}
}


static void createFooter(void) {
	maverick->raw[48]=(AVG_PULSE_LENGTH);
	maverick->raw[49]=(PULSE_DIV*AVG_PULSE_LENGTH);
}

static int createCode(JsonNode *code) {
	int id = -1;
	int systemcode = -1;
	int unit = -1;
	int state = -1;
	double itmp = -1;

	if(json_find_number(code, "id", &itmp) == 0)
		id = (int)round(itmp);
	if(json_find_number(code, "systemcode", &itmp) == 0)
		systemcode = (int)round(itmp);
	if(json_find_number(code, "unit", &itmp) == 0)
		unit = (int)round(itmp);
	if(json_find_number(code, "off", &itmp) == 0)
		state=1;
	if(json_find_number(code, "on", &itmp) == 0)
		state=0;

	if(id == -1 || systemcode == -1 || unit == -1 || state == -1) {
		logprintf(LOG_ERR, "maverick: insufficient number of arguments");
		return EXIT_FAILURE;
	} else if(id > 63 || id < 0) {
		logprintf(LOG_ERR, "maverick: invalid id range");
		return EXIT_FAILURE;
	} else if(systemcode > 16999 || systemcode < 0) {
		logprintf(LOG_ERR, "maverick: invalid systemcode range");
		return EXIT_FAILURE;
	} else if(unit > 7 || unit < 0) {
		logprintf(LOG_ERR, "maverick: invalid unit range");
		return EXIT_FAILURE;
	} else {
		createMessage(id, systemcode, unit, state);
		clearCode();
		createId(id);
		createSystemCode(systemcode);
		createState(state);
		createUnit(unit);
		createFooter();
		maverick->rawlen = RAW_LENGTH;
		state = 0;
	}
	return EXIT_SUCCESS;
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
	maverick->minrawlen = RAW_LENGTH;
	maverick->maxrawlen = RAW_LENGTH;
	maverick->maxgaplen = MAX_PULSE_LENGTH*PULSE_DIV;
	maverick->mingaplen = MIN_PULSE_LENGTH*PULSE_DIV;

	// options_add(&maverick->options, 't', "on", OPTION_NO_VALUE, DEVICES_STATE, JSON_STRING, NULL, NULL);
	// options_add(&maverick->options, 'f', "off", OPTION_NO_VALUE, DEVICES_STATE, JSON_STRING, NULL, NULL);
	// options_add(&maverick->options, 'u', "unit", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "^([0-7])$");
	// options_add(&maverick->options, 's', "systemcode", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "^([0-9]{1,4}|1[0-6][0-9]{3})$");
	options_add(&maverick->options, 'i', "id", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "^([0-3])$");

	// options_add(&maverick->options, 0, "readonly", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *)0, "^[10]{1}$");

	maverick->parseCode=&parseCode;
	maverick->createCode=&createCode;
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