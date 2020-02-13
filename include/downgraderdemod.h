/*
 * downgraderdemod.h
 *
 *  Created on: 22-Jan-2020
 *      Author: Snehasish Kar
 */

#ifndef INCLUDE_DOWNGRADERDEMOD_H_
#define INCLUDE_DOWNGRADERDEMOD_H_

#include "../include/downgrader.h"
#include "viplscannerinit.h"

class downgrader_demod {
public:
	downgrader_demod();
	virtual ~downgrader_demod();
	static void transmit_receive_burst(uhd::usrp::multi_usrp::sptr usrp, double samples_per_burst, char *fifo_name);
	void init_downgrader(enum band_details band, int16_t arfcn, uhd::usrp::multi_usrp::sptr usrp, int32_t mBoard, double samples_per_burst, int8_t chain_num, int8_t num_chain);
};


#endif /* INCLUDE_DOWNGRADERDEMOD_H_ */
