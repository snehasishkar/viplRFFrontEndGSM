/*
 * downgrader.h
 *
 *  Created on: 06-Jan-2020
 *  Author: Snehasish Kar
 */

#ifndef INCLUDE_DOWNGRADER_H_
#define INCLUDE_DOWNGRADER_H_

#include "../include/viplrfinterface.hpp"

#define DEFAULT_GAIN_TX_DOWNGRADER 10
#define DEFAULT_GAIN_RX_DOWNGRADER 20

struct GSMConfig_downgrader{
	int8_t channel_mode;
	int32_t rx_gain;
	int32_t tx_gain;
	int32_t mBoard;
	double dl_freq_a;
	double ul_freq_a;
	double sampleRate;
	double bandwidth;
	double samples_per_burst;
};

class downgrader {
public:
	downgrader();
	virtual ~downgrader();
	int8_t set_rx_freq(uhd::usrp::multi_usrp::sptr usrp, uint8_t channel, double freq);
	int8_t set_tx_freq(uhd::usrp::multi_usrp::sptr usrp, double freq, int8_t channel);
	int8_t set_rx_rate(uhd::usrp::multi_usrp::sptr usrp, uint8_t channel, double samp_rate);
	int8_t set_tx_rate(uhd::usrp::multi_usrp::sptr usrp, uint8_t channel, double samp_rate);
	int8_t set_rx_gain(uhd::usrp::multi_usrp::sptr usrp, uint8_t channel, double samp_rate);
	int8_t set_tx_gain(uhd::usrp::multi_usrp::sptr usrp, uint8_t channel, double samp_rate);
	bool lock_gps(uhd::usrp::multi_usrp::sptr usrp, int32_t mBoard);
	void setup(uhd::usrp::multi_usrp::sptr usrp, int32_t mBoard, struct GSMConfig_downgrader config, int8_t chain_num);
	static void start_rx_streaming(uhd::usrp::multi_usrp::sptr usrp, double samples_per_burst, int32_t channel_no, cbuffercf *cb, unsigned long long total_no_samps);
	static void start_tx_streaming(uhd::usrp::multi_usrp::sptr usrp, double samples_per_burst, int32_t channel_no, cbuffercf *cb, unsigned long long total_no_samps);
};





#endif /* INCLUDE_DOWNGRADER_H_ */
