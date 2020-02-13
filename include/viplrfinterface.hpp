/*
 * viplrfinterface.h
 *
 *  Created on: 01-Apr-2019
 *      Author: root
 */

#ifndef INCLUDE_VIPLRFINTERFACE_HPP_
#define INCLUDE_VIPLRFINTERFACE_HPP_

#include <iostream>
#include <string.h>
#include <stdint.h>
#include <queue>

#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/error.h>

#if 0
#include <uhd/device3.hpp>
#include <uhd/rfnoc/block_id.hpp>
#include <uhd/rfnoc/radio_ctrl.hpp>
#include <uhd/rfnoc/block_ctrl.hpp>
#include <uhd/rfnoc/ddc_block_ctrl.hpp>
#include <uhd/rfnoc/source_block_ctrl_base.hpp>
#endif

#include <semaphore.h>

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#include "../include/liquid.h"

//#include "ringbuffer.hpp"

#define rx 0
#define tx 1
#define ntwrk_scan 2
#define buffer_time_in_secs 15
#define GPS_WAIT_TIME_IN_SECONDS 3
#define MAX_DUPLEX_CHAN 64
#define MAX_SIMPLEX_CHAN 128
#define MODE_SIMPLEX 1
#define MODE_DUPLEX 2

#define GSM_450_freq 460.6e6
#define GSM_800_freq 935e6
#define GSM_850_freq 869.2e6
#define GSM_900_freq 935.2e6
#define DCS_1800_freq 1805.2e6
#define PCS_1900_freq 1930.2e6

#define DEFAULT_GAIN_DWNLINK 91 //25
#define DEFAULT_GAIN_UPLINK 75 //25

#define DEFAULT_GAIN_DWNLINK_NORMAL_SCAN 50 //25
#define DEFAULT_GAIN_UPLINK_NORMAL_SCAN 75 //25

extern char IP_ADDR[50];
extern bool stop_rx;
extern bool stop_gps;

#define SAMPLE_RATE_Cellular_GSM_450 (2*7e6)
#define SAMPLE_RATE_Cellular_GSM_800 (2*35e6)
#define SAMPLE_RATE_Cellular_GSM_850 (2*25e6)
#define SAMPLE_RATE_Cellular_GSM_900 (2*25000000)
#define SAMPLE_RATE_Cellular_DCS_1800 (2*75e6)
#define SAMPLE_RATE_Cellular_PCS_1900 (2*60e6)
#define NUM_OF_RFNOC_RADIO_BLOCKS 2
#define NUM_OF_RFNOC_DDC_BLOCKS 4

enum band_details {gsm_450,  gsm_800, gsm_850, gsm_900, dcs_1800, pcs_1900 };

struct vipl_rf_tap {
	uint8_t channel;
	bool is_saturating;
	bool is_antenna_connected;
	bool is_receiving_gps;
	float gain;
	double latitude;
	double longitude;
	int32_t altitude;
	int32_t no_of_satellite;
};

struct GSMConfig{
	int8_t channel_mode;
	int32_t gain;
	int32_t mBoard;
	double dl_freq_a;
	double ul_freq_a;
	double dl_freq_b;
	double ul_freq_b;
	double sampleRate;
	double bandwidth;
	double samples_per_burst;
};

#if 0
struct RFNOC_Config{
	size_t spp;
	int32_t db_no;
	int32_t radio_channel_no;
	size_t chain_no;
	double primary_ddc_freq;
	double primary_ddc_decim_rate;
	double secondary_ddc_freq;
	double secondary_ddc_decim_rate;
	uhd::rfnoc::radio_ctrl::sptr radio_ctrl[NUM_OF_RFNOC_RADIO_BLOCKS];
	uhd::rfnoc::ddc_block_ctrl::sptr ddc_ctrl[NUM_OF_RFNOC_DDC_BLOCKS];
	uhd::rfnoc::source_block_ctrl_base::sptr splitstream_ctrl[NUM_OF_RFNOC_RADIO_BLOCKS];
};
#endif

extern uhd::time_spec_t time_spec;

class vipl_rf_interface {
public:
	bool tuned;
	int32_t mboardCount;
	double sample_rate;
	double freq_rx_board_a;
	double freq_rx_board_b;
	double freq_tx_board_a;
	double freq_tx_board_b;
	double bandwidth;
	double lo_offset;
	float gain;
	uint8_t channel;
	std::string subdev;
	uint8_t no_of_channels;
	uhd::usrp::multi_usrp::sptr usrp;
	//uhd::device3::sptr usrp_rfnoc;
	struct GSMConfig config;
	vipl_rf_interface();
	virtual ~vipl_rf_interface();
	int8_t set_rx_freq(uhd::usrp::multi_usrp::sptr usrp, uint8_t channel, double freq);
	int8_t set_rx_rate(uhd::usrp::multi_usrp::sptr usrp, uint8_t channel, double samp_rate);
	int8_t set_tx_freq(double freq, int8_t channel);
	int8_t set_gain(uhd::usrp::multi_usrp::sptr usrp, double gain, int8_t channel);
	void setup(uhd::usrp::multi_usrp::sptr usrp, int32_t mBoard, struct GSMConfig config, int32_t mode, int8_t chain_num);
	//void flush(int32_t _nchan, uhd::rx_streamer::sptr rxstream);
	//void rfnoc_conn_setup(uhd::device3::sptr usrp_rfnoc, struct RFNOC_Config rfnoc_config);
	//void rfnoc_setup(uhd::device3::sptr usrp_rfnoc, int32_t mBoard, struct GSMConfig config, struct RFNOC_Config rfnoc_config);
	//void rfnoc_start_streaming(uhd::device3::sptr usrp_rfnoc, cbuffercf *cb, unsigned long long total_no_samps, struct RFNOC_Config rfnoc_config);
	//void change_freq(uint8_t mode, uint8_t db_board, double freq);
	static void start_streaming(uhd::usrp::multi_usrp::sptr usrp, double samples_per_burst, int32_t channel_no, cbuffercf *cb, unsigned long long total_no_samps);
	static void get_gps_val(/*vipl_rf_interface vipl_rf_interface_obj*/uhd::usrp::multi_usrp::sptr usrp, bool tuned);
	bool lock_gps(uhd::usrp::multi_usrp::sptr usrp, int32_t mBoard);
	sem_t sem_status_stream;
};

extern struct vipl_rf_tap rftap;
//extern boost::mutex mutex_lock;
extern boost::mutex rftap_sync;
extern boost::mutex write_lock;
extern double change_freq_val;
extern bool ready;
extern cbuffercf cb1;
extern cbuffercf cb2;
extern cbuffercf cb3;
extern cbuffercf cb4;
extern cbuffercf cb5;
extern cbuffercf cb6;
extern cbuffercf cb7;
extern cbuffercf cb8;
extern cbuffercf cb9;
extern cbuffercf cb10;
extern cbuffercf cb11;
extern cbuffercf cb12;
extern cbuffercf cb13;
extern cbuffercf cb14;
extern cbuffercf cb15;
extern cbuffercf cb16;
extern cbuffercf cb17;
extern cbuffercf cb18;
extern sem_t stop_process;
extern sem_t lock;
extern sem_t stop_receive_gr;
extern char *ntwrkscan;
extern char *signalling;
extern char *traffic;
extern bool d_write_success;
extern bool is_tuned;
extern bool ENABLE_RFNOC;
extern double max_samps_supported_by_usrp;
//extern std::complex<float> store_complex_data[20][50000000];
#endif /* INCLUDE_VIPLRFINTERFACE_HPP_ */
