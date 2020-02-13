/*
 * viplscannerinit.h
 *
 *  Created on: 16-Sep-2019
 *  Author: Snehasish Kar
 */

#ifndef INCLUDE_VIPLSCANNERINIT_H_
#define INCLUDE_VIPLSCANNERINIT_H_

#include "viplrfinterface.hpp"
#include "tcpserver.hpp"
#include <gnuradio/top_block.h>
#include <uhd/types/io_type.hpp>
#include <gnuradio/uhd/usrp_source.h>

extern char GUI_SERVER_IP_DEFINED[16];

#define COLD_START_TIME 1000

class vipl_scanner_init {
	vipl_rf_interface vipl_rf_interface_obj;
public:
	vipl_scanner_init();
	virtual ~vipl_scanner_init();
	void dequeue(tcp_server *tcpServer_gui);
	void start(void);
	static void start_streaming(void);
	static void start_filter_iq_sharing(struct gsm_channel_data channel_data, cbuffercf *cb);
	static void start_baseband_processing(struct gsm_channel_data channel_data);
	static void ntwrk_scan_start_fft(struct gsm_channel_data channel_data, tcp_server *tcpServer_gui, gr::uhd::usrp_source::sptr uhd_src);
	void join_threads();
	static void start_ntwrk_scan_process(struct gsm_channel_data channel_data, char *file_name, gr::top_block_sptr tb);
	static void switch_channel_matrix(struct gsm_channel_data channel_data,  int32_t portno, char *filename, cbuffercf *cb, bool uplink);
	static void switch_channel_matrix_uplink(struct gsm_channel_data channel_data, int32_t portno, char *filename, cbuffercf *cb_up, bool uplink);
	static int32_t count_channel(enum band_details band, int16_t *arfcn_list, int32_t arfcn_count);
	static void demod_agcch(struct gsm_channel_data *channel_data, bool uplink, gr::uhd::usrp_source::sptr uhd_src, int32_t *portno, int32_t num_channel, int32_t num_port);
	static std::complex<float> filter_downsample(const std::complex<float> input[],int32_t d_align, float** d_aligned_taps, int32_t d_ntaps);
	static void get_freq_range_from_arfcn(enum band_details band, int16_t arfcn, struct config *config_to_return);
	static void group_by(enum band_details band, int16_t *arfcn_list, int16_t arfcn_list_by_group[][8], int32_t total_arfcn_count, int32_t *max_rows);
};

struct gsmtap_hdr {
    uint8_t version;        /* version, set to 0x01 currently */
    uint8_t hdr_len;        /* length in number of 32bit words */
    uint8_t type;           /* see GSMTAP_TYPE_* */
    uint8_t timeslot;       /* timeslot (0..7 on Um) */
    uint16_t arfcn;         /* ARFCN (frequency) */
    int8_t signal_dbm;      /* signal level in dBm */
    int8_t snr_db;          /* signal/noise ratio in dB */
    uint32_t frame_number;  /* GSM Frame Number (FN) */
    uint8_t sub_type;       /* Type of burst/channel, see above */
    uint8_t antenna_nr;     /* Antenna Number */
    uint8_t sub_slot;       /* sub-slot within timeslot */
    uint8_t btsid;            /* reserved for future use (RFU) */
};

struct config{
	double samples_per_burst;
	double bandwidth;
	double sample_rate;
	double dl_freq;
	double ul_freq;
};

double convert_arfcn_to_freq(enum band_details band, int16_t arfcn, bool uplink);

#endif /* INCLUDE_VIPLSCANNERINIT_H_ */
