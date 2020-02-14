/*
 * viplscannerinit.cpp
 *
 *  Created on: 16-Sep-2019
 *  Author: Snehasish Kar
 */

#include "../include/viplscannerinit.h"

#include "../include/tcpserver.hpp"
#include <sys/types.h>
#include <sys/stat.h>

#include <gnuradio/filter/firdes.h>
#include <gnuradio/blocks/file_source.h>
//#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/throttle.h>
#include <gnuradio/filter/fir_filter_ccf.h>
#include <gnuradio/blocks/file_descriptor_source.h>
#include <gnuradio/zeromq/push_sink.h>
#include <gnuradio/blocks/null_source.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/zeromq/pull_source.h>
#include <gnuradio/filter/freq_xlating_fir_filter_ccf.h>
#include <gnuradio/blocks/file_descriptor_source.h>
#include <gnuradio/blocks/pdu_to_tagged_stream.h>
#include <gnuradio/zeromq/push_msg_sink.h>
#include <gnuradio/zeromq/pub_msg_sink.h>
#include <gnuradio/zeromq/pub_sink.h>
#include <gnuradio/zeromq/sub_msg_source.h>

#include <gnuradio/blocks/file_source.h>
#include <grgsm/flow_control/uplink_downlink_splitter.h>
#include <grgsm/decoding/tch_f_decoder.h>
#include <grgsm/demapping/tch_f_chans_demapper.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/pdu_to_tagged_stream.h>
#include <grgsm/misc_utils/msg_to_tag.h>
#include <grgsm/misc_utils/controlled_rotator_cc.h>
#include <grgsm/misc_utils/controlled_fractional_resampler_cc.h>
#include <grgsm/demapping/universal_ctrl_chans_demapper.h>
#include <grgsm/decoding/control_channels_decoder.h>
#include <grgsm/receiver/receiver.h>
#include <grgsm/receiver/clock_offset_control.h>
#include <grgsm/misc_utils/message_file_sink.h>
#include <sched.h>


#include <pmt/pmt.h>
#include <fftw3.h>
#include <errno.h>
#include <fcntl.h>
#include <volk/volk.h>

cbuffercf cb1;
cbuffercf cb2;
cbuffercf cb3;
cbuffercf cb4;
cbuffercf cb5;
cbuffercf cb6;
cbuffercf cb7;
cbuffercf cb8;
cbuffercf cb9;
cbuffercf cb10;
cbuffercf cb11;
cbuffercf cb12;
cbuffercf cb13;
cbuffercf cb14;
cbuffercf cb15;
cbuffercf cb16;
cbuffercf cb17;
cbuffercf cb18;

sem_t start_streaming_init;
sem_t start_streaming_init_uplink;

uhd::time_spec_t time_spec;

boost::mutex ntwrk_scan_lock;
sem_t start_read;
bool rx_started = false;
bool stop_demd = false;
bool uplink_completed = false;
bool downlink_started = false;
bool uplink_started = false;

vipl_scanner_init::vipl_scanner_init() {
}

vipl_scanner_init::~vipl_scanner_init() {
	// TODO Auto-generated destructor stub
}

struct gsm_channel_data {
	int8_t chain_num;
	int8_t num_max_chan;
	int16_t arfcn_start;
	int16_t arfcn_list[124];
	int32_t num_channels;
	double center_arfcn;
	double samples_per_burst;
	double sample_rate;
	unsigned long long total_no_of_samps;
	cbuffercf *cb_sub_channels;
	enum band_details band;
	struct GSM_PARAM_FROM_UI gsm_param;
};

double convert_arfcn_to_freq(enum band_details band, int16_t arfcn, bool uplink = false) {
	double freq = 0.00;
	if(uplink){
		switch (band){
		case gsm_450: freq = ((450.6 + 0.2*(arfcn-259)));
					  break;
		case gsm_850: freq = (824.2+0.2*(arfcn-128));
					  break;
		case gsm_800: freq = (824.2+0.2*(arfcn-128));
					  break;
		case gsm_900: freq = (890+0.2*arfcn);
					  break;
		case dcs_1800: freq = (1710.2+0.2*(arfcn-512));
					   break;
		case pcs_1900: freq = (1850.2 + 0.2*(arfcn-512));
					   break;
		}
		return (freq*1e6);
	}
	switch (band){
	case gsm_450: freq = ((450.6 + 0.2*(arfcn-259))+10);
				  break;
	case gsm_850: freq = (824.2+0.2*(arfcn-128))+45;
				  break;
	case gsm_800: freq = (824.2+0.2*(arfcn-128))+45;
				  break;
	case gsm_900: freq = (890+0.2*arfcn)+45;
				  break;
	case dcs_1800: freq = (1710.2+0.2*(arfcn-512)) + 95;
				   break;
	case pcs_1900: freq = (1850.2 + 0.2*(arfcn-512))+80;
				   break;
	}
	return (freq*1e6);
}


static void vipl_scanner_init::start_baseband_processing(struct gsm_channel_data channel_data) {
	uint32_t h_len=12046; //filter length
	double num_samples_read = 0x00;
	double num_sample_requested = channel_data.samples_per_burst;
	std::complex<float> samp_buffer_for_bb_processing[channel_data.num_channels][(int32_t)num_sample_requested]={{0x00}};
	while(true) {
		for(int32_t i=0; i<channel_data.num_channels; i++) {
			std::complex<float> *buffer_read;
			cbuffercf_read(channel_data.cb_sub_channels[i], (uint32_t)num_sample_requested, &buffer_read, (uint32_t *)&num_samples_read);
			cbuffercf_release(channel_data.cb_sub_channels[i],(uint32_t)num_sample_requested);
			memcpy(&samp_buffer_for_bb_processing[(int32_t)num_sample_requested*i], buffer_read, sizeof(std::complex<float>)*channel_data.samples_per_burst);
		}
	}
}

std::complex<float> vipl_scanner_init::filter_downsample(const std::complex<float> input[], int32_t d_align, float** d_aligned_taps, int32_t d_ntaps){
	std::complex<float> *d_output = (std::complex<float>*)volk_malloc(1 * sizeof(std::complex<float>), d_align);
	const std::complex<float>* ar = (std::complex<float>*)((size_t)input & ~(d_align - 1));
	unsigned al = input - ar;
	volk_32fc_32f_dot_prod_32fc_a(d_output, ar, d_aligned_taps[al], (d_ntaps + al));
	return *d_output;
}

static void vipl_scanner_init::switch_channel_matrix_uplink(struct gsm_channel_data channel_data, int32_t portno, char *filename, cbuffercf *cb_up, bool uplink){
	uint32_t num_samples_read = 0x00, num_samples_up = 0x00;
	int32_t fd_down[8] = {0x00}, fd_up[8] = {0x00};
	float delta = 0.00;
	char fifo_name_up[channel_data.num_channels][100]={0x00};
	boost::thread_group tid_group;
	lv_32fc_t *buffer_read_up;
	float cuttoff = 195e3/channel_data.sample_rate;//495e3/channel_data.sample_rate;
	float stop_band_Attenuation= 60;
	float filter_transition = 5e3/channel_data.sample_rate;
	uint32_t filter_Tap_len = estimate_req_filter_len(filter_transition,stop_band_Attenuation);
	float filter_Tap[filter_Tap_len]={0.00};
	liquid_firdes_kaiser(filter_Tap_len,cuttoff,stop_band_Attenuation,0.00f,filter_Tap);
	struct stat st;
	int32_t count = 0x00;
	uplink_started = true;
	puts("switch matrix uplink launched");
	for(int32_t i=0;i<channel_data.num_channels;i++){
		sprintf(fifo_name_up[i],"/tmp/fifo_%d_up.bin",channel_data.arfcn_list[i]);
		if(stat(fifo_name_up[i],&st)){
			if(mkfifo(fifo_name_up[i], 0666)==-1){
				vipl_printf("error: unable to create FIFO", error_lvl, __FILE__, __LINE__);
			}else{
				vipl_printf("info: FIFO already present not deleting!!", error_lvl, __FILE__, __LINE__);
			if(std::remove(fifo_name_up[i])==-1)
				vipl_printf("error: unable to delete captured bin data", error_lvl, __FILE__, __LINE__);
				if(mkfifo(fifo_name_up[i], 0666)==-1){
					vipl_printf("error: unable to create FIFO", error_lvl, __FILE__, __LINE__);
			}
		  }
		}
	}
	gr::top_block_sptr tb[channel_data.num_channels];
	for(int32_t i=0;i<channel_data.num_channels;i++){
		label_fileread_up:
		if((fd_up[i] = open(fifo_name_up[i],O_WRONLY))<=0) {
			vipl_printf("error: unable to open FIFO uplink in switch channel matrix uplink!!", error_lvl, __FILE__, __LINE__);
			goto label_fileread_up;
		}
		if(!(fcntl(fd_up[i], F_SETPIPE_SZ, channel_data.samples_per_burst * 20)))
			vipl_printf("error: unable to set size of FIFO uplink", error_lvl, __FILE__, __LINE__);
	}
	sem_post(&start_streaming_init_uplink);
	int32_t row_count=0x00;
	count =0x00;
	while(!stop_rx) {
			label:
			cbuffercf_read(*cb_up, (uint32_t)channel_data.samples_per_burst, &buffer_read_up, (uint32_t *)&num_samples_up);
			cbuffercf_release(*cb_up, (uint32_t)num_samples_up);
			if(num_samples_up!=(uint32_t)channel_data.samples_per_burst) {
				usleep(1000);
				goto label;
			}
			if(error_lvl==3){
				fprintf(stderr,"\n======================================================\n");
				fprintf(stderr,"Number of samples read from circular buffer uplink %d", num_samples_up);
				fprintf(stderr,"\n======================================================\n");
			}
			//count++;
			for(int32_t i=0;i<channel_data.num_channels;i++){
				if(fd_up[i]<0)
					break;
				int32_t numBytesWrite = write(fd_up[i], buffer_read_up, sizeof(std::complex<float>)*(int32_t)num_samples_up);
				if(error_lvl==3) {
					fprintf(stderr,"\n======================================================\n");
					fprintf(stderr,"Number of samples write to FIFO %d uplink", numBytesWrite);
					fprintf(stderr,"\n======================================================\n");
				}
			}
	}
	if(cbuffercf_size(*cb_up)>0)
		cbuffercf_release(*cb_up,cbuffercf_size(*cb_up));
	uplink_completed = true;
	for(int32_t i=0;i<channel_data.num_channels;i++){
		//close(fd_up[i]);
		if(std::remove(fifo_name_up[i])==-1)
			vipl_printf("error: unable to delete captured bin data", error_lvl, __FILE__, __LINE__);
	}
	vipl_printf("info: switch matirx uplink closed", error_lvl, __FILE__, __LINE__);
}

#if 1
static void vipl_scanner_init::switch_channel_matrix(struct gsm_channel_data channel_data, int32_t portno, char *filename, cbuffercf *cb, bool uplink = false) {
	uint32_t num_samples_read = 0x00;
	int32_t fd[8] = {0x00};
	float delta = 0.00;
	char fifo_name[channel_data.num_channels][100]={0x00};
	boost::thread_group tid_group;
	uint32_t alignment = volk_get_alignment();
	lv_32fc_t *complex_output = (lv_32fc_t *)volk_malloc(sizeof(lv_32fc_t)*channel_data.samples_per_burst, alignment);
	lv_32fc_t *buffer_output = (lv_32fc_t *)volk_malloc(sizeof(lv_32fc_t)*channel_data.samples_per_burst, alignment);
	lv_32fc_t *buffer_read;
	float cuttoff = 195e3/channel_data.sample_rate;//495e3/channel_data.sample_rate;
	float stop_band_Attenuation= 60;
	float filter_transition = 5e3/channel_data.sample_rate;
	uint32_t filter_Tap_len = estimate_req_filter_len(filter_transition,stop_band_Attenuation);
	float filter_Tap[filter_Tap_len]={0.00};
	liquid_firdes_kaiser(filter_Tap_len,cuttoff,stop_band_Attenuation,0.00f,filter_Tap);
	printf("arfcn %d freq %f ofsset %f\n",channel_data.arfcn_start, convert_arfcn_to_freq(channel_data.band,channel_data.arfcn_start),(channel_data.center_arfcn-convert_arfcn_to_freq(channel_data.band,channel_data.arfcn_start)));
	delta = 2*M_PI*(channel_data.center_arfcn-convert_arfcn_to_freq(channel_data.band,channel_data.arfcn_start))/channel_data.sample_rate;
	if((channel_data.center_arfcn-convert_arfcn_to_freq(channel_data.band,channel_data.arfcn_start))>0){
		for(unsigned long long i=0;i<channel_data.samples_per_burst;i++) {
			complex_output[i]= std::exp(std::complex<float>(0,-i*delta));
		}
	}else{
		for(unsigned long long i=0;i<channel_data.samples_per_burst;i++) {
			complex_output[i]= std::exp(std::complex<float>(0,i*delta));
		}
	}
	struct stat st;
	downlink_started = true;
	for(int32_t i=0;i<channel_data.num_channels;i++){
		sprintf(fifo_name[i],"/tmp/fifo_%d_down.bin",channel_data.arfcn_list[i]);
		if(stat(fifo_name[i],&st)){
			if(mkfifo(fifo_name[i], 0666)==-1){
				vipl_printf("error: unable to create FIFO", error_lvl, __FILE__, __LINE__);
			}else{
				vipl_printf("info: FIFO already present not deleting!!", error_lvl, __FILE__, __LINE__);
				if(std::remove(fifo_name[i])==-1)
					vipl_printf("error: unable to delete captured bin data", error_lvl, __FILE__, __LINE__);
				if(mkfifo(fifo_name[i], 0666)==-1){
					vipl_printf("error: unable to create FIFO", error_lvl, __FILE__, __LINE__);
				}
			}
		}
	}
	gr::top_block_sptr tb[channel_data.num_channels];
	struct sched_param param;
	param.sched_priority = 90;
	for(int32_t i=0;i<channel_data.num_channels;i++) {
		char tb_name[50]={0x00};
		sprintf(tb_name,"demod_%d",channel_data.arfcn_list[i]);
		tb[i] = gr::make_top_block(tb_name);
		channel_data.arfcn_start = channel_data.arfcn_list[i];
		//boost::thread t1(demod_agcch, channel_data, fifo_name[i], tb[i], uplink);
		label_fileread:
		if((fd[i] = open(fifo_name[i],O_WRONLY))<=0) {
			vipl_printf("error: unable to open FIFO!!", error_lvl, __FILE__, __LINE__);
			goto label_fileread;
		}
		if(!(fcntl(fd[i], F_SETPIPE_SZ, channel_data.samples_per_burst * 20)))
		vipl_printf("error: unable to set size of FIFO", error_lvl, __FILE__, __LINE__);
	}
#if 0
	int32_t d_align = volk_get_alignment();
	int32_t d_naligned = std::max((size_t)1, d_align / sizeof(std::complex<float>));
	float** d_aligned_taps = (float**)malloc(d_naligned * sizeof(float*));
	std::vector<float> filter_Tap_gnuradio = gr::filter::firdes::low_pass(1.0, channel_data.sample_rate, cuttoff, 5e3, gr::filter::firdes::WIN_KAISER,6.76);
	for(int32_t i = 0; i < d_naligned; i++) {
		d_aligned_taps[i] = (float*)volk_malloc((filter_Tap_len + d_naligned - 1) * sizeof(float), d_align);
		std::fill_n(d_aligned_taps[i], filter_Tap_len + d_naligned - 1, 0);
		for (unsigned int j = 0; j < filter_Tap_gnuradio.size(); j++)
			d_aligned_taps[i][i + j] = filter_Tap_gnuradio[j];
	}
#endif
	sem_post(&start_streaming_init);
	int32_t row_count=0x00;
	float signal_pwr_mag = 0.00;
	while(!stop_rx) {
			label:
			cbuffercf_read(*cb, (uint32_t)channel_data.samples_per_burst, &buffer_read, (uint32_t *)&num_samples_read);
			cbuffercf_release(*cb, (uint32_t)num_samples_read);
			if(num_samples_read!=(uint32_t)channel_data.samples_per_burst) {
				usleep(1000);
				goto label;
			}
			if(error_lvl==3){
				fprintf(stderr,"\n======================================================\n");
				fprintf(stderr,"Number of samples read from circular buffer %d", num_samples_read);
				fprintf(stderr,"\n======================================================\n");
			}
			for(unsigned long long count = 0x00; count<100; count++){
				signal_pwr_mag += abs(buffer_read[count])*abs(buffer_read[count]);
			}
			rftap_sync.lock();
			if(signal_pwr_mag>1)
				rftap.is_saturating = true;
			signal_pwr_mag*=100;
			if(round(signal_pwr_mag)==0.0000){
			   rftap.is_antenna_connected = false;
			}else{
			   rftap.is_antenna_connected = true;
			}
			rftap_sync.unlock();
			//exit(0);
			for(int32_t i=0;i<channel_data.num_channels;i++){
				int32_t numBytesWrite = write(fd[i], buffer_read, sizeof(std::complex<float>)*(int32_t)num_samples_read);
				if(error_lvl==3) {
					fprintf(stderr,"\n======================================================\n");
					fprintf(stderr,"Number of samples write to FIFO %d", numBytesWrite);
					fprintf(stderr,"\n======================================================\n");
				}
			}
	}

	if(cbuffercf_size(*cb)>0)
		cbuffercf_release(*cb,cbuffercf_size(*cb));
	stop_demd = true;
	for(int32_t i=0;i<channel_data.num_channels;i++){
		close(fd[i]);
		if(std::remove(fifo_name[i])==-1)
			vipl_printf("error: unable to delete captured bin data", error_lvl, __FILE__, __LINE__);
	}
	vipl_printf("info: switch matirx closed", error_lvl, __FILE__, __LINE__);
}
#endif



static int32_t vipl_scanner_init::count_channel(enum band_details band, int16_t *arfcn_list, int32_t arfcn_count){
	int32_t i=0x00;
	switch(band){
	case gsm_450: while(i<arfcn_count){
	      	  	 	 if((arfcn_list[i]>=259) &&(arfcn_list[i]<=293))
	      	  		  i++;
	   	   	   	 }
		   	   	   	 break;
	case gsm_800: while(i<arfcn_count){
	   	   	   	 	 if((arfcn_list[i]>=128) &&(arfcn_list[i]<=251))
	   	   	   	 		 i++;
		   	 	 	 }
   		   	 	 	 break;

	case gsm_850: while(i<arfcn_count){
		 	 	 	 	 if(((arfcn_list[i]>=1) &&(arfcn_list[i]<=124))||((arfcn_list[i]>=975) &&(arfcn_list[i]<=1023)))
		 	 	 	 		 i++;
		   	   	   	 }
		 	 	 	 break;
	case gsm_900: while(i<arfcn_count){
		   	 	 	 	 if((arfcn_list[i]>=1) &&(arfcn_list[i]<=124))
		   	 	 	 		 i++;
		 	 	 	 }
   		 	 	 	 break;
	case dcs_1800: while(i<arfcn_count){
		 	 	 	 	 if((arfcn_list[i]>=512) &&(arfcn_list[i]<=885))
		 	 	 	 		 i++;
		 	 	 	  }
		 	 	 	  break;
	case pcs_1900: while(i<arfcn_count){
	   	   	   	  	  if((arfcn_list[i]>=512) &&(arfcn_list[i]<=810))
	   	   	   	  		  i++;
 		  	  	  	  }
 		  	  	  	  break;
	}
	return i;
}
#if 0
static void vipl_scanner_init::demod_agcch(struct gsm_channel_data channel_data, char *filename, gr::top_block_sptr tb, bool uplink) {

	try{
		char top_block_name[30]={0x00};
		float sample_rate_gsm = 2e6;
		sprintf(top_block_name,"agcch_demod_%d", channel_data.arfcn_start);
		std::vector<int32_t> arfcn_list;
		std::vector<int32_t> tseq;
		arfcn_list.push_back(channel_data.arfcn_start);
		tseq.push_back(channel_data.gsm_param.tseq);

		std::vector<int> downlink_starts_fn_mod51{0,0,2,2,2,2,6,6,6,6,0,0,12,12,12,12,16,16,16,16,0,0,22,22,22,22,26,26,26,26,0,0,32,32,32,32,36,36,36,36,0,0,42,42,42,42,46,46,46,46,0};
		std::vector<int> downlink_channel_types{0,0,0,0,0,0,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0};
		//std::vector<int> downlink_channel_types{0,0,1,1,1,1,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0};
		std::vector<int32_t> downlink_subslots{0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3,4,4,4,4,0,0,5,5,5,5,6,6,6,6,0,0,7,7,7,7,8,8,8,8,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3,4,4,4,4,0,0,5,5,5,5,6,6,6,6,0,0,7,7,7,7,8,8,8,8,0};
		std::vector<int32_t> uplink_starts_fn_mod51{0,0,0,0,0,0,6,6,6,6,10,10,10,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,37,37,37,37,41,41,41,41,0,0,47,47,47,47};
		std::vector<int32_t> uplink_channel_types{2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2};
		std::vector<int32_t> uplink_subslots{0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3,0,0,0,0,0,0,2,2,2,2,3,3,3,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3};

		#if 1
			std::vector<int> downlink_starts_fn_mod51_sdcch{0,0,0,0,4,4,4,4,8,8,8,8,12,12,12,12,16,16,16,16,20,20,20,20,24,24,24,24,28,28,28,28,32,32,32,32,36,36,36,36,40,40,40,40,44,44,44,44,0,0,0};
			std::vector<int> downlink_channel_types_sdcch{8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,0,0,0};
			std::vector<int> downlink_subslots_sdcch{0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7,0,0,0};
			std::vector<int> uplink_starts_fn_mod51_sdcch{0,0,0,0,4,4,4,4,8,8,8,8,0,0,0,15,15,15,15,19,19,19,19,23,23,23,23,27,27,27,27,31,31,31,31,35,35,35,35,39,39,39,39,43,43,43,43,47,47,47,47};
			std::vector<int> uplink_channel_types_sdcch{136,136,136,136,136,136,136,136,136,136,136,136,0,0,0,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,136,136,136,136};
			std::vector<int> uplink_subslots_sdcch{1,1,1,1,2,2,2,2,3,3,3,3,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7,0,0,0,0,5,5,5,5,6,6,6,6,7,7,7,7,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7,4,4,4,4};
		#endif
		//SDCCH4
		#if 0
			std::vector<int> downlink_starts_fn_mod51_sdcch{0,0,2,2,2,2,6,6,6,6,0,0,12,12,12,12,16,16,16,16,0,0,22,22,22,22,26,26,26,26,0,0,32,32,32,32,36,36,36,36,0,0,42,42,42,42,46,46,46,46,0};
			std::vector<int> downlink_channel_types_sdcch{0,0,1,1,1,1,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,7,7,7,7,7,7,7,7,0,0,7,7,7,7,7,7,7,7,0,0,135,135,135,135,135,135,135,135,0};
			std::vector<int> downlink_subslots_sdcch{0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,0,0,0,0,1,1,1,1,0,0,2,2,2,2,3,3,3,3,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,0,0,0,0,1,1,1,1,0,0,2,2,2,2,3,3,3,3,0,0,2,2,2,2,3,3,3,3,0};
			std::vector<int> uplink_starts_fn_mod51_sdcch{0,0,0,0,0,0,6,6,6,6,10,10,10,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,37,37,37,37,41,41,41,41,0,0,47,47,47,47};
			std::vector<int> uplink_channel_types_sdcch{7,7,7,7,0,0,135,135,135,135,135,135,135,135,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7,7,7,7,7,7,7,7,0,0,7,7,7,7};
			std::vector<int> uplink_subslots_sdcch{0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3,0,0,0,0,0,0,2,2,2,2,3,3,3,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3};
		#endif
		int32_t fd = 0x00, fd_up = 0x00;
		if(!uplink){
			fileopen:
			fd = open(filename, O_RDONLY);
			if(fd<=0x00){
				vipl_printf("error: unable to open FIFO downlink", error_lvl, __FILE__, __LINE__);
				goto fileopen;
			}
		}else{
			fileopen_downlink:
			fd = open(filename, O_RDONLY);
			if(fd<=0x00){
				vipl_printf("error: unable to open FIFO downlink", error_lvl, __FILE__, __LINE__);
				goto fileopen_downlink;
			}
			if(error_lvl==3)
				fprintf(stderr,"FIFO Downlink successfully opened!!\n");
			char fifoname[100]={0x00};
			memset(fifoname,0x00, sizeof(char)*100);
			sprintf(fifoname,"/tmp/fifo_%d_up.bin",channel_data.arfcn_start);
			fileopen_uplink:
			fd_up = open(fifoname, O_RDONLY);
			if(fd_up<=0x00){
				vipl_printf("error: unable to open FIFO uplink", error_lvl, __FILE__, __LINE__);
				goto fileopen_uplink;
			}
			if(error_lvl==3)
				fprintf(stderr,"FIFO Uplink successfully opened!!\n");
		}

		//gr::blocks::file_descriptor_source::sptr fs =  gr::blocks::file_descriptor_source::make(sizeof(std::complex<float>), fd, false);
		//gr::blocks::file_descriptor_source::sptr fs_1;
		//if(uplink)
		//	fs_1 =  gr::blocks::file_descriptor_source::make(sizeof(std::complex<float>), fd_up, false);

		//gr::zeromq::pull_source::sptr zero_mq_source = gr::zeromq::pull_source::make(sizeof(std::complex<float>), 1, zeromq_addr, 100, false, -1);gr::blocks::throttle::sptr throttle = gr::blocks::throttle::make(sizeof(std::complex<float>), channel_data.sample_rate, true);
		gr::gsm::msg_to_tag::sptr msg2tag_0 = gr::gsm::msg_to_tag::make();
		int32_t ppm = 0x00;
		gr::gsm::controlled_rotator_cc::sptr controlled_rotator_0 = gr::gsm::controlled_rotator_cc::make(ppm/1.0e6*2*M_PI*convert_arfcn_to_freq(channel_data.band, channel_data.arfcn_start)/(4*(1625000.0/6.0)));
		gr::gsm::controlled_fractional_resampler_cc::sptr controlled_fractional_resampler_0 = gr::gsm::controlled_fractional_resampler_cc::make(0,(sample_rate_gsm/(4*(1625000.0/6.0))));
		gr::gsm::msg_to_tag::sptr msg2tag_1 = gr::gsm::msg_to_tag::make();
		gr::gsm::controlled_rotator_cc::sptr controlled_rotator_1 = gr::gsm::controlled_rotator_cc::make(ppm/1.0e6*2*M_PI*convert_arfcn_to_freq(channel_data.band, channel_data.arfcn_start)/(4*(1625000.0/6.0)));
		gr::gsm::controlled_fractional_resampler_cc::sptr controlled_fractional_resampler_1 = gr::gsm::controlled_fractional_resampler_cc::make(0,(sample_rate_gsm/(4*(1625000.0/6.0))));
		gr::gsm::receiver::sptr receiver;
		if(!uplink)
			receiver = gr::gsm::receiver::make(4,arfcn_list,tseq,false);
		else
			receiver = gr::gsm::receiver::make(4,arfcn_list,tseq,true);
		receiver->set_thread_priority(99);
		gr::gsm::universal_ctrl_chans_demapper::sptr control_BCCH_demapper(gr::gsm::universal_ctrl_chans_demapper::make(0,downlink_starts_fn_mod51,downlink_channel_types,downlink_subslots,uplink_starts_fn_mod51,uplink_channel_types,uplink_subslots));
		gr::filter::firdes lowpass_taps;
		gr::filter::fir_filter_ccf::sptr lowpassFilter = gr::filter::fir_filter_ccf::make(1,lowpass_taps.low_pass(1,4*(1625000.0/6.0),125000,5000,lowpass_taps.WIN_KAISER,6.76));
		gr::filter::fir_filter_ccf::sptr lowpassFilter_1 = gr::filter::fir_filter_ccf::make(1,lowpass_taps.low_pass(1,4*(1625000.0/6.0),125000,5000,lowpass_taps.WIN_KAISER,6.76));
		gr::gsm::clock_offset_control::sptr clock_control_offset= gr::gsm::clock_offset_control::make(convert_arfcn_to_freq(channel_data.band,channel_data.arfcn_start),sample_rate_gsm,4);
		//gr::gsm::clock_offset_control::sptr clock_control_offset_1= gr::gsm::clock_offset_control::make(convert_arfcn_to_freq(channel_data.band,channel_data.arfcn_start, true),sample_rate_gsm,4);
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_1(gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)1,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch));
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_2(gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)2,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch));
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_3(gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)3,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch));
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_4(gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)4,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch));
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_5(gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)5,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch));
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_6(gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)6,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch));
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_7(gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)7,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch));
		char zeromq_addr[15]={0x00};
		gr::zeromq::pub_sink::sptr pubsink_0 = gr::zeromq::pub_sink::make(sizeof(std::complex<float>), 1, "tcp://127.0.0.1:1234", 100);
		pubsink_0->set_thread_priority(99);
		//gr::zeromq::pub_sink::sptr pubsink_1 = gr::zeromq::pub_sink::make(sizeof(std::complex<float>), 1, "tcp://127.0.0.1:1235", 100);
		sprintf(zeromq_addr, "tcp://127.0.0.1:%d",(1000+channel_data.arfcn_start));
		gr::zeromq::pub_msg_sink::sptr zmq_push = gr::zeromq::pub_msg_sink::make(zeromq_addr);
		float delta = ((channel_data.center_arfcn-convert_arfcn_to_freq(channel_data.band, channel_data.arfcn_start)))*(-1);
		std::cout<<delta<<" "<<channel_data.sample_rate<<std::endl;
		gr::filter::freq_xlating_fir_filter_ccf::sptr freq_decim = gr::filter::freq_xlating_fir_filter_ccf::make(channel_data.sample_rate/sample_rate_gsm,gr::filter::firdes::low_pass(1,channel_data.sample_rate,125e3,75e3,gr::filter::firdes::WIN_KAISER,6.76),delta,channel_data.sample_rate);
		gr::filter::freq_xlating_fir_filter_ccf::sptr freq_decim_1 = gr::filter::freq_xlating_fir_filter_ccf::make(channel_data.sample_rate/sample_rate_gsm,gr::filter::firdes::low_pass(1,channel_data.sample_rate,125e3,75e3,gr::filter::firdes::WIN_KAISER,6.76),delta,channel_data.sample_rate);
		FILE *fp =NULL;
		gr::gsm::control_channels_decoder::sptr control_channel_decoder_bcch = gr::gsm::control_channels_decoder::make(channel_data.arfcn_start);
		//gr::gsm::control_channels_decoder::sptr control_channel_decoder_sdcch_1 = gr::gsm::control_channels_decoder::make(channel_data.arfcn_start);
		//gr::gsm::control_channels_decoder::sptr control_channel_decoder_sdcch_2 = gr::gsm::control_channels_decoder::make(channel_data.arfcn_start);
		//gr::gsm::control_channels_decoder::sptr control_channel_decoder_sdcch_3 = gr::gsm::control_channels_decoder::make(channel_data.arfcn_start);
		//gr::gsm::control_channels_decoder::sptr control_channel_decoder_sdcch_4 = gr::gsm::control_channels_decoder::make(channel_data.arfcn_start);
		//gr::gsm::control_channels_decoder::sptr control_channel_decoder_sdcch_5 = gr::gsm::control_channels_decoder::make(channel_data.arfcn_start);
		//gr::gsm::control_channels_decoder::sptr control_channel_decoder_sdcch_6 = gr::gsm::control_channels_decoder::make(channel_data.arfcn_start);
		//gr::gsm::control_channels_decoder::sptr control_channel_decoder_sdcch_7 = gr::gsm::control_channels_decoder::make(channel_data.arfcn_start);

		char control_channel_decoder_filename[100]={0x00};
		sprintf(control_channel_decoder_filename,"%s/%d_agcch_%d.bin",signalling,channel_data.arfcn_start,std::time(nullptr));
		gr::gsm::message_file_sink::sptr fs_control_channel_decoder_bcch = gr::gsm::message_file_sink::make(control_channel_decoder_filename);
		memset(control_channel_decoder_filename, 0x00, sizeof(char)*100);
		sprintf(control_channel_decoder_filename,"%s/%d_agcch_%d.bin",signalling,channel_data.arfcn_start,std::time(nullptr));
		gr::gsm::message_file_sink::sptr fs_control_channel_decoder_sdcch_1 = gr::gsm::message_file_sink::make(control_channel_decoder_filename);
		memset(control_channel_decoder_filename, 0x00, sizeof(char)*100);
		sprintf(control_channel_decoder_filename,"%s/%d_agcch_%d.bin",signalling,channel_data.arfcn_start,std::time(nullptr));
		gr::gsm::message_file_sink::sptr fs_control_channel_decoder_sdcch_2 = gr::gsm::message_file_sink::make(control_channel_decoder_filename);
		memset(control_channel_decoder_filename, 0x00, sizeof(char)*100);
		sprintf(control_channel_decoder_filename,"%s/%d_agcch_%d.bin",signalling,channel_data.arfcn_start,std::time(nullptr));
		gr::gsm::message_file_sink::sptr fs_control_channel_decoder_sdcch_3 = gr::gsm::message_file_sink::make(control_channel_decoder_filename);
		memset(control_channel_decoder_filename, 0x00, sizeof(char)*100);
		sprintf(control_channel_decoder_filename,"%s/%d_agcch_%d.bin",signalling,channel_data.arfcn_start,std::time(nullptr));
		gr::gsm::message_file_sink::sptr fs_control_channel_decoder_sdcch_4 = gr::gsm::message_file_sink::make(control_channel_decoder_filename);
		memset(control_channel_decoder_filename, 0x00, sizeof(char)*100);
		sprintf(control_channel_decoder_filename,"%s/%d_agcch_%d.bin",signalling,channel_data.arfcn_start,std::time(nullptr));
		gr::gsm::message_file_sink::sptr fs_control_channel_decoder_sdcch_5 = gr::gsm::message_file_sink::make(control_channel_decoder_filename);
		memset(control_channel_decoder_filename, 0x00, sizeof(char)*100);
		sprintf(control_channel_decoder_filename,"%s/%d_agcch_%d.bin",signalling,channel_data.arfcn_start,std::time(nullptr));
		gr::gsm::message_file_sink::sptr fs_control_channel_decoder_sdcch_6 = gr::gsm::message_file_sink::make(control_channel_decoder_filename);
		memset(control_channel_decoder_filename, 0x00, sizeof(char)*100);
		sprintf(control_channel_decoder_filename,"%s/%d_agcch_%d.bin",signalling,channel_data.arfcn_start,std::time(nullptr));
		gr::gsm::message_file_sink::sptr fs_control_channel_decoder_sdcch_7 = gr::gsm::message_file_sink::make(control_channel_decoder_filename);
		float timediff = 0.00, timediff_rftap = 0.00;
		gr::blocks::file_source::sptr fs_0_0 = gr::blocks::file_source::make(sizeof(std::complex<float>), "/tmp/downlink", false);
		gr::blocks::file_source::sptr fs_0_1 = gr::blocks::file_source::make(sizeof(std::complex<float>), "/tmp/uplink", false);
		if(uplink)
			receiver->set_max_noutput_items(sample_rate_gsm*2);
		//receiver->max_output_buffer(0);
		std::cout<<"Number of samples received: "<<receiver->max_noutput_items()<<std::endl;
		//memset(control_channel_decoder_filename, 0x00, sizeof(char)*100);
		//sprintf(control_channel_decoder_filename,"/tmp/rf_data_%d.bin",std::time(nullptr));
		tb->connect(fs,0,freq_decim,0);
		//tb->connect(freq_decim,0,msg2tag_0,0);
		tb->connect(freq_decim,0,controlled_fractional_resampler_0,0);

		//tb->connect(msg2tag_0,0,controlled_fractional_resampler_0,0);
		tb->connect(controlled_fractional_resampler_0,0,controlled_rotator_0,0);
		tb->connect(controlled_rotator_0,0,receiver,0);
		//tb->connect(controlled_rotator_0,0, pubsink_0, 0);
		if(uplink){
			tb->connect(fs_1,0,freq_decim_1,0);
			//tb->connect(freq_decim_1,0,msg2tag_1,0);
			//tb->connect(msg2tag_1,0,controlled_fractional_resampler_1,0);
			tb->connect(freq_decim_1,0,controlled_fractional_resampler_0,0);
			tb->connect(controlled_fractional_resampler_1,0,controlled_rotator_1,0);
			tb->connect(controlled_rotator_1,0,receiver,1);
			//tb->connect(controlled_rotator_1,0,pubsink,0);
			tb->connect(controlled_rotator_1,0,pubsink_0,0);
		}
		tb->msg_connect(receiver,"measurements",clock_control_offset,"measurements");
		tb->msg_connect(clock_control_offset,"ctrl",msg2tag_0,"msg");
		if(uplink){
			tb->msg_connect(clock_control_offset,"ctrl",msg2tag_1,"msg");
		}
#if 1
		gr::gsm::tch_f_decoder::sptr tch_decoder_down_1 = gr::gsm::tch_f_decoder::make(gr::gsm::TCH_AFS12_2,true);
				gr::gsm::tch_f_chans_demapper::sptr tch_demapper_down_1 = gr::gsm::tch_f_chans_demapper::make(1);
				gr::gsm::tch_f_decoder::sptr tch_decoder_up_1 = gr::gsm::tch_f_decoder::make(gr::gsm::TCH_AFS12_2,true);
				gr::gsm::tch_f_chans_demapper::sptr tch_demapper_up_1 = gr::gsm::tch_f_chans_demapper::make(1);
				gr::gsm::uplink_downlink_splitter::sptr uplinkdownlinksplitter_1 = gr::gsm::uplink_downlink_splitter::make();

				gr::gsm::tch_f_decoder::sptr tch_decoder_down_2 = gr::gsm::tch_f_decoder::make(gr::gsm::TCH_AFS12_2,true);
				gr::gsm::tch_f_chans_demapper::sptr tch_demapper_down_2 = gr::gsm::tch_f_chans_demapper::make(2);
				gr::gsm::tch_f_decoder::sptr tch_decoder_up_2 = gr::gsm::tch_f_decoder::make(gr::gsm::TCH_AFS12_2,true);
				gr::gsm::tch_f_chans_demapper::sptr tch_demapper_up_2 = gr::gsm::tch_f_chans_demapper::make(2);
				gr::gsm::uplink_downlink_splitter::sptr uplinkdownlinksplitter_2 = gr::gsm::uplink_downlink_splitter::make();

				gr::gsm::tch_f_decoder::sptr tch_decoder_down_3 = gr::gsm::tch_f_decoder::make(gr::gsm::TCH_AFS12_2, true);
				gr::gsm::tch_f_chans_demapper::sptr tch_demapper_down_3 = gr::gsm::tch_f_chans_demapper::make(3);
				gr::gsm::tch_f_decoder::sptr tch_decoder_up_3 = gr::gsm::tch_f_decoder::make(gr::gsm::TCH_AFS12_2,true);
				gr::gsm::tch_f_chans_demapper::sptr tch_demapper_up_3 = gr::gsm::tch_f_chans_demapper::make(3);
				gr::gsm::uplink_downlink_splitter::sptr uplinkdownlinksplitter_3 = gr::gsm::uplink_downlink_splitter::make();

				gr::gsm::tch_f_decoder::sptr tch_decoder_down_4 = gr::gsm::tch_f_decoder::make(gr::gsm::TCH_AFS12_2,true);
				gr::gsm::tch_f_chans_demapper::sptr tch_demapper_down_4 = gr::gsm::tch_f_chans_demapper::make(4);
				gr::gsm::tch_f_decoder::sptr tch_decoder_up_4 = gr::gsm::tch_f_decoder::make(gr::gsm::TCH_AFS12_2,true);
				gr::gsm::tch_f_chans_demapper::sptr tch_demapper_up_4 = gr::gsm::tch_f_chans_demapper::make(4);
				gr::gsm::uplink_downlink_splitter::sptr uplinkdownlinksplitter_4 = gr::gsm::uplink_downlink_splitter::make();

				gr::gsm::tch_f_decoder::sptr tch_decoder_down_5 = gr::gsm::tch_f_decoder::make(gr::gsm::TCH_AFS12_2,true);
				gr::gsm::tch_f_chans_demapper::sptr tch_demapper_down_5 = gr::gsm::tch_f_chans_demapper::make(5);
				gr::gsm::tch_f_decoder::sptr tch_decoder_up_5 = gr::gsm::tch_f_decoder::make(gr::gsm::TCH_AFS12_2,true);
				gr::gsm::tch_f_chans_demapper::sptr tch_demapper_up_5 = gr::gsm::tch_f_chans_demapper::make(5);
				gr::gsm::uplink_downlink_splitter::sptr uplinkdownlinksplitter_5 = gr::gsm::uplink_downlink_splitter::make();

				gr::gsm::tch_f_decoder::sptr tch_decoder_down_6 = gr::gsm::tch_f_decoder::make(gr::gsm::TCH_AFS12_2,true);
				gr::gsm::tch_f_chans_demapper::sptr tch_demapper_down_6 = gr::gsm::tch_f_chans_demapper::make(6);
				gr::gsm::tch_f_decoder::sptr tch_decoder_up_6 = gr::gsm::tch_f_decoder::make(gr::gsm::TCH_AFS12_2,true);
				gr::gsm::tch_f_chans_demapper::sptr tch_demapper_up_6 = gr::gsm::tch_f_chans_demapper::make(6);
				gr::gsm::uplink_downlink_splitter::sptr uplinkdownlinksplitter_6 = gr::gsm::uplink_downlink_splitter::make();

				gr::gsm::tch_f_decoder::sptr tch_decoder_down_7 = gr::gsm::tch_f_decoder::make(gr::gsm::TCH_AFS12_2,true);
				gr::gsm::tch_f_chans_demapper::sptr tch_demapper_down_7 = gr::gsm::tch_f_chans_demapper::make(7);
				gr::gsm::tch_f_decoder::sptr tch_decoder_up_7 = gr::gsm::tch_f_decoder::make(gr::gsm::TCH_AFS12_2,true);
				gr::gsm::tch_f_chans_demapper::sptr tch_demapper_up_7 = gr::gsm::tch_f_chans_demapper::make(7);
				gr::gsm::uplink_downlink_splitter::sptr uplinkdownlinksplitter_7 = gr::gsm::uplink_downlink_splitter::make();

				gr::blocks::pdu_to_tagged_stream::sptr pdu_0 = gr::blocks::pdu_to_tagged_stream::make(gr::blocks::pdu::byte_t,"packet_len");
				gr::blocks::pdu_to_tagged_stream::sptr pdu_1 = gr::blocks::pdu_to_tagged_stream::make(gr::blocks::pdu::byte_t,"packet_len");
				gr::blocks::pdu_to_tagged_stream::sptr pdu_2 = gr::blocks::pdu_to_tagged_stream::make(gr::blocks::pdu::byte_t,"packet_len");
				gr::blocks::pdu_to_tagged_stream::sptr pdu_3 = gr::blocks::pdu_to_tagged_stream::make(gr::blocks::pdu::byte_t,"packet_len");
				gr::blocks::pdu_to_tagged_stream::sptr pdu_4 = gr::blocks::pdu_to_tagged_stream::make(gr::blocks::pdu::byte_t,"packet_len");
				gr::blocks::pdu_to_tagged_stream::sptr pdu_5 = gr::blocks::pdu_to_tagged_stream::make(gr::blocks::pdu::byte_t,"packet_len");
				gr::blocks::pdu_to_tagged_stream::sptr pdu_6 = gr::blocks::pdu_to_tagged_stream::make(gr::blocks::pdu::byte_t,"packet_len");
				gr::blocks::pdu_to_tagged_stream::sptr pdu_7 = gr::blocks::pdu_to_tagged_stream::make(gr::blocks::pdu::byte_t,"packet_len");

				gr::blocks::file_sink::sptr fs_0 = gr::blocks::file_sink::make(sizeof(uint8_t),"/opt/out0.amr", false);
				gr::blocks::file_sink::sptr fs_01 = gr::blocks::file_sink::make(sizeof(uint8_t),"/opt/out1.amr", false);
				gr::blocks::file_sink::sptr fs_2 = gr::blocks::file_sink::make(sizeof(uint8_t),"/opt/out2.amr", false);
				gr::blocks::file_sink::sptr fs_3 = gr::blocks::file_sink::make(sizeof(uint8_t),"/opt/out3.amr", false);
				gr::blocks::file_sink::sptr fs_4 = gr::blocks::file_sink::make(sizeof(uint8_t),"/opt/out4.amr", false);
				gr::blocks::file_sink::sptr fs_5 = gr::blocks::file_sink::make(sizeof(uint8_t),"/opt/out5.amr", false);
				gr::blocks::file_sink::sptr fs_6 = gr::blocks::file_sink::make(sizeof(uint8_t),"/opt/out6.amr", false);
				gr::blocks::file_sink::sptr fs_7 = gr::blocks::file_sink::make(sizeof(uint8_t),"/opt/out7.amr", false);

				tb->msg_connect(receiver,"C0", uplinkdownlinksplitter_1,"in");
						tb->msg_connect(uplinkdownlinksplitter_1, "downlink", tch_demapper_down_1,"bursts");
						//tb->msg_connect(tch_demapper_down_1,"tch_bursts",tch_decoder_down_1,"bursts");
						tb->msg_connect(uplinkdownlinksplitter_1, "uplink", tch_demapper_up_1,"bursts");
						tb->msg_connect(tch_demapper_up_1,"tch_bursts",tch_decoder_up_1,"bursts");

						tb->msg_connect(receiver,"C0", uplinkdownlinksplitter_2,"in");
						//tb->msg_connect(uplinkdownlinksplitter_2, "downlink", tch_demapper_down_2,"bursts");
						tb->msg_connect(tch_demapper_down_2,"tch_bursts",tch_decoder_down_2,"bursts");
						tb->msg_connect(uplinkdownlinksplitter_2, "uplink", tch_demapper_up_2,"bursts");
						tb->msg_connect(tch_demapper_up_2,"tch_bursts",tch_decoder_up_2,"bursts");

						tb->msg_connect(receiver,"C0", uplinkdownlinksplitter_3,"in");
						//tb->msg_connect(uplinkdownlinksplitter_3, "downlink", tch_demapper_down_3,"bursts");
						tb->msg_connect(tch_demapper_down_3,"tch_bursts",tch_decoder_down_3,"bursts");
						tb->msg_connect(uplinkdownlinksplitter_3, "uplink", tch_demapper_up_3,"bursts");
						tb->msg_connect(tch_demapper_up_3,"tch_bursts",tch_decoder_up_3,"bursts");

						tb->msg_connect(receiver,"C0", uplinkdownlinksplitter_4,"in");
						//tb->msg_connect(uplinkdownlinksplitter_4, "downlink", tch_demapper_down_4,"bursts");
						tb->msg_connect(tch_demapper_down_4,"tch_bursts",tch_decoder_down_4,"bursts");
						tb->msg_connect(uplinkdownlinksplitter_4, "uplink", tch_demapper_up_4,"bursts");
						tb->msg_connect(tch_demapper_up_4,"tch_bursts",tch_decoder_up_4,"bursts");

						tb->msg_connect(receiver,"C0", uplinkdownlinksplitter_5,"in");
						//tb->msg_connect(uplinkdownlinksplitter_5, "downlink", tch_demapper_down_5,"bursts");
						tb->msg_connect(tch_demapper_down_5,"tch_bursts",tch_decoder_down_5,"bursts");
						tb->msg_connect(uplinkdownlinksplitter_5, "uplink", tch_demapper_up_5,"bursts");
						tb->msg_connect(tch_demapper_up_5,"tch_bursts",tch_decoder_up_5,"bursts");

						tb->msg_connect(receiver,"C0", uplinkdownlinksplitter_6,"in");
						//tb->msg_connect(uplinkdownlinksplitter_6, "downlink", tch_demapper_down_6,"bursts");
						tb->msg_connect(tch_demapper_down_6,"tch_bursts",tch_decoder_down_6,"bursts");
						tb->msg_connect(uplinkdownlinksplitter_6, "uplink", tch_demapper_up_6,"bursts");
						tb->msg_connect(tch_demapper_up_6,"tch_bursts",tch_decoder_up_6,"bursts");

						tb->msg_connect(receiver,"C0", uplinkdownlinksplitter_7,"in");
						//tb->msg_connect(uplinkdownlinksplitter_7, "downlink", tch_demapper_down_7,"bursts");
						tb->msg_connect(tch_demapper_down_7,"tch_bursts",tch_decoder_down_7,"bursts");
						tb->msg_connect(uplinkdownlinksplitter_7, "uplink", tch_demapper_up_7,"bursts");
						tb->msg_connect(tch_demapper_up_7,"tch_bursts",tch_decoder_up_7,"bursts");

						tb->msg_connect(tch_decoder_up_1, "voice", pdu_0, "pdus");
						tb->msg_connect(tch_decoder_up_2, "voice", pdu_1, "pdus");
						tb->msg_connect(tch_decoder_up_3, "voice", pdu_2, "pdus");
						tb->msg_connect(tch_decoder_up_4, "voice", pdu_3, "pdus");
						tb->msg_connect(tch_decoder_up_5, "voice", pdu_4, "pdus");
						tb->msg_connect(tch_decoder_up_6, "voice", pdu_5, "pdus");
						tb->msg_connect(tch_decoder_up_7, "voice", pdu_6, "pdus");

						tb->connect(pdu_0,0, fs_0, 0);
						tb->connect(pdu_1,0, fs_01, 0);
						tb->connect(pdu_2,0, fs_2, 0);
						tb->connect(pdu_3,0, fs_3, 0);
						tb->connect(pdu_4,0, fs_4, 0);
						tb->connect(pdu_5,0, fs_5, 0);
						tb->connect(pdu_6,0, fs_6, 0);
						tb->connect(pdu_7,0, fs_7, 0);











		//tb->connect(fs_0_0,0,receiver,0);
		//tb->connect(fs_0_1,0,receiver,1);
		//tb->msg_connect(receiver,"C0",zmq_push,"in");
		/*tb->msg_connect(receiver,"C0",control_BCCH_demapper,"bursts");
		tb->msg_connect(control_BCCH_demapper,"bursts",control_channel_decoder_bcch,"bursts");
		tb->msg_connect(receiver,"C0", control_SDCCH_demapper_1,"bursts");
		tb->msg_connect(receiver,"C0",control_SDCCH_demapper_2,"bursts");
		tb->msg_connect(receiver,"C0",control_SDCCH_demapper_3,"bursts");
		tb->msg_connect(receiver,"C0",control_SDCCH_demapper_4,"bursts");
		tb->msg_connect(receiver,"C0",control_SDCCH_demapper_5,"bursts");
		tb->msg_connect(receiver,"C0",control_SDCCH_demapper_6,"bursts");
		tb->msg_connect(receiver,"C0",control_SDCCH_demapper_7,"bursts");*/
		tb->msg_connect(control_SDCCH_demapper_1,"bursts",control_channel_decoder_bcch,"bursts");
		tb->msg_connect(control_SDCCH_demapper_2,"bursts",control_channel_decoder_bcch,"bursts");
		tb->msg_connect(control_SDCCH_demapper_3,"bursts",control_channel_decoder_bcch,"bursts");
		tb->msg_connect(control_SDCCH_demapper_4,"bursts",control_channel_decoder_bcch,"bursts");
		tb->msg_connect(control_SDCCH_demapper_5,"bursts",control_channel_decoder_bcch,"bursts");
		tb->msg_connect(control_SDCCH_demapper_6,"bursts",control_channel_decoder_bcch,"bursts");
		tb->msg_connect(control_SDCCH_demapper_7,"bursts",control_channel_decoder_bcch,"bursts");





#endif
		tb->start();
		while(!stop_rx){}
		tb->stop();
		vipl_printf("info: demod agcch stopped", error_lvl, __FILE__, __LINE__);
	}catch(const std::exception& ex) {
        char msg[100]={0x00};
        sprintf(msg,"error: while creating flow graph %s", ex.what());
        vipl_printf(msg,error_lvl, __FILE__, __LINE__);
    }

}
#endif

static void vipl_scanner_init::demod_agcch(struct gsm_channel_data *channel_data, bool uplink, gr::uhd::usrp_source::sptr uhd_src, int32_t *portno, int32_t num_channel, int32_t num_port){
	std::vector<int> downlink_starts_fn_mod51{0,0,2,2,2,2,6,6,6,6,0,0,12,12,12,12,16,16,16,16,0,0,22,22,22,22,26,26,26,26,0,0,32,32,32,32,36,36,36,36,0,0,42,42,42,42,46,46,46,46,0};
	std::vector<int> downlink_channel_types{0,0,0,0,0,0,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0};
	std::vector<int32_t> downlink_subslots{0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3,4,4,4,4,0,0,5,5,5,5,6,6,6,6,0,0,7,7,7,7,8,8,8,8,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3,4,4,4,4,0,0,5,5,5,5,6,6,6,6,0,0,7,7,7,7,8,8,8,8,0};
	std::vector<int32_t> uplink_starts_fn_mod51{0,0,0,0,0,0,6,6,6,6,10,10,10,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,37,37,37,37,41,41,41,41,0,0,47,47,47,47};
	std::vector<int32_t> uplink_channel_types{2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2};
	std::vector<int32_t> uplink_subslots{0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3,0,0,0,0,0,0,2,2,2,2,3,3,3,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3};

	std::vector<int> downlink_starts_fn_mod51_sdcch{0,0,0,0,4,4,4,4,8,8,8,8,12,12,12,12,16,16,16,16,20,20,20,20,24,24,24,24,28,28,28,28,32,32,32,32,36,36,36,36,40,40,40,40,44,44,44,44,0,0,0};
	std::vector<int> downlink_channel_types_sdcch{8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,136,0,0,0};
	std::vector<int> downlink_subslots_sdcch{0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7,0,0,0};
	std::vector<int> uplink_starts_fn_mod51_sdcch{0,0,0,0,4,4,4,4,8,8,8,8,0,0,0,15,15,15,15,19,19,19,19,23,23,23,23,27,27,27,27,31,31,31,31,35,35,35,35,39,39,39,39,43,43,43,43,47,47,47,47};
	std::vector<int> uplink_channel_types_sdcch{136,136,136,136,136,136,136,136,136,136,136,136,0,0,0,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,136,136,136,136};
	std::vector<int> uplink_subslots_sdcch{1,1,1,1,2,2,2,2,3,3,3,3,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7,0,0,0,0,5,5,5,5,6,6,6,6,7,7,7,7,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7,4,4,4,4};
	std::vector<int32_t> arfcn_list;
	std::vector<std::vector<int32_t>> tseq;
	arfcn_list.push_back(0x01);
	float sample_rate_gsm = 2e6;
	tseq.resize(num_channel);
	for(int32_t i=0;i<num_channel;i++) {
		tseq[i].push_back(/*(int32_t)channel_data[i].gsm_param.tseq*/4);
	}
	if(!uplink){
		gr::top_block_sptr tb = gr::make_top_block("simplex_monitor");
		gr::gsm::msg_to_tag::sptr msg2tag[num_channel];
		gr::gsm::controlled_rotator_cc::sptr controlled_rotator[num_channel];
		gr::gsm::controlled_fractional_resampler_cc::sptr controlled_fractional_resampler[num_channel];
		gr::gsm::receiver::sptr receiver[num_channel];
		gr::gsm::clock_offset_control::sptr clock_control_offset[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_BCCH_demapper[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_1[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_2[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_3[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_4[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_5[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_6[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_7[num_channel];
		gr::gsm::control_channels_decoder::sptr control_channel_decoder_bcch[num_channel];
		gr::filter::freq_xlating_fir_filter_ccf::sptr freq_decim[num_channel];
		//sprintf(zeromq_addr, "tcp://127.0.0.1:%d",(1000+channel_data.arfcn_start));
		gr::zeromq::pub_msg_sink::sptr zmq_push[num_channel];
		int32_t ppm = 0x00;
		int32_t count = 0x00;
		char zeromq_addr[30]={0x00};
		for(int32_t i=0;i<num_channel;i++){
			msg2tag[i] = gr::gsm::msg_to_tag::make();
			controlled_fractional_resampler[i] = gr::gsm::controlled_fractional_resampler_cc::make(0,(sample_rate_gsm/(4*(1625000.0/6.0))));
			control_BCCH_demapper[i]    = gr::gsm::universal_ctrl_chans_demapper::make(0,downlink_starts_fn_mod51,downlink_channel_types,downlink_subslots,uplink_starts_fn_mod51,uplink_channel_types,uplink_subslots);
			control_SDCCH_demapper_1[i] = gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)1,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch);
			control_SDCCH_demapper_2[i] = gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)2,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch);
			control_SDCCH_demapper_3[i] = gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)3,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch);
			control_SDCCH_demapper_4[i] = gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)4,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch);
			control_SDCCH_demapper_5[i] = gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)5,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch);
			control_SDCCH_demapper_6[i] = gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)6,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch);
			control_SDCCH_demapper_7[i] = gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)7,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch);
		}
		for(int32_t i=0;i<num_port;i++) {
			for(int32_t j=0;j<channel_data[i].num_channels;j++) {
				memset(zeromq_addr, 0x00, 30);
				sprintf(zeromq_addr, "tcp://127.0.0.1:%d",(1000+channel_data[i].arfcn_list[j]));
				zmq_push[count] = gr::zeromq::pub_msg_sink::make(zeromq_addr, 1000);
				controlled_rotator[count] = gr::gsm::controlled_rotator_cc::make(ppm/1.0e6*2*M_PI*convert_arfcn_to_freq(channel_data[i].band, channel_data[i].arfcn_list[j])/(4*(1625000.0/6.0)));
				clock_control_offset[count] = gr::gsm::clock_offset_control::make(convert_arfcn_to_freq(channel_data[i].band, channel_data[i].arfcn_list[j]),sample_rate_gsm,4);
				float delta = ((channel_data[i].center_arfcn-convert_arfcn_to_freq(channel_data[i].band, channel_data[i].arfcn_list[j])))*(-1);
				freq_decim[count] = gr::filter::freq_xlating_fir_filter_ccf::make(channel_data[i].sample_rate/sample_rate_gsm,gr::filter::firdes::low_pass(1,channel_data[num_port].sample_rate,125e3,75e3,gr::filter::firdes::WIN_KAISER,6.76),delta,channel_data[i].sample_rate);
				receiver[count] = gr::gsm::receiver::make(4,arfcn_list,tseq[0],false);
				control_channel_decoder_bcch[count] = gr::gsm::control_channels_decoder::make(channel_data[i].arfcn_list[j], false);
				count++;
			}
		}
		count = 0x00;
		for(int32_t i=0;i<num_port;i++) {
			for(int32_t j=0;j<channel_data[i].num_channels;j++){
				tb->connect(uhd_src,i, freq_decim[count],0x00);
				tb->connect(freq_decim[count],0x00, msg2tag[count],0x00);
				tb->connect(msg2tag[count],0x00, controlled_fractional_resampler[count],0x00);
				tb->connect(controlled_fractional_resampler[count],0,controlled_rotator[count],0);
				tb->connect(controlled_rotator[count],0,receiver[count],0);
				tb->msg_connect(receiver[count],"measurements",clock_control_offset[count],"measurements");
				tb->msg_connect(clock_control_offset[count],"ctrl",msg2tag[count],"msg");
				tb->msg_connect(receiver[count],"C0",zmq_push[count],"in");
				tb->msg_connect(receiver[count],"C0",control_BCCH_demapper[count],"bursts");
				tb->msg_connect(control_BCCH_demapper[count],"bursts",control_channel_decoder_bcch[count],"bursts");
				tb->msg_connect(receiver[count],"C0", control_SDCCH_demapper_1[count],"bursts");
				tb->msg_connect(receiver[count],"C0",control_SDCCH_demapper_2[count],"bursts");
				tb->msg_connect(receiver[count],"C0",control_SDCCH_demapper_3[count],"bursts");
				tb->msg_connect(receiver[count],"C0",control_SDCCH_demapper_4[count],"bursts");
				tb->msg_connect(receiver[count],"C0",control_SDCCH_demapper_5[count],"bursts");
				tb->msg_connect(receiver[count],"C0",control_SDCCH_demapper_6[count],"bursts");
				tb->msg_connect(receiver[count],"C0",control_SDCCH_demapper_7[count],"bursts");
				tb->msg_connect(control_SDCCH_demapper_1[count],"bursts",control_channel_decoder_bcch[count],"bursts");
				tb->msg_connect(control_SDCCH_demapper_2[count],"bursts",control_channel_decoder_bcch[count],"bursts");
				tb->msg_connect(control_SDCCH_demapper_3[count],"bursts",control_channel_decoder_bcch[count],"bursts");
				tb->msg_connect(control_SDCCH_demapper_4[count],"bursts",control_channel_decoder_bcch[count],"bursts");
				tb->msg_connect(control_SDCCH_demapper_5[count],"bursts",control_channel_decoder_bcch[count],"bursts");
				tb->msg_connect(control_SDCCH_demapper_6[count],"bursts",control_channel_decoder_bcch[count],"bursts");
				tb->msg_connect(control_SDCCH_demapper_7[count],"bursts",control_channel_decoder_bcch[count],"bursts");
				count++;
			}
		}
		sem_post(&start_streaming_init);
		tb->edge_list();
		tb->start();
		while(!stop_rx);
		tb->stop();
		tb->disconnect_all();
		sleep(1);
	}else{
		gr::top_block_sptr tb = gr::make_top_block("duplex_monitor");
		gr::gsm::msg_to_tag::sptr msg2tag[num_channel*2];
		gr::gsm::controlled_rotator_cc::sptr controlled_rotator[num_channel*2];
		gr::gsm::controlled_fractional_resampler_cc::sptr controlled_fractional_resampler[num_channel*2];
		gr::gsm::receiver::sptr receiver[num_channel];
		gr::gsm::clock_offset_control::sptr clock_control_offset[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_BCCH_demapper[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_1[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_2[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_3[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_4[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_5[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_6[num_channel];
		gr::gsm::universal_ctrl_chans_demapper::sptr control_SDCCH_demapper_7[num_channel];
		gr::gsm::control_channels_decoder::sptr control_channel_decoder_bcch[num_channel];
		gr::filter::freq_xlating_fir_filter_ccf::sptr freq_decim[num_channel*2];
		//sprintf(zeromq_addr, "tcp://127.0.0.1:%d",(1000+channel_data.arfcn_start));
		gr::zeromq::pub_msg_sink::sptr zmq_push[num_channel];
		int32_t ppm = 0x00;
		int32_t count = 0x00;
		char zeromq_addr[30]={0x00};
		for(int32_t i=0;i<num_channel;i++){
			control_BCCH_demapper[i]    = gr::gsm::universal_ctrl_chans_demapper::make(0,downlink_starts_fn_mod51,downlink_channel_types,downlink_subslots,uplink_starts_fn_mod51,uplink_channel_types,uplink_subslots);
			control_SDCCH_demapper_1[i] = gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)1,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch);
			control_SDCCH_demapper_2[i] = gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)2,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch);
			control_SDCCH_demapper_3[i] = gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)3,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch);
			control_SDCCH_demapper_4[i] = gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)4,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch);
			control_SDCCH_demapper_5[i] = gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)5,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch);
			control_SDCCH_demapper_6[i] = gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)6,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch);
			control_SDCCH_demapper_7[i] = gr::gsm::universal_ctrl_chans_demapper::make((uint32_t)7,downlink_starts_fn_mod51_sdcch,downlink_channel_types_sdcch,downlink_subslots_sdcch,uplink_starts_fn_mod51_sdcch,uplink_channel_types_sdcch,uplink_subslots_sdcch);
		}
		count = 0x00;
		for(int32_t i=0;i<num_port;i++) {
			for(int32_t j=0;j<channel_data[i].num_channels;j++) {
				float delta = ((channel_data[i].center_arfcn-convert_arfcn_to_freq(channel_data[i].band, channel_data[i].arfcn_list[j])))*(-1);
				freq_decim[count] = gr::filter::freq_xlating_fir_filter_ccf::make(channel_data[i].sample_rate/sample_rate_gsm,gr::filter::firdes::low_pass(1,channel_data[num_port].sample_rate,125e3,75e3,gr::filter::firdes::WIN_KAISER,6.76),delta,channel_data[i].sample_rate);
				freq_decim[count+1] = gr::filter::freq_xlating_fir_filter_ccf::make(channel_data[i].sample_rate/sample_rate_gsm,gr::filter::firdes::low_pass(1,channel_data[num_port].sample_rate,125e3,75e3,gr::filter::firdes::WIN_KAISER,6.76),delta,channel_data[i].sample_rate);
				count+=2;
			}
		}
		for(int32_t i=0;i<num_port;i++) {
			for(int32_t j=0;j<channel_data[i].num_channels*2;j++) {
				msg2tag[j] = gr::gsm::msg_to_tag::make();
				controlled_fractional_resampler[j] = gr::gsm::controlled_fractional_resampler_cc::make(0,(sample_rate_gsm/(4*(1625000.0/6.0))));
				controlled_rotator[j] = gr::gsm::controlled_rotator_cc::make(ppm/1.0e6*2*M_PI*convert_arfcn_to_freq(channel_data[i].band, channel_data[i].arfcn_list[j])/(4*(1625000.0/6.0)));
			}
		}
		for(int32_t i=0;i<num_port;i++) {
			for(int32_t j=0;j<channel_data[i].num_channels;j++) {
				memset(zeromq_addr, 0x00, 30);
				sprintf(zeromq_addr, "tcp://127.0.0.1:%d",(1000+channel_data[i].arfcn_list[j]));
				zmq_push[j] = gr::zeromq::pub_msg_sink::make(zeromq_addr, 1000);
				clock_control_offset[j] = gr::gsm::clock_offset_control::make(convert_arfcn_to_freq(channel_data[i].band, channel_data[i].arfcn_list[j]),sample_rate_gsm,4);
				receiver[j] = gr::gsm::receiver::make(4,arfcn_list,tseq[0],true);
				control_channel_decoder_bcch[j] = gr::gsm::control_channels_decoder::make(channel_data[i].arfcn_list[j], false);
			}
		}
		count = 0x00;
		int32_t usrp_count = 0x00;
		for(int32_t i=0;i<num_port;i++) {
			for(int32_t j=0;j<channel_data[i].num_channels*2;j+=2){
				tb->connect(uhd_src,usrp_count, freq_decim[j],0x00);
				tb->connect(uhd_src,usrp_count+1, freq_decim[j+1],0x00);
			}
			usrp_count+=2;
			for(int32_t j=0;j<channel_data[i].num_channels*2;j+=2){
				tb->connect(freq_decim[j],0x00, msg2tag[j],0x00);
				tb->connect(msg2tag[j], 0x00, controlled_fractional_resampler[j],0x00);
				tb->connect(controlled_fractional_resampler[j], 0x00, controlled_rotator[j],0x00);
				tb->connect(freq_decim[j+1],0x00, msg2tag[j+1],0x00);
				tb->connect(msg2tag[j+1], 0x00, controlled_fractional_resampler[j+1],0x00);
				tb->connect(controlled_fractional_resampler[j+1], 0x00, controlled_rotator[j+1],0x00);
			}
			gr::zeromq::pub_sink::sptr pubsink = gr::zeromq::pub_sink::make(sizeof(std::complex<float>), 1, "tcp://192.168.2.113:5301", 100);
			count =0x00;
			for(int32_t j=0;j<channel_data[i].num_channels;j++){
				tb->connect(controlled_rotator[count],0x00, receiver[j], 0);
				tb->connect(controlled_rotator[count+1], 0x00, receiver[j],1);
				tb->connect(controlled_rotator[count+1],0,pubsink,0);
				count+=2;
			}
			count =0x00;
			for(int32_t j=0;j<channel_data[i].num_channels;j++){
				tb->msg_connect(clock_control_offset[j],"ctrl", msg2tag[count], "msg");
				tb->msg_connect(clock_control_offset[j],"ctrl", msg2tag[count+1],"msg");
				count+=2;
			}
			count =0x00;
			for(int32_t j=0;j<channel_data[i].num_channels;j++){
				tb->msg_connect(receiver[count],"measurements",clock_control_offset[count],"measurements");
				tb->msg_connect(receiver[count],"C0",zmq_push[count],"in");
#if 1
				tb->msg_connect(receiver[count],"C0",control_BCCH_demapper[count],"bursts");
				tb->msg_connect(control_BCCH_demapper[count],"bursts",control_channel_decoder_bcch[count],"bursts");
				tb->msg_connect(receiver[count],"C0", control_SDCCH_demapper_1[count],"bursts");
				tb->msg_connect(receiver[count],"C0",control_SDCCH_demapper_2[count],"bursts");
				tb->msg_connect(receiver[count],"C0",control_SDCCH_demapper_3[count],"bursts");
				tb->msg_connect(receiver[count],"C0",control_SDCCH_demapper_4[count],"bursts");
				tb->msg_connect(receiver[count],"C0",control_SDCCH_demapper_5[count],"bursts");
				tb->msg_connect(receiver[count],"C0",control_SDCCH_demapper_6[count],"bursts");
				tb->msg_connect(receiver[count],"C0",control_SDCCH_demapper_7[count],"bursts");
				tb->msg_connect(control_SDCCH_demapper_1[count], "bursts", control_channel_decoder_bcch[count],"bursts");
				tb->msg_connect(control_SDCCH_demapper_2[count], "bursts", control_channel_decoder_bcch[count],"bursts");
				tb->msg_connect(control_SDCCH_demapper_3[count], "bursts", control_channel_decoder_bcch[count],"bursts");
				tb->msg_connect(control_SDCCH_demapper_4[count], "bursts", control_channel_decoder_bcch[count],"bursts");
				tb->msg_connect(control_SDCCH_demapper_5[count], "bursts", control_channel_decoder_bcch[count],"bursts");
				tb->msg_connect(control_SDCCH_demapper_6[count], "bursts", control_channel_decoder_bcch[count],"bursts");
				tb->msg_connect(control_SDCCH_demapper_7[count], "bursts", control_channel_decoder_bcch[count],"bursts");
#endif
				count++;
			}
		}

		sem_post(&start_streaming_init);
		//std::cout<<tb->edge_list()<<std::endl;
		tb->start();
		while(!stop_rx);
		tb->stop();
		tb->disconnect_all();
		sleep(1);
	}
}

static void vipl_scanner_init::start_filter_iq_sharing(struct gsm_channel_data channel_data, cbuffercf *cb) {
	uint32_t num_samples_read = 0x00;
	std::complex<float> *buffer_read;
	int32_t channel_count_var = 0;
	boost::thread_group tid_group;
	channel_count_var  = count_channel(channel_data.band, channel_data.arfcn_list, channel_data.num_channels);
	char fifo_name[channel_count_var][50]={0x00};
	int32_t cb_sub_channel[channel_count_var];
	uint32_t numSample2Read = 50e6;
	for(int32_t i=0;i<channel_count_var;i++) {
		channel_data.arfcn_start = channel_data.arfcn_list[i];
		struct stat st;
		sprintf(fifo_name[i],"/tmp/pipe_%d.bin",channel_data.arfcn_list[i]);
		if(stat(fifo_name[i],&st)){
			if(mkfifo(fifo_name[i], 0666)==-1){
				vipl_printf("error: unable to create FIFO", error_lvl, __FILE__, __LINE__);
		    }else{
		    	vipl_printf("info: FIFO already present not creating!!", error_lvl, __FILE__, __LINE__);
		    }
		}
		//tid_group.add_thread(new boost::thread(switch_channel_matrix, channel_data, channel_data.arfcn_start+1000, fifo_name[i]));
		cb_sub_channel[i] = open(fifo_name[i],O_WRONLY);
		if(cb_sub_channel[i]<=0x00)
			continue;
		if(!(fcntl(cb_sub_channel[i], F_SETPIPE_SZ, numSample2Read * 2)))
			vipl_printf("error: unable to set size of FIFO", error_lvl, __FILE__, __LINE__);
	}
	bool done = false;
	while(!stop_rx){
		label:
		cbuffercf_read(*cb, (uint32_t)channel_data.samples_per_burst, &buffer_read, (uint32_t *)&num_samples_read);
		cbuffercf_release(*cb, (uint32_t)num_samples_read);
		if(num_samples_read!=(uint32_t)channel_data.samples_per_burst){
			usleep(1000);
			goto label;
		}
		if(error_lvl==3){
			fprintf(stderr,"\n======================================================\n");
			fprintf(stderr,"Number of samples read from circular buffer %d", num_samples_read);
			fprintf(stderr,"\n======================================================\n");
		}
		if(!done){
			sem_post(&start_read);
			for(int32_t i=0;i<channel_count_var;i++) {
				channel_data.arfcn_start = channel_data.arfcn_list[i];
				//tid_group.add_thread(new boost::thread(demod_agcch, channel_data, channel_data.arfcn_start+1000));
				usleep(100);
			}
			done = true;
		}
		for(int32_t i=0;i<channel_count_var;i++) {
			write(cb_sub_channel[i], buffer_read, sizeof(std::complex<float>)*num_samples_read);
		}
	}
	vipl_printf("info: IQ sharing stopped", error_lvl, __FILE__, __LINE__);
}

#if 0
static void vipl_scanner_init::start_filter_iq_sharing(struct gsm_channel_data channel_data, cbuffercf *cb) {
	double num_samples_read = 0x00;
    std::complex<float> *buffer_read[4];
    int32_t fd[channel_data.num_max_chan] = {0x00}, max_chan_dsp1 = 0x00, max_chan_dsp2 = 0x00;
    boost::thread_group tid_group;
    char fifo_name[channel_data.num_max_chan][100]={0x00};
    for(int32_t i = 0; i<channel_data.num_max_chan; i++) {
    	struct stat st;
    	sprintf(fifo_name[i],"/tmp/pipe_%d.bin",channel_data.arfcn_list[i]);
    	if(stat(fifo_name[i],&st)){
    		if(mkfifo(fifo_name[i], 0666)==-1)
    			vipl_printf("error: unable to create FIFO", error_lvl, __FILE__, __LINE__);
    	}else{
    		vipl_printf("info: FIFO already present not creating!!", error_lvl, __FILE__, __LINE__);
    	}
    }
    for(int32_t i = 0; i<channel_data.num_channels; i++) {
    	fd[i] = open(fifo_name[i], O_WRONLY);
    	if(fd[i]<= 0x00){
    		vipl_printf("error: unable to open file descriptor", error_lvl, __FILE__, __LINE__);
    		continue;
    	}
    	if(error_lvl==3){
    		char msg[100]={0x00};
    		sprintf(msg,"debug: FIFO %s successfully created and opened", fifo_name[i]);
    		vipl_printf(msg, error_lvl, __FILE__, __LINE__);
    	}
    	channel_data.arfcn_start = channel_data.arfcn_list[i];
    	tid_group.add_thread(new boost::thread(switch_channel_matrix, channel_data, fifo_name[i]));
    }

    for(int32_t i=0;i<channel_data.num_channels;i++) {

    }
    while(true) {
    	memset(buffer_read, 0x00, sizeof(std::complex<float>));
    	for(int32_t i=0;i<4;i++) {
    		cbuffercf_read(cb[i], (uint32_t)channel_data.samples_per_burst, &buffer_read[i], (uint32_t *)&num_samples_read);
    		cbuffercf_release(cb[i], (uint32_t)num_samples_read);
    		if(channel_data.samples_per_burst!=num_samples_read) {
    		  char msg[100] = {0x00};
    		  sprintf(msg,"error: improper samples read %d, should have read %d", num_samples_read, (uint32_t)channel_data.samples_per_burst);
    		  vipl_printf(msg, error_lvl, __FILE__, __LINE__);
    		}
    	}

    	for(int32_t i = 0; i<channel_data.num_max_chan; i++) {
    		if((write(fd[i], buffer_read[i], sizeof(std::complex<float>)*num_samples_read))!=(sizeof(std::complex<float>)*num_samples_read))
    			vipl_printf("error: improper write to FIFO", error_lvl, __FILE__, __LINE__);
    	}
    }
}
#endif

struct initialization_status{
	bool status;
	vipl_rf_interface vipl_rf_interface_obj;
	gr::uhd::usrp_source::sptr uhd_src;
	char mboard_ip[16];
	//struct RFNOC_Config rfnoc_config;
	struct initialization_status *next;
}*head,*cont;

bool d_write_success = false;
int8_t both_bands_done = 0;
int32_t counter = 0x00;

static void vipl_scanner_init::start_ntwrk_scan_process(struct gsm_channel_data channel_data, char *file_name, gr::top_block_sptr tb) {
	float cuttoff = 495e3;
	std::vector<int32_t> arfcn_list;
	std::vector<int32_t> tseq;
	arfcn_list.push_back(channel_data.arfcn_start);
	tseq.push_back(0);
	std::vector<int> downlink_starts_fn_mod51{0,0,2,2,2,2,6,6,6,6,0,0,12,12,12,12,16,16,16,16,0,0,22,22,22,22,26,26,26,26,0,0,32,32,32,32,36,36,36,36,0,0,42,42,42,42,46,46,46,46,0};
	//std::vector<int> downlink_channel_types{0,0,1,1,1,1,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0};
	std::vector<int32_t> downlink_channel_types{0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	std::vector<int32_t> downlink_subslots{0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3,4,4,4,4,0,0,5,5,5,5,6,6,6,6,0,0,7,7,7,7,8,8,8,8,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3,4,4,4,4,0,0,5,5,5,5,6,6,6,6,0,0,7,7,7,7,8,8,8,8,0};
	std::vector<int32_t> uplink_starts_fn_mod51{0,0,0,0,0,0,6,6,6,6,10,10,10,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,37,37,37,37,41,41,41,41,0,0,47,47,47,47};
	std::vector<int32_t> uplink_channel_types{2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2};
	std::vector<int32_t> uplink_subslots{0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3,0,0,0,0,0,0,2,2,2,2,3,3,3,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3};
	//gr::blocks::file_sink::sptr fs =  gr::blocks::file_sink::make(sizeof(std::complex<float>), "/tmp/test_output.bin", false);
	d_write_success = false;
	char dst_path[200]={0x00};
	sprintf(dst_path,"%s/%d_gsmBCCH.bin",ntwrkscan,channel_data.arfcn_start);
	FILE *fp=fopen(dst_path,"w+");
	for(int32_t i=channel_data.arfcn_start;i<channel_data.num_channels;i++) {
		//gr::filter::freq_xlating_fir_filter_ccf::sptr freq_decim = gr::filter::freq_xlating_fir_filter_ccf::make(channel_data.sample_rate/500000,gr::filter::firdes::low_pass(1.0, channel_data.sample_rate, cuttoff, 5e3, gr::filter::firdes::WIN_KAISER,6.76),(-(channel_data.center_arfcn-convert_arfcn_to_freq(channel_data.band,i))), channel_data.sample_rate);
		gr::blocks::throttle::sptr throttle = gr::blocks::throttle::make(sizeof(std::complex<float>), channel_data.sample_rate, true);
		gr::gsm::msg_to_tag::sptr msg2tag_0 = gr::gsm::msg_to_tag::make();
		gr::gsm::controlled_rotator_cc::sptr controlled_rotator_0 = gr::gsm::controlled_rotator_cc::make(0);
		gr::gsm::controlled_fractional_resampler_cc::sptr controlled_fractional_resampler_0 = gr::gsm::controlled_fractional_resampler_cc::make(0,(2e6/(4*(1625000.0/6.0))));
		gr::gsm::receiver::sptr receiver = gr::gsm::receiver::make(4,arfcn_list,tseq,false);
		gr::gsm::universal_ctrl_chans_demapper::sptr control_BCCH_demapper(gr::gsm::universal_ctrl_chans_demapper::make(0,downlink_starts_fn_mod51,downlink_channel_types,downlink_subslots,uplink_starts_fn_mod51,uplink_channel_types,uplink_subslots));
		gr::filter::firdes lowpass_taps;
		gr::filter::fir_filter_ccf::sptr lowpassFilter = gr::filter::fir_filter_ccf::make(1,lowpass_taps.low_pass(1,4*(1625000.0/6.0),125000,5000,lowpass_taps.WIN_KAISER,6.76));
		gr::gsm::control_channels_decoder::sptr control_channel_decoder = gr::gsm::control_channels_decoder::make(channel_data.arfcn_start, true);
		gr::gsm::clock_offset_control::sptr clock_control_offset= gr::gsm::clock_offset_control::make(convert_arfcn_to_freq(channel_data.band,i),2e6,4);
		gr::blocks::file_source::sptr fd = gr::blocks::file_source::make(sizeof(std::complex<float>), file_name, false);
		tb->connect(fd, 0, msg2tag_0,0);
		tb->connect(msg2tag_0,0,controlled_fractional_resampler_0,0);
		tb->connect(controlled_fractional_resampler_0,0,controlled_rotator_0,0);
		tb->connect(controlled_rotator_0,0,lowpassFilter,0);
		tb->connect(lowpassFilter,0,receiver,0);
		tb->msg_connect(receiver,"measurements",clock_control_offset,"measurements");
		tb->msg_connect(clock_control_offset,"ctrl",msg2tag_0,"msg");
		tb->msg_connect(receiver,"C0",control_BCCH_demapper,"bursts");
		tb->msg_connect(control_BCCH_demapper,"bursts",control_channel_decoder,"bursts");
		tb->start(); // TODO prachi to check with various iterations
		tb->wait();
		tb->stop();
	}
	if(d_write_success==false)
		fwrite(&rftap,sizeof(rftap),1,fp);
	fclose(fp);
}

static void vipl_scanner_init::ntwrk_scan_start_fft(struct gsm_channel_data channel_data, tcp_server *tcpServer_gui, gr::uhd::usrp_source::sptr uhd_src) {
	ntwrk_scan_lock.lock();
	boost::thread_group tid_group;
	struct gsm_channel_data channel_data_local;
	memset(&channel_data_local, 0x00, sizeof(channel_data_local));
	memcpy(&channel_data_local,&channel_data, sizeof(channel_data));
	clock_t begin_time = std::clock();
	std::vector<int32_t> arfcn_list;
	std::vector<int32_t> tseq;
	arfcn_list.push_back(channel_data.arfcn_start);
	tseq.push_back(0);
	std::vector<int> downlink_starts_fn_mod51{0,0,2,2,2,2,6,6,6,6,0,0,12,12,12,12,16,16,16,16,0,0,22,22,22,22,26,26,26,26,0,0,32,32,32,32,36,36,36,36,0,0,42,42,42,42,46,46,46,46,0};
	//std::vector<int> downlink_channel_types{0,0,1,1,1,1,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2,2,2,2,2,0};
	std::vector<int32_t> downlink_channel_types{0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	std::vector<int32_t> downlink_subslots{0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3,4,4,4,4,0,0,5,5,5,5,6,6,6,6,0,0,7,7,7,7,8,8,8,8,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3,4,4,4,4,0,0,5,5,5,5,6,6,6,6,0,0,7,7,7,7,8,8,8,8,0};
	std::vector<int32_t> uplink_starts_fn_mod51{0,0,0,0,0,0,6,6,6,6,10,10,10,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,37,37,37,37,41,41,41,41,0,0,47,47,47,47};
	std::vector<int32_t> uplink_channel_types{2,2,2,2,0,0,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,2,2,2,2,2,2,2,0,0,2,2,2,2};
	std::vector<int32_t> uplink_subslots{0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3,0,0,0,0,0,0,2,2,2,2,3,3,3,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,0,0,3,3,3,3};
	//gr::blocks::file_sink::sptr fs =  gr::blocks::file_sink::make(sizeof(std::complex<float>), "/tmp/test_output.bin", false);
	//gr::filter::freq_xlating_fir_filter_ccf::sptr freq_decim = gr::filter::freq_xlating_fir_filter_ccf::make(channel_data.sample_rate/500000,gr::filter::firdes::low_pass(1.0, channel_data.sample_rate, cuttoff, 5e3, gr::filter::firdes::WIN_KAISER,6.76),(-(channel_data.center_arfcn-convert_arfcn_to_freq(channel_data.band,i))), channel_data.sample_rate);
	gr::blocks::throttle::sptr throttle = gr::blocks::throttle::make(sizeof(std::complex<float>), channel_data.sample_rate, true);
	gr::gsm::msg_to_tag::sptr msg2tag_0 = gr::gsm::msg_to_tag::make();
	gr::gsm::controlled_rotator_cc::sptr controlled_rotator_0 = gr::gsm::controlled_rotator_cc::make(0);
	gr::gsm::controlled_fractional_resampler_cc::sptr controlled_fractional_resampler_0 = gr::gsm::controlled_fractional_resampler_cc::make(0,(2e6/(4*(1625000.0/6.0))));

	gr::gsm::msg_to_tag::sptr msg2tag_1 = gr::gsm::msg_to_tag::make();
	gr::gsm::controlled_rotator_cc::sptr controlled_rotator_1 = gr::gsm::controlled_rotator_cc::make(0);
	gr::gsm::controlled_fractional_resampler_cc::sptr controlled_fractional_resampler_1 = gr::gsm::controlled_fractional_resampler_cc::make(0,(2e6/(4*(1625000.0/6.0))));

	gr::gsm::msg_to_tag::sptr msg2tag_2 = gr::gsm::msg_to_tag::make();
	gr::gsm::controlled_rotator_cc::sptr controlled_rotator_2 = gr::gsm::controlled_rotator_cc::make(0);
	gr::gsm::controlled_fractional_resampler_cc::sptr controlled_fractional_resampler_2 = gr::gsm::controlled_fractional_resampler_cc::make(0,(2e6/(4*(1625000.0/6.0))));

	gr::gsm::msg_to_tag::sptr msg2tag_3 = gr::gsm::msg_to_tag::make();
	gr::gsm::controlled_rotator_cc::sptr controlled_rotator_3 = gr::gsm::controlled_rotator_cc::make(0);
	gr::gsm::controlled_fractional_resampler_cc::sptr controlled_fractional_resampler_3 = gr::gsm::controlled_fractional_resampler_cc::make(0,(2e6/(4*(1625000.0/6.0))));


	gr::gsm::receiver::sptr receiver_0 = gr::gsm::receiver::make(4,arfcn_list,tseq,false);
	gr::gsm::receiver::sptr receiver_1 = gr::gsm::receiver::make(4,arfcn_list,tseq,false);
	gr::gsm::receiver::sptr receiver_2 = gr::gsm::receiver::make(4,arfcn_list,tseq,false);
	gr::gsm::receiver::sptr receiver_3 = gr::gsm::receiver::make(4,arfcn_list,tseq,false);

	gr::gsm::universal_ctrl_chans_demapper::sptr control_BCCH_demapper_0(gr::gsm::universal_ctrl_chans_demapper::make(0,downlink_starts_fn_mod51,downlink_channel_types,downlink_subslots,uplink_starts_fn_mod51,uplink_channel_types,uplink_subslots));
	gr::gsm::universal_ctrl_chans_demapper::sptr control_BCCH_demapper_1(gr::gsm::universal_ctrl_chans_demapper::make(0,downlink_starts_fn_mod51,downlink_channel_types,downlink_subslots,uplink_starts_fn_mod51,uplink_channel_types,uplink_subslots));
	gr::gsm::universal_ctrl_chans_demapper::sptr control_BCCH_demapper_2(gr::gsm::universal_ctrl_chans_demapper::make(0,downlink_starts_fn_mod51,downlink_channel_types,downlink_subslots,uplink_starts_fn_mod51,uplink_channel_types,uplink_subslots));
	gr::gsm::universal_ctrl_chans_demapper::sptr control_BCCH_demapper_3(gr::gsm::universal_ctrl_chans_demapper::make(0,downlink_starts_fn_mod51,downlink_channel_types,downlink_subslots,uplink_starts_fn_mod51,uplink_channel_types,uplink_subslots));

	gr::filter::firdes lowpass_taps;
	gr::filter::fir_filter_ccf::sptr lowpassFilter_0 = gr::filter::fir_filter_ccf::make(1,lowpass_taps.low_pass(1,4*(1625000.0/6.0),125000,5000,lowpass_taps.WIN_KAISER,6.76));
	gr::filter::fir_filter_ccf::sptr lowpassFilter_1 = gr::filter::fir_filter_ccf::make(1,lowpass_taps.low_pass(1,4*(1625000.0/6.0),125000,5000,lowpass_taps.WIN_KAISER,6.76));
	gr::filter::fir_filter_ccf::sptr lowpassFilter_2 = gr::filter::fir_filter_ccf::make(1,lowpass_taps.low_pass(1,4*(1625000.0/6.0),125000,5000,lowpass_taps.WIN_KAISER,6.76));
	gr::filter::fir_filter_ccf::sptr lowpassFilter_3 = gr::filter::fir_filter_ccf::make(1,lowpass_taps.low_pass(1,4*(1625000.0/6.0),125000,5000,lowpass_taps.WIN_KAISER,6.76));

	gr::gsm::control_channels_decoder::sptr control_channel_decoder_0 = gr::gsm::control_channels_decoder::make(channel_data.arfcn_start, true);
	gr::gsm::control_channels_decoder::sptr control_channel_decoder_1 = gr::gsm::control_channels_decoder::make(channel_data.arfcn_start+1, true);
	gr::gsm::control_channels_decoder::sptr control_channel_decoder_2 = gr::gsm::control_channels_decoder::make(channel_data.arfcn_start+2, true);
	gr::gsm::control_channels_decoder::sptr control_channel_decoder_3 = gr::gsm::control_channels_decoder::make(channel_data.arfcn_start+3, true);

	gr::gsm::clock_offset_control::sptr clock_control_offset_0 = gr::gsm::clock_offset_control::make(convert_arfcn_to_freq(channel_data.band,channel_data.arfcn_start),2e6,4);
	gr::gsm::clock_offset_control::sptr clock_control_offset_1 = gr::gsm::clock_offset_control::make(convert_arfcn_to_freq(channel_data.band,channel_data.arfcn_start+1),2e6,4);
	gr::gsm::clock_offset_control::sptr clock_control_offset_2 = gr::gsm::clock_offset_control::make(convert_arfcn_to_freq(channel_data.band,channel_data.arfcn_start+2),2e6,4);
	gr::gsm::clock_offset_control::sptr clock_control_offset_3 = gr::gsm::clock_offset_control::make(convert_arfcn_to_freq(channel_data.band,channel_data.arfcn_start+3),2e6,4);
	gr::top_block_sptr tb = gr::make_top_block("bcchDemod");
	tb->connect(uhd_src, 0, msg2tag_0,0);
	tb->connect(msg2tag_0,0,controlled_fractional_resampler_0,0);
	tb->connect(controlled_fractional_resampler_0,0,controlled_rotator_0,0);
	tb->connect(controlled_rotator_0,0,lowpassFilter_0,0);
	tb->connect(lowpassFilter_0,0,receiver_0,0);
	tb->msg_connect(receiver_0,"measurements",clock_control_offset_0,"measurements");
	tb->msg_connect(clock_control_offset_0,"ctrl",msg2tag_0,"msg");
	tb->msg_connect(receiver_0,"C0",control_BCCH_demapper_0,"bursts");
	tb->msg_connect(control_BCCH_demapper_0,"bursts",control_channel_decoder_0,"bursts");

	tb->connect(uhd_src, 1, msg2tag_1,0);
	tb->connect(msg2tag_1,0,controlled_fractional_resampler_1,0);
	tb->connect(controlled_fractional_resampler_1,0,controlled_rotator_1,0);
	tb->connect(controlled_rotator_1,0,lowpassFilter_1,0);
	tb->connect(lowpassFilter_1,0,receiver_1,0);
	tb->msg_connect(receiver_1,"measurements",clock_control_offset_1,"measurements");
	tb->msg_connect(clock_control_offset_1,"ctrl",msg2tag_1,"msg");
	tb->msg_connect(receiver_1,"C0",control_BCCH_demapper_1,"bursts");
	tb->msg_connect(control_BCCH_demapper_1,"bursts",control_channel_decoder_1,"bursts");

	tb->connect(uhd_src, 2, msg2tag_2,0);
	tb->connect(msg2tag_2,0,controlled_fractional_resampler_2,0);
	tb->connect(controlled_fractional_resampler_2,0,controlled_rotator_2,0);
	tb->connect(controlled_rotator_2,0,lowpassFilter_2,0);
	tb->connect(lowpassFilter_2,0,receiver_2,0);
	tb->msg_connect(receiver_2,"measurements",clock_control_offset_2,"measurements");
	tb->msg_connect(clock_control_offset_2,"ctrl",msg2tag_2,"msg");
	tb->msg_connect(receiver_2,"C0",control_BCCH_demapper_2,"bursts");
	tb->msg_connect(control_BCCH_demapper_2,"bursts",control_channel_decoder_2,"bursts");

	tb->connect(uhd_src, 3, msg2tag_3,0);
	tb->connect(msg2tag_3,0,controlled_fractional_resampler_3,0);
	tb->connect(controlled_fractional_resampler_3,0,controlled_rotator_3,0);
	tb->connect(controlled_rotator_3,0,lowpassFilter_3,0);
	tb->connect(lowpassFilter_3,0,receiver_3,0);
	tb->msg_connect(receiver_3,"measurements",clock_control_offset_3,"measurements");
	tb->msg_connect(clock_control_offset_3,"ctrl",msg2tag_3,"msg");
	tb->msg_connect(receiver_3,"C0",control_BCCH_demapper_3,"bursts");
	tb->msg_connect(control_BCCH_demapper_3,"bursts",control_channel_decoder_3,"bursts");
	uhd_src->set_samp_rate(channel_data.sample_rate);
	{
		char msg[100]={0x00};
		sprintf(msg,"info: sample rate set to %fmsps",uhd_src->get_samp_rate());
		vipl_printf(msg, error_lvl, __FILE__, __LINE__);
	}
	int32_t i = channel_data.arfcn_start;
	while(i<(channel_data.arfcn_start+channel_data.num_channels)){
		if(i!=channel_data.arfcn_start){
			control_channel_decoder_0->set_arfcn(i);
			control_channel_decoder_1->set_arfcn(i+1);
			control_channel_decoder_2->set_arfcn(i+2);
			control_channel_decoder_3->set_arfcn(i+3);
		}
		clock_control_offset_0->set_fc(convert_arfcn_to_freq(channel_data.band,i));
		clock_control_offset_1->set_fc(convert_arfcn_to_freq(channel_data.band,i+1));
		clock_control_offset_2->set_fc(convert_arfcn_to_freq(channel_data.band,i+2));
		clock_control_offset_3->set_fc(convert_arfcn_to_freq(channel_data.band,i+3));
		if(i==125)
			i=975;
		uhd_src->set_center_freq(convert_arfcn_to_freq(channel_data.band,i),0);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: Frequency set to %0.9fMHz rf chain 0",uhd_src->get_center_freq(0));
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd_src->set_gain(DEFAULT_GAIN_DWNLINK, 0x00);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: gain set to %f rf chain 0",uhd_src->get_gain(0));
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd_src->set_bandwidth(channel_data.sample_rate,0x00);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: bandwidth set to %fMHz rf chain 0",uhd_src->get_gain(0));
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd_src->set_antenna("RX1",0x00);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: Antenna used with rf chain 0 is %s",uhd_src->get_antenna(0x00).c_str());
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd_src->set_center_freq(convert_arfcn_to_freq(channel_data.band,i+1),0x01);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: Frequency set to %0.9fMHz rf chain 1",uhd_src->get_center_freq(0x01));
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd_src->set_gain(DEFAULT_GAIN_DWNLINK, 0x01);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: gain set to %f rf chain 1",uhd_src->get_gain(0x01));
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd_src->set_bandwidth(channel_data.sample_rate,0x01);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: bandwidth set to %fMHz rf chain 1",uhd_src->get_bandwidth(0x01));
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd_src->set_antenna("RX2",0x01);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: Antenna used with rf chain 1 is %s",uhd_src->get_antenna(0x01).c_str());
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd_src->set_center_freq(convert_arfcn_to_freq(channel_data.band,i+2),0x02);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: Frequency set to %0.9fMHz rf chain 2",uhd_src->get_center_freq(0x02));
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd_src->set_gain(DEFAULT_GAIN_DWNLINK, 0x02);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: gain set to %f rf chain 2",uhd_src->get_gain(0x02));
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd_src->set_bandwidth(channel_data.sample_rate,0x02);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: bandwidth set to %fMHz rf chain 2",uhd_src->get_bandwidth(0x02));
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd_src->set_antenna("RX1",0x00);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: Antenna used with rf chain 2 is %s",uhd_src->get_antenna(0x02).c_str());
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd_src->set_center_freq(convert_arfcn_to_freq(channel_data.band,i+3),0x03);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: Frequency set to %0.9fMHz rf chain 3",uhd_src->get_center_freq(0x03));
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd_src->set_gain(DEFAULT_GAIN_DWNLINK, 0x03);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: gain set to %f rf chain 3",uhd_src->get_gain(0x03));
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd_src->set_bandwidth(channel_data.sample_rate,0x03);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: bandwidth set to %fMHz rf chain 3",uhd_src->get_bandwidth(0x03));
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd_src->set_antenna("RX2",0x00);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: Antenna used with rf chain 3 is %s",uhd_src->get_antenna(0x03).c_str());
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		sleep(1);
		i+=4;
		tb->start();
		sleep(2);
		tb->stop();
	}
	uhd_src->clear_finished();
	tb->disconnect_all();
	sleep(1);
	if(error_lvl==3)
		std::cout<<"time elapsed "<<(float(std::clock()-begin_time)/CLOCKS_PER_SEC)<<"secs"<<std::endl;
	vipl_printf("info: Network scan stopped", error_lvl, __FILE__, __LINE__);
	both_bands_done+=1;
	if(both_bands_done==counter) {
		struct RESPONSE_TO_GUI write_to_gui;
		memset(&write_to_gui, 0x00, sizeof(write_to_gui));
		write_to_gui.perform_network_scan = true;
		write_to_gui.already_tuned = true;
		uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
		memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
		memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
		tcpServer_gui->write_action(buffer);
		rx_started = false;
	}
	ntwrk_scan_lock.unlock();
}

void vipl_scanner_init::group_by(enum band_details band, int16_t *arfcn_list, int16_t arfcn_list_by_group[][8], int32_t total_arfcn_count, int32_t *max_rows){
	int32_t counter1 = 0x00, counter2 = 0x00, counter3 = 0x00, counter4 = 0x00, counter5 = 0x00, counter6 = 0x00, counter7 = 0x00, counter8 = 0x00;
	switch(band){
		case gsm_450: 	memcpy(arfcn_list_by_group[0],arfcn_list, sizeof(int16_t)*total_arfcn_count);
		                *max_rows = 1;
					  	break;
		case gsm_800:   counter = 0;
						for(int32_t arfcn = 0;arfcn<total_arfcn_count;arfcn++){
								   if(arfcn_list[arfcn]>=0 && arfcn_list[arfcn]<=49){
									   arfcn_list_by_group[0][counter1++] = arfcn_list[arfcn];
				                   }
				                   else if(arfcn_list[arfcn]>=50 && arfcn_list[arfcn]<=99){
				                	   arfcn_list_by_group[1][counter2++] = arfcn_list[arfcn];
				                   }
				                   else if(arfcn_list[arfcn]>=100 && arfcn_list[arfcn]<=124){
				                	   arfcn_list_by_group[2][counter3++] = arfcn_list[arfcn];
				                   }
				                   else if(arfcn_list[arfcn]>=975 && arfcn_list[arfcn]<=1023){
				                       arfcn_list_by_group[3][counter4++] = arfcn_list[arfcn];
				                   }
						}
                       *max_rows = 4;
		   		   	   break;

		case gsm_850: counter = 0x00;
		              for(int32_t arfcn = 0;arfcn<total_arfcn_count;arfcn++){
		            	 if(arfcn_list[arfcn]>=128 && arfcn_list[arfcn]<=177){
						    arfcn_list_by_group[0][counter1++] = arfcn_list[arfcn];
						 }
						 else if(arfcn_list[arfcn]>=178 && arfcn_list[arfcn]<=227){
						    arfcn_list_by_group[1][counter2++] = arfcn_list[arfcn];
						 }
						 else if(arfcn_list[arfcn]>=228 && arfcn_list[arfcn]<=251){
					       arfcn_list_by_group[2][counter3++] = arfcn_list[arfcn];
						}
					  }
		              *max_rows = 3;
				 	  break;
		case gsm_900: counter = 0x00;
                      for(int32_t arfcn = 0;arfcn<total_arfcn_count;arfcn++){
      	                 if(arfcn_list[arfcn]>=1 && arfcn_list[arfcn]<=50){
			                arfcn_list_by_group[0][counter1++] = arfcn_list[arfcn];
			             }
			             else if(arfcn_list[arfcn]>=51 && arfcn_list[arfcn]<=100){
			               arfcn_list_by_group[1][counter2++] = arfcn_list[arfcn];
			             }
			             else if(arfcn_list[arfcn]>=101 && arfcn_list[arfcn]<=124){
			              arfcn_list_by_group[2][counter3++] = arfcn_list[arfcn];
		               }
                      }
                      *max_rows = 3;
		   		 	  break;
		case dcs_1800: counter = 0x00;
                       for(int32_t arfcn = 0;arfcn<total_arfcn_count;arfcn++){
                          if(arfcn_list[arfcn]>=512 && arfcn_list[arfcn]<=561){
                            arfcn_list_by_group[0][counter1++] = arfcn_list[arfcn];
                          }
                          else if(arfcn_list[arfcn]>=562 && arfcn_list[arfcn]<=611){
                            arfcn_list_by_group[1][counter2++] = arfcn_list[arfcn];
                          }
                          else if(arfcn_list[arfcn]>=612 && arfcn_list[arfcn]<=661){
                            arfcn_list_by_group[2][counter3++] = arfcn_list[arfcn];
                          }
                          else if(arfcn_list[arfcn]>=662 && arfcn_list[arfcn]<=711){
                            arfcn_list_by_group[3][counter4++] = arfcn_list[arfcn];
                          }
                          else if(arfcn_list[arfcn]>=712 && arfcn_list[arfcn]<=761){
                            arfcn_list_by_group[4][counter5++] = arfcn_list[arfcn];
                          }
                          else if(arfcn_list[arfcn]>=762 && arfcn_list[arfcn]<=811){
                            arfcn_list_by_group[5][counter6++] = arfcn_list[arfcn];
                          }
                          else if(arfcn_list[arfcn]>=812 && arfcn_list[arfcn]<=861){
                            arfcn_list_by_group[6][counter7++] = arfcn_list[arfcn];
                          }
                          else if(arfcn_list[arfcn]>=862 && arfcn_list[arfcn]<=885){
                            arfcn_list_by_group[7][counter8++] = arfcn_list[arfcn];
                          }
                       }
                       *max_rows = 8;
				 	   break;
		case pcs_1900: counter = 0x00;
                       for(int32_t arfcn = 0;arfcn<total_arfcn_count;arfcn++){
                          if(arfcn_list[arfcn]>=512 && arfcn_list[arfcn]<=561){
                            arfcn_list_by_group[0][counter1++] = arfcn_list[arfcn];
                          }
                          else if(arfcn_list[arfcn]>=562 && arfcn_list[arfcn]<=611){
                            arfcn_list_by_group[1][counter2++] = arfcn_list[arfcn];
                          }
                          else if(arfcn_list[arfcn]>=612 && arfcn_list[arfcn]<=661){
                            arfcn_list_by_group[2][counter3++] = arfcn_list[arfcn];
                          }
                          else if(arfcn_list[arfcn]>=662 && arfcn_list[arfcn]<=711){
                            arfcn_list_by_group[3][counter4++] = arfcn_list[arfcn];
                          }
                          else if(arfcn_list[arfcn]>=712 && arfcn_list[arfcn]<=761){
                            arfcn_list_by_group[4][counter5++] = arfcn_list[arfcn];
                          }
                          else if(arfcn_list[arfcn]>=762 && arfcn_list[arfcn]<=810){
                            arfcn_list_by_group[5][counter6 ++] = arfcn_list[arfcn];
                          }
                       }
                       *max_rows = 6;
		 		  	   break;
		}

}

static void vipl_scanner_init::get_freq_range_from_arfcn(enum band_details band, int16_t arfcn, struct config *config_to_return){
	switch(band){
	case gsm_450: 	config_to_return->dl_freq = 464e6;
				  	config_to_return->sample_rate = 10e6;
				  	config_to_return->bandwidth = 10e6;
				  	config_to_return->samples_per_burst = config_to_return->sample_rate;
				  	break;
	case gsm_800:  if(arfcn>=0 && arfcn<=49){
                      config_to_return->dl_freq = 939.8e6;
		              config_to_return->sample_rate = 10e6;
		              config_to_return->bandwidth = 10e6;
		              config_to_return->samples_per_burst = config_to_return->sample_rate;
                   }
                   else if(arfcn>=50 && arfcn<=99){
 	                  config_to_return->dl_freq = 949.8e6;
 	                  config_to_return->sample_rate = 10e6;
 	                  config_to_return->bandwidth = 10e6;
 	                  config_to_return->samples_per_burst = config_to_return->sample_rate;
                   }
                   else if(arfcn>=100 && arfcn<=124){
 	                  config_to_return->dl_freq = 957.4e6;
 	                  config_to_return->sample_rate = 10e6;
 	                  config_to_return->bandwidth = 10e6;
 	                  config_to_return->samples_per_burst = config_to_return->sample_rate;
                   }
                   else if(arfcn>=975 && arfcn<=1023){
                	   config_to_return->dl_freq = 930e6;
                	   config_to_return->sample_rate = 10e6;
                	   config_to_return->bandwidth = 10e6;
                	   config_to_return->samples_per_burst = config_to_return->sample_rate;
                   }
	   		   	   break;

	case gsm_850: if(arfcn>=128 && arfcn<=177){
		              config_to_return->dl_freq = 874e6;
					  config_to_return->sample_rate = 10e6;
					  config_to_return->bandwidth = 10e6;
					  config_to_return->samples_per_burst = config_to_return->sample_rate;
	              }
	              else if(arfcn>=178 && arfcn<=227){
	            	  config_to_return->dl_freq = 884e6;
	            	  config_to_return->sample_rate = 10e6;
	                  config_to_return->bandwidth = 10e6;
	            	  config_to_return->samples_per_burst = config_to_return->sample_rate;
	              }
	              else if(arfcn>=228 && arfcn<=251){
	            	  config_to_return->dl_freq = 891.4e6;
	            	  config_to_return->sample_rate = 10e6;
	            	  config_to_return->bandwidth = 10e6;
	            	  config_to_return->samples_per_burst = config_to_return->sample_rate;
	              }
			 	  break;
	case gsm_900: if(arfcn>=1 && arfcn<=50){
		              config_to_return->dl_freq = 940e6;
			     	  config_to_return->sample_rate = 10e6;
					  config_to_return->bandwidth = 10e6;
					  config_to_return->samples_per_burst = config_to_return->sample_rate;
	              }
	              else if(arfcn>=51 && arfcn<=100){
	            	  config_to_return->dl_freq = 950e6;
	            	  config_to_return->sample_rate = 10e6;
	            	  config_to_return->bandwidth = 10e6;
	            	  config_to_return->samples_per_burst = config_to_return->sample_rate;
	              }
	              else if(arfcn>=101 && arfcn<=124){
	            	  config_to_return->dl_freq = 957.4e6;
	            	  config_to_return->sample_rate = 10e6;
	            	  config_to_return->bandwidth = 10e6;
	            	  config_to_return->samples_per_burst = config_to_return->sample_rate;
	              }
	   		 	  break;
	case dcs_1800: if(arfcn>=512 && arfcn<=561){
		               config_to_return->dl_freq = 1810e6;
					   config_to_return->sample_rate = 10e6;
					   config_to_return->bandwidth = 10e6;
					   config_to_return->samples_per_burst = config_to_return->sample_rate;
	               }
	               else if(arfcn>=562 && arfcn<=611){
	            	   config_to_return->dl_freq = 1820e6;
	            	   config_to_return->sample_rate = 10e6;
	            	   config_to_return->bandwidth = 10e6;
	            	   config_to_return->samples_per_burst = config_to_return->sample_rate;
	               }
	               else if(arfcn>=612 && arfcn<=661){
	            	   config_to_return->dl_freq = 1830e6;
	            	   config_to_return->sample_rate = 10e6;
	            	   config_to_return->bandwidth = 10e6;
	            	   config_to_return->samples_per_burst = config_to_return->sample_rate;
	               }
	               else if(arfcn>=662 && arfcn<=711){
	               	   config_to_return->dl_freq = 1840e6;
	               	   config_to_return->sample_rate = 10e6;
	               	   config_to_return->bandwidth = 10e6;
	               	   config_to_return->samples_per_burst = config_to_return->sample_rate;
	               }
	               else if(arfcn>=712 && arfcn<=761){
	               	   config_to_return->dl_freq = 1850e6;
	               	   config_to_return->sample_rate = 10e6;
	               	   config_to_return->bandwidth = 10e6;
	               	   config_to_return->samples_per_burst = config_to_return->sample_rate;
	               }
	               else if(arfcn>=762 && arfcn<=811){
	            	   config_to_return->dl_freq = 1860e6;
	            	   config_to_return->sample_rate = 10e6;
	            	   config_to_return->bandwidth = 10e6;
	            	   config_to_return->samples_per_burst = config_to_return->sample_rate;
	               }
	               else if(arfcn>=812 && arfcn<=861){
	            	   config_to_return->dl_freq = 1870e6;
	            	   config_to_return->sample_rate = 10e6;
	            	   config_to_return->bandwidth = 10e6;
	            	   config_to_return->samples_per_burst = config_to_return->sample_rate;
	               }
	               else if(arfcn>=862 && arfcn<=885){
	            	   config_to_return->dl_freq = 1877.4e6;
	            	   config_to_return->sample_rate = 10e6;
	            	   config_to_return->bandwidth = 10e6;
	            	   config_to_return->samples_per_burst = config_to_return->sample_rate;
	               }
			 	   break;
	case pcs_1900:  if(arfcn>=512 && arfcn<=561){
                       config_to_return->dl_freq = 1935e6;
		               config_to_return->sample_rate = 10e6;
		               config_to_return->bandwidth = 10e6;
		               config_to_return->samples_per_burst = config_to_return->sample_rate;
                    }
                    else if(arfcn>=562 && arfcn<=611){
 	                   config_to_return->dl_freq = 1945e6;
 	                   config_to_return->sample_rate = 10e6;
 	                   config_to_return->bandwidth = 10e6;
 	                   config_to_return->samples_per_burst = config_to_return->sample_rate;
                    }
                    else if(arfcn>=612 && arfcn<=661){
 	                   config_to_return->dl_freq = 1955e6;
 	                   config_to_return->sample_rate = 10e6;
 	                   config_to_return->bandwidth = 10e6;
 	                   config_to_return->samples_per_burst = config_to_return->sample_rate;
                    }
                    else if(arfcn>=662 && arfcn<=711){
    	               config_to_return->dl_freq = 1965e6;
    	               config_to_return->sample_rate = 10e6;
    	               config_to_return->bandwidth = 10e6;
    	               config_to_return->samples_per_burst = config_to_return->sample_rate;
                    }
                    else if(arfcn>=712 && arfcn<=761){
                       config_to_return->dl_freq =1975e6;
                       config_to_return->sample_rate = 10e6;
                       config_to_return->bandwidth = 10e6;
                       config_to_return->samples_per_burst = config_to_return->sample_rate;
                    }
                    else if(arfcn>=762 && arfcn<=810){
                       config_to_return->dl_freq =1985e6;
                       config_to_return->sample_rate = 10e6;
                       config_to_return->bandwidth = 10e6;
                       config_to_return->samples_per_burst = config_to_return->sample_rate;
                    }
	 		  	  	break;
	}
}

struct temp{
	int8_t chain_no;
	bool already_tuned;
};

void vipl_scanner_init::dequeue(tcp_server *tcpServer_gui) {
	boost::thread_group tid_group;
	bool gps_started = false;
	stop_rx = false;
	sem_init(&start_streaming_init, 0, 1);
	sem_init(&start_streaming_init_uplink, 0, 1);
	while(!stop_read_tcp){
		if(!command_queue.empty()){
			struct COMMAND_FROM_GUI commandfromgui = command_queue.front();
			double band_seperation[2]={0x00};
			int32_t mBoard = 0x00;
			int32_t count = 0x00;
			if(commandfromgui.perform_network_scan) {
				bool found = false;
				stop_rx = false;
				struct initialization_status *temp_check = head;
				struct initialization_status *temp_cont;
				if(rx_started) {
					struct RESPONSE_TO_GUI write_to_gui;
					memset(&write_to_gui, 0x00, sizeof(write_to_gui));
					write_to_gui.already_tuned = true;
					uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
					memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
					memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
					tcpServer_gui->write_action(buffer);
					command_queue.pop();
					continue;
				}

				while(temp_check!=NULL){
					if(strcmp(temp_check->mboard_ip,commandfromgui.mboard_ip)==0x00){
						found = true;
						break;
					}
					temp_check = temp_check->next;
				}
				if(!found){
					std::string temp("addr=");
					std::string args = temp+commandfromgui.mboard_ip;
					struct initialization_status *temp_link = (struct initialization_status *)malloc(sizeof(struct initialization_status));
					temp_link->status = true;
					std::string arg("addr=");
					std::string arg_dev = arg+commandfromgui.mboard_ip;
					uhd::device_addr_t dev_addr(arg_dev);
					gr::uhd::usrp_source::sptr uhd_src;
					try{
						temp_link->uhd_src =  gr::uhd::usrp_source::make(dev_addr, uhd::io_type_t::COMPLEX_FLOAT32, 4);

					}catch(const std::exception& ex) {
						//auto s = ex.what();
						char msg[100]={0x00};
						sprintf(msg,"error: lookup error %s", ex.what());
						vipl_printf(msg,error_lvl, __FILE__, __LINE__);
						command_queue.pop();
						struct RESPONSE_TO_GUI write_to_gui;
						memset(&write_to_gui, 0x00, sizeof(write_to_gui));
						write_to_gui.tune_request = false;
						uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
						memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
						memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
						tcpServer_gui->write_action(buffer);
						rx_started = false;
						continue;
					}
					if(head==NULL){
						temp_cont = head = cont = temp_link;
					}else{
						cont->next = temp_cont;
						temp_cont = cont = temp_cont;
					}
				}else{
					temp_cont = temp_check;
				}


				both_bands_done = counter = 0x00;
				char *token = strtok(commandfromgui.band,",");
				//Set band seperation for all bands. Currently 900 and 1800 implemented
				struct gsm_channel_data channel_data[2];
				while(token!=NULL) {
					if(strcmp(token,"GSM_450")==0x00) {
						band_seperation[counter] = 10e6;
						channel_data[counter].center_arfcn = GSM_450_freq;
						channel_data[counter].sample_rate =  SAMPLE_RATE_Cellular_GSM_450;
						channel_data[counter].num_channels = 35;
						channel_data[counter].arfcn_start = 259;
						channel_data[counter].band = gsm_450;
					}else if (strcmp(token,"GSM_850")==0x00) {
						band_seperation[counter] = 45e6;
						channel_data[counter].center_arfcn = GSM_800_freq;
						channel_data[counter].sample_rate =  SAMPLE_RATE_Cellular_GSM_800;
						channel_data[counter].num_channels = 174;
						channel_data[counter].arfcn_start = 0;
						channel_data[counter].band = gsm_800;
					} else if (strcmp(token,"GSM_800")==0x00){
						band_seperation[counter] = 45e6;
						channel_data[counter].center_arfcn = GSM_850_freq;
						channel_data[counter].sample_rate =  SAMPLE_RATE_Cellular_GSM_850;
						channel_data[counter].num_channels = 124;
						channel_data[counter].arfcn_start = 128;
						channel_data[counter].band = gsm_850;
					} else if(strcmp(token,"GSM_900")==0x00) {
						band_seperation[counter] = 45e6;
						channel_data[counter].center_arfcn = 935.2e6;//GSM_900_freq;
						channel_data[counter].sample_rate =  SAMPLE_RATE_Cellular_GSM_900;
						channel_data[counter].num_channels = 124;
						channel_data[counter].arfcn_start = 1;
						channel_data[counter].band = gsm_900;
					}else if(strcmp(token,"DCS_1800")==0x00) {
						band_seperation[counter] = 95e6;
						channel_data[counter].center_arfcn = DCS_1800_freq;
						channel_data[counter].sample_rate =  SAMPLE_RATE_Cellular_DCS_1800;
						channel_data[counter].num_channels = 374;
						channel_data[counter].arfcn_start = 512;
						channel_data[counter].band = dcs_1800;
					}else if(strcmp(token,"PCS_1900")==0x00) {
						band_seperation[counter] = 80e6;
						channel_data[counter].center_arfcn = PCS_1900_freq;
						channel_data[counter].sample_rate =  SAMPLE_RATE_Cellular_PCS_1900;
						channel_data[counter].center_arfcn = PCS_1900_freq;
						channel_data[counter].num_channels = 299;
						channel_data[counter].arfcn_start = 512;
						channel_data[counter].band = pcs_1900;
					}
					counter++;
					token = strtok(NULL,",");
				}
				if(counter==1){
					if(commandfromgui.perform_network_scan){
						usleep(COLD_START_TIME);
						struct RESPONSE_TO_GUI write_to_gui;
						memset(&write_to_gui, 0x00, sizeof(write_to_gui));
						write_to_gui.perform_network_scan = true;
						uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
						memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
						memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
						tcpServer_gui->write_action(buffer);
						rx_started = true;
						channel_data[0].chain_num = 0x00;
						channel_data[0].sample_rate = channel_data[0].samples_per_burst = 2e6;
						channel_data[0].total_no_of_samps = channel_data[0].samples_per_burst;

						temp_cont->uhd_src->set_clock_source("gpsdo",0x00);
						{
							char msg[100]={0x00};
							sprintf(msg,"info: clock source set to %s", temp_cont->uhd_src->get_clock_source(0x00).c_str());
							vipl_printf(msg, error_lvl, __FILE__, __LINE__);
						}
						temp_cont->uhd_src->set_time_source("gpsdo",0x00);
						{
							char msg[100]={0x00};
							sprintf(msg,"info: time source set to %s", temp_cont->uhd_src->get_time_source(0x00).c_str());
							vipl_printf(msg, error_lvl, __FILE__, __LINE__);
						}
						for(int32_t ii=0x00;ii<4;ii++){
							temp_cont->uhd_src->set_auto_iq_balance(true, ii);
							temp_cont->uhd_src->set_auto_dc_offset(true, ii);
						}
						tid_group.add_thread(new boost::thread(ntwrk_scan_start_fft, channel_data[0], tcpServer_gui, temp_cont->uhd_src));
						rx_started = true;
					} else if(commandfromgui.tx_tune_request) {
						boost::thread t2(start_baseband_processing, channel_data[0]);
					}
				}else{
					char *token = strtok(commandfromgui.band,",");
					int32_t j = 0x00;
					//Set band seperation for all bands. Currently 900 and 1800 implemented
					channel_data[1].sample_rate = channel_data[1].samples_per_burst = channel_data[0].sample_rate = channel_data[0].samples_per_burst = 2e6;
					if(commandfromgui.perform_network_scan){
						struct RESPONSE_TO_GUI write_to_gui;
						memset(&write_to_gui, 0x00, sizeof(write_to_gui));
						write_to_gui.perform_network_scan = true;
						uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
						memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
						memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
						tcpServer_gui->write_action(buffer);
						rx_started = true;
						std::string arg("addr=");
						std::string arg_dev = arg+commandfromgui.mboard_ip;
						uhd::device_addr_t dev_addr(arg_dev);
						gr::uhd::usrp_source::sptr uhd_src =  gr::uhd::usrp_source::make(dev_addr, uhd::io_type_t::COMPLEX_FLOAT32, 4);
						uhd_src->set_clock_source("gpsdo",0x00);
						{
							char msg[100]={0x00};
							sprintf(msg,"info: clock source set to %s", uhd_src->get_clock_source(0x00).c_str());
							vipl_printf(msg, error_lvl, __FILE__, __LINE__);
						}
						uhd_src->set_time_source("gpsdo",0x00);
						{
							char msg[100]={0x00};
							sprintf(msg,"info: time source set to %s", uhd_src->get_time_source(0x00).c_str());
							vipl_printf(msg, error_lvl, __FILE__, __LINE__);
						}
						for(int32_t ii=0x00;ii<4;ii++){
							uhd_src->set_auto_iq_balance(true, ii);
							uhd_src->set_auto_dc_offset(true, ii);
						}
						for(int32_t i=0;i<2;i++){
							if(i==1){
								tid_group.add_thread(new boost::thread(ntwrk_scan_start_fft, channel_data[i], tcpServer_gui, uhd_src));
							}else{
								tid_group.add_thread(new boost::thread(ntwrk_scan_start_fft, channel_data[i], tcpServer_gui, uhd_src));
							}
							usleep(100000);
						}

					}
				}
			}
			else if(commandfromgui.tune_request){
				struct COMMAND_FROM_GUI commandfromgui = command_queue.front();
				double band_seperation[2]={0x00};
				int32_t mBoard = 0x00;
				int32_t count = 0x00;
				bool found = false;
				stop_rx = false;
				is_tuned = true;
				stop_demd = false;
				uplink_completed = false;
				struct initialization_status *temp_check = head;
				struct initialization_status *temp_cont;
				sem_init(&start_read, 0, 1);
				downlink_started = false;
				uplink_started = false;
				if(rx_started) {
					struct RESPONSE_TO_GUI write_to_gui;
					memset(&write_to_gui, 0x00, sizeof(write_to_gui));
					write_to_gui.already_tuned = true;
					uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
					memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
					memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
					tcpServer_gui->write_action(buffer);
					command_queue.pop();
					continue;
				}

				while(temp_check!=NULL){
					if(strcmp(temp_check->mboard_ip,commandfromgui.mboard_ip)==0x00){
						found = true;
						break;
					}
					temp_check = temp_check->next;
				}
				if(!found){
					std::string temp("addr=");
					std::string args = temp+commandfromgui.mboard_ip;
					struct initialization_status *temp_link = (struct initialization_status *)malloc(sizeof(struct initialization_status));
					temp_link->status = true;
					std::string arg("addr=");
					std::string arg_dev = arg+commandfromgui.mboard_ip;
					uhd::device_addr_t dev_addr(arg_dev);
					gr::uhd::usrp_source::sptr uhd_src;
					try{
						temp_link->uhd_src =  gr::uhd::usrp_source::make(dev_addr, uhd::io_type_t::COMPLEX_FLOAT32, 4);

					}catch(const std::exception& ex) {
						//auto s = ex.what();
						char msg[100]={0x00};
						sprintf(msg,"error: lookup error %s", ex.what());
						vipl_printf(msg,error_lvl, __FILE__, __LINE__);
						command_queue.pop();
						struct RESPONSE_TO_GUI write_to_gui;
						memset(&write_to_gui, 0x00, sizeof(write_to_gui));
						write_to_gui.tune_request = false;
						uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
						memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
						memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
						tcpServer_gui->write_action(buffer);
						rx_started = false;
						continue;
					}
					if(head==NULL){
						temp_cont = head = cont = temp_link;
					}else{
						cont->next = temp_cont;
						temp_cont = cont = temp_cont;
					}
				}else{
					temp_cont = temp_check;
				}

				char *token = strtok(commandfromgui.band,",");
				struct gsm_channel_data channel_data[2];
				counter=0x00;
				while(token!=NULL) {
					if(strcmp(token,"GSM_450")==0x00) {
						band_seperation[counter] = 10e6;
						channel_data[counter].center_arfcn = GSM_450_freq;
						channel_data[counter].sample_rate = SAMPLE_RATE_Cellular_GSM_450;
						channel_data[counter].num_channels = 35;
						channel_data[counter].arfcn_start = 259;
						channel_data[counter].band = gsm_450;
					}else if (strcmp(token,"GSM_850")==0x00) {
						band_seperation[counter] = 45e6;
						channel_data[counter].center_arfcn = GSM_800_freq;
						channel_data[counter].sample_rate = SAMPLE_RATE_Cellular_GSM_800;
						channel_data[counter].num_channels = 174;
						channel_data[counter].arfcn_start = 0;
						channel_data[counter].band = gsm_800;
					} else if (strcmp(token,"GSM_800")==0x00){
						band_seperation[counter] = 45e6;
						channel_data[counter].center_arfcn = GSM_850_freq;
						channel_data[counter].sample_rate = SAMPLE_RATE_Cellular_GSM_850;
						channel_data[counter].num_channels = 124;
						channel_data[counter].arfcn_start = 128;
						channel_data[counter].band = gsm_850;
					} else if(strcmp(token,"GSM_900")==0x00) {
						band_seperation[counter] = 45e6;
						channel_data[counter].center_arfcn = GSM_900_freq;
						channel_data[counter].sample_rate  = SAMPLE_RATE_Cellular_GSM_900;
						channel_data[counter].num_channels = 124;
						channel_data[counter].arfcn_start = 1;
						channel_data[counter].band = gsm_900;
					}else if(strcmp(token,"DCS_1800")==0x00) {
						band_seperation[counter] = 95e6;
						channel_data[counter].center_arfcn = DCS_1800_freq;
						channel_data[counter].sample_rate = SAMPLE_RATE_Cellular_DCS_1800;
						channel_data[counter].num_channels = 374;
						channel_data[counter].arfcn_start = 512;
						channel_data[counter].band = dcs_1800;
					}else if(strcmp(token,"PCS_1900")==0x00) {
						band_seperation[counter] = 80e6;
						channel_data[counter].center_arfcn = PCS_1900_freq;
						channel_data[counter].sample_rate = SAMPLE_RATE_Cellular_PCS_1900;
						channel_data[counter].center_arfcn = PCS_1900_freq;
						channel_data[counter].num_channels = 299;
						channel_data[counter].arfcn_start = 512;
						channel_data[counter].band = pcs_1900;
					}
					counter++;
					token = strtok(NULL,",");
				}
				if(counter==1) {
					if(commandfromgui.mode == MODE_SIMPLEX){
						int16_t arfcn_list_by_group[5][8]={0x00};
						int32_t max_rows = 0x00;
						group_by(channel_data[0].band, commandfromgui.arfcn_list, arfcn_list_by_group, commandfromgui.number_of_arfcn_to_scan, &max_rows);
						int32_t db_count =0x00;
						for(int32_t arfcn =0;arfcn<max_rows;arfcn++){
							if(arfcn_list_by_group[arfcn][0]==0x00)
								continue;
							db_count++;
						}

						temp_cont->uhd_src->set_clock_source("gpsdo",0x00);
						{
							char msg[100]={0x00};
							sprintf(msg,"info: clock source set to %s", temp_cont->uhd_src->get_clock_source(0x00).c_str());
							vipl_printf(msg, error_lvl, __FILE__, __LINE__);
						}
						temp_cont->uhd_src->set_time_source("gpsdo",0x00);
						{
							char msg[100]={0x00};
							sprintf(msg,"info: time source set to %s", temp_cont->uhd_src->get_time_source(0x00).c_str());
							vipl_printf(msg, error_lvl, __FILE__, __LINE__);
						}
						for(int32_t ii=0x00;ii<db_count;ii++){
							temp_cont->uhd_src->set_auto_iq_balance(true, ii);
							temp_cont->uhd_src->set_auto_dc_offset(true, ii);
						}
						db_count =0x00;
						for(int32_t arfcn =0;arfcn<max_rows;arfcn++){
							if(arfcn_list_by_group[arfcn][0]==0x00)
								continue;
							struct config config_to_return;
							get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);

							channel_data[0].center_arfcn = config_to_return.dl_freq;
							channel_data[0].sample_rate = 10e6;
							channel_data[0].samples_per_burst = 10e6;
							temp_cont->uhd_src->set_center_freq(channel_data[0].center_arfcn, db_count);
							{
								char msg[100]={0x00};
								sprintf(msg,"info: Frequency set to %0.9fMHz rf chain %d",temp_cont->uhd_src->get_center_freq(db_count), db_count);
								vipl_printf(msg, error_lvl, __FILE__, __LINE__);
							}
							temp_cont->uhd_src->set_gain(DEFAULT_GAIN_DWNLINK, db_count);
							{
								char msg[100]={0x00};
								sprintf(msg,"info: gain set to %f rf chain %d",temp_cont->uhd_src->get_gain(db_count), db_count);
								vipl_printf(msg, error_lvl, __FILE__, __LINE__);
							}
							temp_cont->uhd_src->set_bandwidth(channel_data[0].sample_rate, db_count);
							{
								char msg[100]={0x00};
								sprintf(msg,"info: bandwidth set to %fMHz rf chain %d",temp_cont->uhd_src->get_bandwidth(db_count), db_count);
								vipl_printf(msg, error_lvl, __FILE__, __LINE__);
							}
							temp_cont->uhd_src->set_samp_rate(channel_data[0].sample_rate);
							{
								char msg[100]={0x00};
								sprintf(msg,"info: sample rate set to %fmsps rf chain %d",temp_cont->uhd_src->get_samp_rate(), db_count);
								vipl_printf(msg, error_lvl, __FILE__, __LINE__);
							}
							switch(db_count){
							case 0:	temp_cont->uhd_src->set_antenna("RX1",db_count);
									{
										char msg[100]={0x00};
										sprintf(msg,"info: Antenna used with rf chain %d is %s",db_count, temp_cont->uhd_src->get_antenna(db_count).c_str());
										vipl_printf(msg, error_lvl, __FILE__, __LINE__);
									}
									break;
							case 1:	temp_cont->uhd_src->set_antenna("RX2",db_count);
									{
										char msg[100]={0x00};
										sprintf(msg,"info: Antenna used with rf chain %d is %s",db_count, temp_cont->uhd_src->get_antenna(db_count).c_str());
										vipl_printf(msg, error_lvl, __FILE__, __LINE__);
									}
									break;
							case 2:	temp_cont->uhd_src->set_antenna("RX1",db_count);
									{
										char msg[100]={0x00};
										sprintf(msg,"info: Antenna used with rf chain %d is %s",db_count, temp_cont->uhd_src->get_antenna(db_count).c_str());
										vipl_printf(msg, error_lvl, __FILE__, __LINE__);
									}
									break;
							case 3:	temp_cont->uhd_src->set_antenna("RX2",db_count);
									{
										char msg[100]={0x00};
										sprintf(msg,"info: Antenna used with rf chain %d is %s",db_count, temp_cont->uhd_src->get_antenna(db_count).c_str());
										vipl_printf(msg, error_lvl, __FILE__, __LINE__);
									}
									break;
							}

							db_count++;
						}
						db_count =0x00;
						bool done1,done2,done3,done4,done5,done6,done7,done8,done9;
						done1=done2=done3=done4=done5=done6=done7=done8=done9=false;
						int32_t port_no[4]={0x00};
						int32_t num_channels =0x00;
						struct gsm_channel_data channel_data_local[4];
						for(int32_t bb=0x00;bb<4;bb++)
							memcpy(&channel_data_local[bb], &channel_data[0], sizeof(struct gsm_channel_data));
						int32_t rf_chain_id =0x00;
						int32_t channel_count =0x00;
						for(int32_t arfcn =0;arfcn<max_rows;arfcn++) {
							if(arfcn_list_by_group[arfcn][0]==0x00)
								continue;
							if(rf_chain_id>3)
								break;
							if(arfcn==0) {
								port_no[arfcn]=rf_chain_id;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								channel_data_local[rf_chain_id].num_channels = cnt;
								num_channels+=cnt;
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
							}
							else if(arfcn==1) {
								port_no[arfcn]=rf_chain_id;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								channel_data_local[rf_chain_id].num_channels = cnt;
								num_channels+=cnt;
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
							}
							else if(arfcn==2) {
								port_no[arfcn]=rf_chain_id;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
							}
							else if(arfcn==3) {
								port_no[arfcn]=rf_chain_id;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
							}
							else if(arfcn==4) {
								port_no[arfcn]=rf_chain_id;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								channel_data_local[rf_chain_id].num_channels = cnt;
								num_channels+=cnt;
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
							}
							else if(arfcn==5) {
								port_no[arfcn]=rf_chain_id;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
							}
							else if(arfcn==6) {
								port_no[arfcn]=rf_chain_id;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								channel_data_local[rf_chain_id].num_channels = cnt;
								num_channels+=cnt;
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
							}
							else if(arfcn==7) {
								port_no[arfcn]=rf_chain_id;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								channel_data_local[rf_chain_id].num_channels = cnt;
								num_channels+=cnt;
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
							}
							rf_chain_id++;
						}
						tid_group.add_thread(new boost::thread(demod_agcch, channel_data_local, false, temp_cont->uhd_src, port_no, num_channels, rf_chain_id));
						sem_wait(&start_streaming_init);
						struct RESPONSE_TO_GUI write_to_gui;
						memset(&write_to_gui, 0x00, sizeof(write_to_gui));
						write_to_gui.tune_request = true;
						write_to_gui.already_tuned = false;
						uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
						memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
						memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
						tcpServer_gui->write_action(buffer);
					}else if(commandfromgui.mode == MODE_DUPLEX){
						int16_t arfcn_list_by_group[5][8]={0x00};
						int32_t max_rows = 0x00;
						group_by(channel_data[0].band, commandfromgui.arfcn_list, arfcn_list_by_group, commandfromgui.number_of_arfcn_to_scan, &max_rows);
						int32_t db_count =0x00;
						for(int32_t arfcn =0;arfcn<max_rows;arfcn++){
							if(arfcn_list_by_group[arfcn][0]==0x00)
								continue;
							db_count++;
						}

						temp_cont->uhd_src->set_clock_source("gpsdo",0x00);
						{
							char msg[100]={0x00};
							sprintf(msg,"info: clock source set to %s", temp_cont->uhd_src->get_clock_source(0x00).c_str());
							vipl_printf(msg, error_lvl, __FILE__, __LINE__);
						}
						temp_cont->uhd_src->set_time_source("gpsdo",0x00);
						{
							char msg[100]={0x00};
							sprintf(msg,"info: time source set to %s", temp_cont->uhd_src->get_time_source(0x00).c_str());
							vipl_printf(msg, error_lvl, __FILE__, __LINE__);
						}
						for(int32_t ii=0x00;ii<db_count;ii++){
							temp_cont->uhd_src->set_auto_iq_balance(true, ii);
							temp_cont->uhd_src->set_auto_dc_offset(true, ii);
						}
						db_count =0x00;
						for(int32_t arfcn =0;arfcn<max_rows;arfcn++){
							if(arfcn_list_by_group[arfcn][0]==0x00)
								continue;
							if(db_count>2)
								break;
							struct config config_to_return;
							get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
							channel_data[0].center_arfcn = config_to_return.dl_freq;
							channel_data[0].sample_rate = 10e6;
							channel_data[0].samples_per_burst = 10e6;
							temp_cont->uhd_src->set_center_freq(channel_data[0].center_arfcn, db_count);
							{
								char msg[100]={0x00};
								sprintf(msg,"info: Frequency set to %0.9fMHz rf chain %d",temp_cont->uhd_src->get_center_freq(db_count), db_count);
								vipl_printf(msg, error_lvl, __FILE__, __LINE__);
							}
							temp_cont->uhd_src->set_center_freq(channel_data[0].center_arfcn-band_seperation[0], db_count+1);
							{
								char msg[100]={0x00};
								sprintf(msg,"info: Frequency set to %0.9fMHz rf chain %d",temp_cont->uhd_src->get_center_freq(db_count+1), db_count+1);
								vipl_printf(msg, error_lvl, __FILE__, __LINE__);
							}
							temp_cont->uhd_src->set_gain(DEFAULT_GAIN_DWNLINK_NORMAL_SCAN, db_count);
							{
								char msg[100]={0x00};
								sprintf(msg,"info: gain set to %f rf chain %d",temp_cont->uhd_src->get_gain(db_count), db_count);
								vipl_printf(msg, error_lvl, __FILE__, __LINE__);
							}

							temp_cont->uhd_src->set_gain(DEFAULT_GAIN_UPLINK_NORMAL_SCAN, db_count+1);
							{
								char msg[100]={0x00};
								sprintf(msg,"info: gain set to %f rf chain %d",temp_cont->uhd_src->get_gain(db_count+1), db_count+1);
								vipl_printf(msg, error_lvl, __FILE__, __LINE__);
							}

							temp_cont->uhd_src->set_bandwidth(channel_data[0].sample_rate, db_count);
							{
								char msg[100]={0x00};
								sprintf(msg,"info: bandwidth set to %fMHz rf chain %d",temp_cont->uhd_src->get_bandwidth(db_count), db_count);
								vipl_printf(msg, error_lvl, __FILE__, __LINE__);
							}

							temp_cont->uhd_src->set_bandwidth(channel_data[0].sample_rate, db_count+1);
							{
								char msg[100]={0x00};
								sprintf(msg,"info: bandwidth set to %fMHz rf chain %d",temp_cont->uhd_src->get_bandwidth(db_count+1), db_count+1);
								vipl_printf(msg, error_lvl, __FILE__, __LINE__);
							}
							temp_cont->uhd_src->set_samp_rate(channel_data[0].sample_rate);
							{
								char msg[100]={0x00};
								sprintf(msg,"info: sample rate set to %fmsps rf chain %d",temp_cont->uhd_src->get_samp_rate(), db_count);
								vipl_printf(msg, error_lvl, __FILE__, __LINE__);
							}

							switch(db_count){
							case 0:	temp_cont->uhd_src->set_antenna("RX1",db_count);
									{
										char msg[100]={0x00};
										sprintf(msg,"info: Antenna used with rf chain %d is %s",db_count, temp_cont->uhd_src->get_antenna(db_count).c_str());
										vipl_printf(msg, error_lvl, __FILE__, __LINE__);
									}
									temp_cont->uhd_src->set_antenna("RX2",db_count+1);
									{
										char msg[100]={0x00};
										sprintf(msg,"info: Antenna used with rf chain %d is %s",db_count+1, temp_cont->uhd_src->get_antenna(db_count+1).c_str());
										vipl_printf(msg, error_lvl, __FILE__, __LINE__);
									}
									break;
							case 2:	temp_cont->uhd_src->set_antenna("RX1",db_count);
									{
										char msg[100]={0x00};
										sprintf(msg,"info: Antenna used with rf chain %d is %s",db_count, temp_cont->uhd_src->get_antenna(db_count).c_str());
										vipl_printf(msg, error_lvl, __FILE__, __LINE__);
									}
									temp_cont->uhd_src->set_antenna("RX2",db_count+1);
									{
										char msg[100]={0x00};
										sprintf(msg,"info: Antenna used with rf chain %d is %s",db_count+1, temp_cont->uhd_src->get_antenna(db_count+1).c_str());
										vipl_printf(msg, error_lvl, __FILE__, __LINE__);
									}
									break;
							}

							db_count+=2;
						}
						db_count =0x00;
						bool done1,done2,done3,done4,done5,done6,done7,done8,done9;
						done1=done2=done3=done4=done5=done6=done7=done8=done9=false;
						int32_t port_no[4]={0x00};
						int32_t num_channels =0x00;
						struct gsm_channel_data channel_data_local[4];
						for(int32_t bb=0x00;bb<4;bb++)
							memcpy(&channel_data_local[bb], &channel_data[0], sizeof(struct gsm_channel_data));
						int32_t rf_chain_id =0x00;
						int32_t channel_count =0x00;
						for(int32_t arfcn =0;arfcn<max_rows;arfcn++) {
							if(arfcn_list_by_group[arfcn][0]==0x00)
								continue;
							if(rf_chain_id>2)
								break;
							if(arfcn==0) {
								port_no[arfcn]=rf_chain_id;
								//port_no[arfcn+1]=rf_chain_id+1;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								channel_data_local[rf_chain_id].num_channels = cnt;
								num_channels+=cnt;
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								db_count++;
							}
							else if(arfcn==1) {
								port_no[arfcn]=rf_chain_id;
								//port_no[arfcn+1]=rf_chain_id+1;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								channel_data_local[rf_chain_id].num_channels = cnt;
								num_channels+=cnt;
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
							}
							else if(arfcn==2) {
								port_no[arfcn]=rf_chain_id;
								//port_no[arfcn+1]=rf_chain_id+1;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								db_count++;
							}
							else if(arfcn==3) {
								port_no[arfcn]=rf_chain_id;
								//port_no[arfcn+1]=rf_chain_id+1;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								db_count++;
							}
							else if(arfcn==4) {
								port_no[arfcn]=rf_chain_id;
								//port_no[arfcn+1]=rf_chain_id+1;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								channel_data_local[rf_chain_id].num_channels = cnt;
								num_channels+=cnt;
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								db_count++;
							}
							else if(arfcn==5) {
								port_no[arfcn]=rf_chain_id;
								//port_no[arfcn+1]=rf_chain_id+1;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								db_count++;
							}
							else if(arfcn==6) {
								port_no[arfcn]=rf_chain_id;
								//port_no[arfcn+1]=rf_chain_id+1;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								channel_data_local[rf_chain_id].num_channels = cnt;
								num_channels+=cnt;
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								db_count++;
							}
							else if(arfcn==7) {
								port_no[arfcn]=rf_chain_id;
								//port_no[arfcn+1]=rf_chain_id+1;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
								int32_t cnt = 0x00;
								for(cnt=0x00; cnt<8; cnt++) {
									if(arfcn_list_by_group[arfcn][cnt]==0)
										break;
								}
								channel_data_local[rf_chain_id].num_channels = cnt;
								num_channels+=cnt;
								memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								db_count++;
							}

							rf_chain_id+=1;
						}
						tid_group.add_thread(new boost::thread(demod_agcch, channel_data_local, true, temp_cont->uhd_src, port_no, num_channels, rf_chain_id));
						sem_wait(&start_streaming_init);
						struct RESPONSE_TO_GUI write_to_gui;
						memset(&write_to_gui, 0x00, sizeof(write_to_gui));
						write_to_gui.tune_request = true;
						write_to_gui.already_tuned = false;
						uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
						memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
						memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
						tcpServer_gui->write_action(buffer);

					}
				}else if(counter==2) {
					if(commandfromgui.mode == MODE_SIMPLEX){
						int32_t rf_chain_id =0x00;
						int32_t port_no[4]={0x00};
						int32_t num_channels =0x00;
						struct gsm_channel_data channel_data_local[4];
						for(int32_t i=0x00;i<counter;i++) {
							int16_t arfcn_list_by_group[5][8]={0x00};
							int32_t max_rows = 0x00;
							group_by(channel_data[0].band, commandfromgui.arfcn_list, arfcn_list_by_group, commandfromgui.number_of_arfcn_to_scan, &max_rows);
							int32_t channel_counter = count_channel(channel_data[i].band, commandfromgui.arfcn_list, commandfromgui.number_of_arfcn_to_scan);
							for(int32_t arfcn =0;arfcn<max_rows;arfcn++) {
								if(arfcn_list_by_group[arfcn][0]==0x00)
									continue;
								if(rf_chain_id>3)
									break;
								if(arfcn==0) {
									port_no[arfcn]=rf_chain_id;
									struct config config_to_return;
									get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
									channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
									int32_t cnt = 0x00;
									for(cnt=0x00; cnt<8; cnt++) {
										if(arfcn_list_by_group[arfcn][cnt]==0)
											break;
									}
									channel_data_local[rf_chain_id].num_channels = cnt;
									num_channels+=cnt;
									memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								}else if(arfcn==1) {
									port_no[arfcn]=rf_chain_id;
									struct config config_to_return;
									get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
									channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
									int32_t cnt = 0x00;
									for(cnt=0x00; cnt<8; cnt++) {
										if(arfcn_list_by_group[arfcn][cnt]==0)
											break;
									}
									channel_data_local[rf_chain_id].num_channels = cnt;
									num_channels+=cnt;
									memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								}else if(arfcn==2) {
									port_no[arfcn]=rf_chain_id;
									struct config config_to_return;
									get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
									channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
									int32_t cnt = 0x00;
									for(cnt=0x00; cnt<8; cnt++) {
										if(arfcn_list_by_group[arfcn][cnt]==0)
											break;
									}
									memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								}else if(arfcn==3) {
									port_no[arfcn]=rf_chain_id;
									struct config config_to_return;
									get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
									channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
									int32_t cnt = 0x00;
									for(cnt=0x00; cnt<8; cnt++) {
										if(arfcn_list_by_group[arfcn][cnt]==0)
											break;
									}
									memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								}else if(arfcn==4) {
									port_no[arfcn]=rf_chain_id;
									struct config config_to_return;
									get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
									channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
									int32_t cnt = 0x00;
									for(cnt=0x00; cnt<8; cnt++) {
										if(arfcn_list_by_group[arfcn][cnt]==0)
											break;
									}
									channel_data_local[rf_chain_id].num_channels = cnt;
									num_channels+=cnt;
									memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								}else if(arfcn==5) {
									port_no[arfcn]=rf_chain_id;
									struct config config_to_return;
									get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
									channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
									int32_t cnt = 0x00;
									for(cnt=0x00; cnt<8; cnt++) {
										if(arfcn_list_by_group[arfcn][cnt]==0)
											break;
									}
									memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								}else if(arfcn==6) {
									port_no[arfcn]=rf_chain_id;
									struct config config_to_return;
									get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
									channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
									int32_t cnt = 0x00;
									for(cnt=0x00; cnt<8; cnt++) {
										if(arfcn_list_by_group[arfcn][cnt]==0)
											break;
									}
									channel_data_local[rf_chain_id].num_channels = cnt;
									num_channels+=cnt;
									memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								}else if(arfcn==7) {
									port_no[arfcn]=rf_chain_id;
									struct config config_to_return;
									get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
									channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
									int32_t cnt = 0x00;
									for(cnt=0x00; cnt<8; cnt++) {
										if(arfcn_list_by_group[arfcn][cnt]==0)
											break;
									}
									channel_data_local[rf_chain_id].num_channels = cnt;
									num_channels+=cnt;
									memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								}
								rf_chain_id++;
							}
						}
						tid_group.add_thread(new boost::thread(demod_agcch, channel_data_local, false, temp_cont->uhd_src, port_no, num_channels, rf_chain_id));
						sem_wait(&start_streaming_init);
						struct RESPONSE_TO_GUI write_to_gui;
						memset(&write_to_gui, 0x00, sizeof(write_to_gui));
						write_to_gui.tune_request = true;
						write_to_gui.already_tuned = false;
						uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
						memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
						memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
						tcpServer_gui->write_action(buffer);
					}else if(commandfromgui.mode == MODE_DUPLEX){
						int32_t rf_chain_id =0x00;
						int32_t port_no[4]={0x00};
						int32_t num_channels =0x00;
						struct gsm_channel_data channel_data_local[4];
						for(int32_t i=0x00;i<counter;i++) {
							int16_t arfcn_list_by_group[5][8]={0x00};
							int32_t max_rows = 0x00;
							group_by(channel_data[i].band, commandfromgui.arfcn_list, arfcn_list_by_group, commandfromgui.number_of_arfcn_to_scan, &max_rows);
							int32_t db_count =0x00;
							for(int32_t arfcn =0;arfcn<max_rows;arfcn++){
								if(arfcn_list_by_group[arfcn][0]==0x00)
									continue;
								db_count++;
							}

							temp_cont->uhd_src->set_clock_source("gpsdo",0x00);
							{
								char msg[100]={0x00};
								sprintf(msg,"info: clock source set to %s", temp_cont->uhd_src->get_clock_source(0x00).c_str());
								vipl_printf(msg, error_lvl, __FILE__, __LINE__);
							}
							temp_cont->uhd_src->set_time_source("gpsdo",0x00);
							{
								char msg[100]={0x00};
								sprintf(msg,"info: time source set to %s", temp_cont->uhd_src->get_time_source(0x00).c_str());
								vipl_printf(msg, error_lvl, __FILE__, __LINE__);
							}
							for(int32_t ii=0x00;ii<db_count;ii++){
								temp_cont->uhd_src->set_auto_iq_balance(true, ii);
								temp_cont->uhd_src->set_auto_dc_offset(true, ii);
							}
							db_count =0x00;
							for(int32_t arfcn =0;arfcn<max_rows;arfcn++){
								if(arfcn_list_by_group[arfcn][0]==0x00)
									continue;
								if(db_count>2)
									break;
								struct config config_to_return;
								get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
								channel_data[0].center_arfcn = config_to_return.dl_freq;
								channel_data[0].sample_rate = 10e6;
								channel_data[0].samples_per_burst = 10e6;
								temp_cont->uhd_src->set_center_freq(channel_data[i].center_arfcn, db_count);
								{
									char msg[100]={0x00};
									sprintf(msg,"info: Frequency set to %0.9fMHz rf chain %d",temp_cont->uhd_src->get_center_freq(db_count), db_count);
									vipl_printf(msg, error_lvl, __FILE__, __LINE__);
								}
								temp_cont->uhd_src->set_center_freq(channel_data[i].center_arfcn-band_seperation[0], db_count+1);
								{
									char msg[100]={0x00};
									sprintf(msg,"info: Frequency set to %0.9fMHz rf chain %d",temp_cont->uhd_src->get_center_freq(db_count+1), db_count+1);
									vipl_printf(msg, error_lvl, __FILE__, __LINE__);
								}
								temp_cont->uhd_src->set_gain(DEFAULT_GAIN_DWNLINK_NORMAL_SCAN, db_count);
								{
									char msg[100]={0x00};
									sprintf(msg,"info: gain set to %f rf chain %d",temp_cont->uhd_src->get_gain(db_count), db_count);
									vipl_printf(msg, error_lvl, __FILE__, __LINE__);
								}

								temp_cont->uhd_src->set_gain(DEFAULT_GAIN_UPLINK_NORMAL_SCAN, db_count+1);
								{
									char msg[100]={0x00};
									sprintf(msg,"info: gain set to %f rf chain %d",temp_cont->uhd_src->get_gain(db_count+1), db_count+1);
									vipl_printf(msg, error_lvl, __FILE__, __LINE__);
								}

								temp_cont->uhd_src->set_bandwidth(channel_data[i].sample_rate, db_count);
								{
									char msg[100]={0x00};
									sprintf(msg,"info: bandwidth set to %fMHz rf chain %d",temp_cont->uhd_src->get_bandwidth(db_count), db_count);
									vipl_printf(msg, error_lvl, __FILE__, __LINE__);
								}

								temp_cont->uhd_src->set_bandwidth(channel_data[i].sample_rate, db_count+1);
								{
									char msg[100]={0x00};
									sprintf(msg,"info: bandwidth set to %fMHz rf chain %d",temp_cont->uhd_src->get_bandwidth(db_count+1), db_count+1);
									vipl_printf(msg, error_lvl, __FILE__, __LINE__);
								}
								temp_cont->uhd_src->set_samp_rate(channel_data[0].sample_rate);
								{
									char msg[100]={0x00};
									sprintf(msg,"info: sample rate set to %fmsps rf chain %d",temp_cont->uhd_src->get_samp_rate(), db_count);
									vipl_printf(msg, error_lvl, __FILE__, __LINE__);
								}

								switch(db_count) {
								case 0:	temp_cont->uhd_src->set_antenna("RX1",db_count);
										{
											char msg[100]={0x00};
											sprintf(msg,"info: Antenna used with rf chain %d is %s",db_count, temp_cont->uhd_src->get_antenna(db_count).c_str());
											vipl_printf(msg, error_lvl, __FILE__, __LINE__);
										}
										temp_cont->uhd_src->set_antenna("RX2",db_count+1);
										{
											char msg[100]={0x00};
											sprintf(msg,"info: Antenna used with rf chain %d is %s",db_count+1, temp_cont->uhd_src->get_antenna(db_count+1).c_str());
											vipl_printf(msg, error_lvl, __FILE__, __LINE__);
										}
										break;
								case 2:	temp_cont->uhd_src->set_antenna("RX1",db_count);
										{
											char msg[100]={0x00};
											sprintf(msg,"info: Antenna used with rf chain %d is %s",db_count, temp_cont->uhd_src->get_antenna(db_count).c_str());
											vipl_printf(msg, error_lvl, __FILE__, __LINE__);
										}
										temp_cont->uhd_src->set_antenna("RX2",db_count+1);
										{
											char msg[100]={0x00};
											sprintf(msg,"info: Antenna used with rf chain %d is %s",db_count+1, temp_cont->uhd_src->get_antenna(db_count+1).c_str());
											vipl_printf(msg, error_lvl, __FILE__, __LINE__);
										}
										break;
								}

								db_count+=2;
							}
							db_count =0x00;
							bool done1,done2,done3,done4,done5,done6,done7,done8,done9;
							done1=done2=done3=done4=done5=done6=done7=done8=done9=false;
							int32_t port_no[4]={0x00};
							int32_t num_channels =0x00;
							struct gsm_channel_data channel_data_local[4];
							for(int32_t bb=0x00;bb<4;bb++)
								memcpy(&channel_data_local[bb], &channel_data[i], sizeof(struct gsm_channel_data));
							int32_t rf_chain_id =0x00;
							int32_t channel_count =0x00;
							for(int32_t arfcn =0;arfcn<max_rows;arfcn++) {
								if(arfcn_list_by_group[arfcn][0]==0x00)
									continue;
								if(rf_chain_id>2)
									break;
								if(arfcn==0) {
									port_no[arfcn]=rf_chain_id;
									//port_no[arfcn+1]=rf_chain_id+1;
									struct config config_to_return;
									get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
									channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
									int32_t cnt = 0x00;
									for(cnt=0x00; cnt<8; cnt++) {
										if(arfcn_list_by_group[arfcn][cnt]==0)
											break;
									}
									channel_data_local[rf_chain_id].num_channels = cnt;
									num_channels+=cnt;
									memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
									db_count++;
								}
								else if(arfcn==1) {
									port_no[arfcn]=rf_chain_id;
									//port_no[arfcn+1]=rf_chain_id+1;
									struct config config_to_return;
									get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
									channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
									int32_t cnt = 0x00;
									for(cnt=0x00; cnt<8; cnt++) {
										if(arfcn_list_by_group[arfcn][cnt]==0)
											break;
									}
									channel_data_local[rf_chain_id].num_channels = cnt;
									num_channels+=cnt;
									memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
								}
								else if(arfcn==2) {
										port_no[arfcn]=rf_chain_id;
										//port_no[arfcn+1]=rf_chain_id+1;
										struct config config_to_return;
										get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
										channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
										int32_t cnt = 0x00;
										for(cnt=0x00; cnt<8; cnt++) {
											if(arfcn_list_by_group[arfcn][cnt]==0)
												break;
										}
										memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
										db_count++;
									}
									else if(arfcn==3) {
										port_no[arfcn]=rf_chain_id;
										//port_no[arfcn+1]=rf_chain_id+1;
										struct config config_to_return;
										get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
										channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
										int32_t cnt = 0x00;
										for(cnt=0x00; cnt<8; cnt++) {
											if(arfcn_list_by_group[arfcn][cnt]==0)
												break;
										}
										memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
										db_count++;
									}
									else if(arfcn==4) {
										port_no[arfcn]=rf_chain_id;
										//port_no[arfcn+1]=rf_chain_id+1;
										struct config config_to_return;
										get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
										channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
										int32_t cnt = 0x00;
										for(cnt=0x00; cnt<8; cnt++) {
											if(arfcn_list_by_group[arfcn][cnt]==0)
												break;
										}
										channel_data_local[rf_chain_id].num_channels = cnt;
										num_channels+=cnt;
										memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
										db_count++;
									}
									else if(arfcn==5) {
										port_no[arfcn]=rf_chain_id;
										//port_no[arfcn+1]=rf_chain_id+1;
										struct config config_to_return;
										get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
										channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
										int32_t cnt = 0x00;
										for(cnt=0x00; cnt<8; cnt++) {
											if(arfcn_list_by_group[arfcn][cnt]==0)
												break;
										}
										memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
										db_count++;
									}
									else if(arfcn==6) {
										port_no[arfcn]=rf_chain_id;
										//port_no[arfcn+1]=rf_chain_id+1;
										struct config config_to_return;
										get_freq_range_from_arfcn(channel_data[0].band, arfcn_list_by_group[arfcn][0], &config_to_return);
										channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
										int32_t cnt = 0x00;
										for(cnt=0x00; cnt<8; cnt++) {
											if(arfcn_list_by_group[arfcn][cnt]==0)
												break;
										}
										channel_data_local[rf_chain_id].num_channels = cnt;
										num_channels+=cnt;
										memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
										db_count++;
									}
									else if(arfcn==7) {
										port_no[arfcn]=rf_chain_id;
										//port_no[arfcn+1]=rf_chain_id+1;
										struct config config_to_return;
										get_freq_range_from_arfcn(channel_data[i].band, arfcn_list_by_group[arfcn][0], &config_to_return);
										channel_data_local[rf_chain_id].center_arfcn = config_to_return.dl_freq;
										int32_t cnt = 0x00;
										for(cnt=0x00; cnt<8; cnt++) {
											if(arfcn_list_by_group[arfcn][cnt]==0)
												break;
										}
										channel_data_local[rf_chain_id].num_channels = cnt;
										num_channels+=cnt;
										memcpy(channel_data_local[rf_chain_id].arfcn_list, arfcn_list_by_group[arfcn],sizeof(int16_t)*cnt);
										db_count++;
									}
									rf_chain_id+=1;
								}
								tid_group.add_thread(new boost::thread(demod_agcch, channel_data_local, true, temp_cont->uhd_src, port_no, num_channels, rf_chain_id));
								sem_wait(&start_streaming_init);
								struct RESPONSE_TO_GUI write_to_gui;
								memset(&write_to_gui, 0x00, sizeof(write_to_gui));
								write_to_gui.tune_request = true;
								write_to_gui.already_tuned = false;
								uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
								memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
								memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
								tcpServer_gui->write_action(buffer);
						}
					}
				}
			}else if(commandfromgui.tune_gps){
				if(gps_started){
					struct RESPONSE_TO_GUI write_to_gui;
					memset(&write_to_gui, 0x00, sizeof(write_to_gui));
					write_to_gui.already_tuned = true;
					uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
					memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
					memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
					tcpServer_gui->write_action(buffer);
					command_queue.pop();
					continue;
				}
				bool found = false;
				struct GSMConfig config;
				uhd::usrp::multi_usrp::sptr usrp;
				struct initialization_status *temp_check = head;
				struct initialization_status *temp_cont;
				while(temp_check!=NULL){
					if(strcmp(temp_check->mboard_ip,commandfromgui.mboard_ip)==0x00){
						found = true;
						break;
					}
					temp_check =  temp_check->next;
				}
				if(!found){
					std::string temp("addr=");
					std::string args = temp+commandfromgui.mboard_ip;
					struct initialization_status *temp_link = (struct initialization_status *)malloc(sizeof(struct initialization_status));
					temp_link->status = true;
					strcpy(temp_link->mboard_ip,commandfromgui.mboard_ip);
					try{
						temp_link->vipl_rf_interface_obj.usrp = uhd::usrp::multi_usrp::make(args);
					}catch(const std::exception& ex){
						//auto s = ex.what();
						char msg[100]={0x00};
						sprintf(msg,"error: lookup error %s",ex.what());
						vipl_printf(msg,error_lvl, __FILE__, __LINE__);
						command_queue.pop();
						struct RESPONSE_TO_GUI write_to_gui;
						memset(&write_to_gui, 0x00, sizeof(write_to_gui));
						write_to_gui.tune_gps = false;
						uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
						memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
						memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
						tcpServer_gui->write_action(buffer);
						gps_started = false;
						continue;
					}
					mBoard = temp_link->vipl_rf_interface_obj.mboardCount = count;
					temp_link->next = NULL;
					if(temp_link->vipl_rf_interface_obj.usrp==NULL) {
						command_queue.pop();
						continue;
					}
					count++;
					if(head==NULL){
						temp_cont = head = cont = temp_link;
					}else{
						cont->next = temp_cont;
						temp_cont = cont = temp_cont;
					}
				}else{
					temp_cont = temp_check;
				}
				if(!gps_started) {
					tid_group.create_thread(boost::bind(temp_cont->vipl_rf_interface_obj.get_gps_val, temp_cont->vipl_rf_interface_obj.usrp, temp_cont->vipl_rf_interface_obj.tuned));
					//temp_cont->tid = new boost::thread(boost::bind(temp_cont->vipl_rf_interface_obj.get_gps_val, temp_cont->vipl_rf_interface_obj.usrp));
					//temp_cont->tid = t1;
					gps_started = true;
					struct RESPONSE_TO_GUI write_to_gui;
					memset(&write_to_gui, 0x00, sizeof(write_to_gui));
					write_to_gui.tune_gps = true;
					uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
					memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
					memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
					tcpServer_gui->write_action(buffer);
				}
			}else if(commandfromgui.tx_tune_request){
				//TODO
			}else if(commandfromgui.untune) {
				stop_rx = true;
				rx_started = false;
				struct RESPONSE_TO_GUI write_to_gui;
				memset(&write_to_gui, 0x00, sizeof(write_to_gui));
				write_to_gui.tune_request = true;
				write_to_gui.already_tuned = true;
				uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
				memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
				memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
				tcpServer_gui->write_action(buffer);
			}else if(commandfromgui.stop_gps) {
				gps_started = false;
				stop_gps = true;
				usleep(1000);
				stop_gps = false;
			}else if(commandfromgui.change_gain) {
				bool found = false;
				struct initialization_status *temp_check = head;
				struct initialization_status *temp_cont;
				while(temp_check!=NULL) {
					if(strcmp(temp_check->mboard_ip,commandfromgui.mboard_ip)==0x00)
						found = true;
					temp_check =  temp_check->next;
				}
				if(found){
					temp_cont = temp_check;
					int8_t rtnval = temp_cont->vipl_rf_interface_obj.set_gain(temp_cont->vipl_rf_interface_obj.usrp, commandfromgui.gain, 0x00);
				}
			}
			command_queue.pop();
		}else{
			sem_wait(&command_wait);
		}
	}
	if(error_lvl==3)
			vipl_printf("info: dequeue stopped", error_lvl, __FILE__, __LINE__);
}

void vipl_scanner_init::start(void){
	boost::thread(dequeue);
}
