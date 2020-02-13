/*
 * downgrader.cpp
 *
 *  Created on: 06-Jan-2020
 *  Author: Snehasish kar
 */

#include "../include/downgrader.h"
#include "../include/vipl_printf.hpp"

downgrader::downgrader() {
	// TODO Auto-generated constructor stub

}

downgrader::~downgrader() {
	vipl_printf("info: downgrader stopped!!", error_lvl, __FILE__, __LINE__);
}
void downgrader::setup(uhd::usrp::multi_usrp::sptr usrp, int32_t mBoard, struct GSMConfig_downgrader config, int8_t chain_num){
	try{
		usrp->set_rx_gain(DEFAULT_GAIN_RX_DOWNGRADER, chain_num);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: current rx gain set to %fdb to db %d mBoard %d", usrp->get_rx_gain(chain_num), chain_num, mBoard);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		usrp->set_tx_gain(DEFAULT_GAIN_TX_DOWNGRADER, chain_num);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: current tx gain set to %fdb to db %d mBoard %d", usrp->get_tx_gain(chain_num), chain_num, mBoard);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		usrp->set_rx_antenna("RX2", chain_num);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: RX Antenna set to %s in db %d of mBoard", usrp->get_rx_antenna(chain_num).c_str(), chain_num,  mBoard);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		usrp->set_tx_antenna("TX/RX", chain_num);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: TX Antenna set to %s in db %d of mBoard", usrp->get_tx_antenna(chain_num).c_str(), chain_num,  mBoard);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		usrp->set_rx_bandwidth(config.bandwidth, chain_num);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: bandwidth set to %fMHz db %d mBoard %d", usrp->get_rx_bandwidth(chain_num), chain_num,  mBoard);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		usrp->set_tx_bandwidth(config.bandwidth, chain_num);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: bandwidth set to %fMHz db %d mBoard %d", usrp->get_rx_bandwidth(chain_num), chain_num,  mBoard);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd::tune_request_t tune_request_rx(config.dl_freq_a, 0.00);
		usrp->set_rx_freq(tune_request_rx, chain_num);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: current rx freq set to %fMHz db %d mBoard %d", usrp->get_tx_gain(chain_num), chain_num,  mBoard);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		uhd::tune_request_t tune_request_tx(config.ul_freq_a, 0.00);
		usrp->set_tx_freq(tune_request_tx, chain_num);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: current tx freq set to %fMHz db %d mBoard %d", usrp->get_tx_gain(chain_num), chain_num,  mBoard);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
	}catch(const std::exception& ex){
		 char msg[100]={0x00};
		 sprintf(msg,"error: %s",ex.what());
		 vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		 usrp->clear_command_time(0);
		 usrp->~multi_usrp();
	 }
}

void downgrader::start_rx_streaming(uhd::usrp::multi_usrp::sptr usrp, double samples_per_burst, int32_t channel_no, cbuffercf *cb, unsigned long long total_no_samps){
	std::vector<std::string> blocks;
	/*
	 * create a receive streamer
	 */
	uhd::stream_args_t stream_args("fc32","sc16");
	std::vector<size_t> channel_nums;
	std::vector<std::complex<float>> buffer(samples_per_burst);
	unsigned long long total_samps_recvd =0x00;
	channel_nums.push_back(channel_no);
	stream_args.channels = channel_nums;
	uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);
	uhd::rx_metadata_t md;
	bool overflow_message = true;
	uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
	stream_cmd.num_samps  = 0x00;
	stream_cmd.stream_now = true;
	stream_cmd.time_spec =  uhd::time_spec_t(0.00);
	rx_stream->issue_stream_cmd(stream_cmd);
	//sem_wait(&sem_status_stream);
	bool discard_first_set_of_samples = false;
	uint32_t row_count =0x00;
	while(!stop_rx and ((total_samps_recvd!=total_no_samps) or (total_no_samps==0x00))) {
		buffer.clear();
		size_t num_rx_samps;
		try{
			num_rx_samps = rx_stream->recv(&buffer.front(), samples_per_burst, md, 3.0);
			if(error_lvl==3) {
				fprintf(stderr,"\n========================================================\n");
				fprintf(stderr,"  Number of samples received %u db %d \n",num_rx_samps, channel_no);
				fprintf(stderr,"\n========================================================\n");
			}
		}catch (const std::exception& ex){
			//auto s = ex.what();
			char msg[100]={0x00};
			sprintf(msg,"error: error while streaming %s", ex.what());
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
			vipl_printf("error: timeout while streaming", error_lvl, __FILE__, __LINE__);
			break;
		}
		if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
			if (overflow_message) {
				overflow_message = false;
				vipl_printf("error: overflow detected", error_lvl, __FILE__, __LINE__);
			}
			continue;
		}
		if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
			char msg[100]={0x00};
			sprintf(msg,"error: %s",md.strerror());
			vipl_printf(msg , error_lvl, __FILE__, __LINE__);
		}
		if(discard_first_set_of_samples==false){
			discard_first_set_of_samples=true;
			continue;
		}
	#if 1
		if(((cbuffercf_size(*cb)-num_rx_samps)>=(cbuffercf_max_size(*cb))||(cbuffercf_size(*cb)-num_rx_samps)==(0))&&(cbuffercf_size(*cb)!=0)) {
			cbuffercf_release(*cb,(uint32_t)(num_rx_samps));
		}
		cbuffercf_write(*cb, &buffer.front(), num_rx_samps);
		total_samps_recvd+=num_rx_samps;
	#endif
	}
	vipl_printf("info: downgrader rx streaming stopped!!", error_lvl, __FILE__, __LINE__);
}

void downgrader::start_tx_streaming(uhd::usrp::multi_usrp::sptr usrp, double samples_per_burst, int32_t channel_no, cbuffercf *cb, unsigned long long total_no_samps){
	 uhd::tx_streamer::sptr tx_stream;
	 uhd::tx_metadata_t md;
	 md.start_of_burst = false;
	 md.end_of_burst = false;
	 std::vector<std::complex<float>> buff(samples_per_burst);
	 size_t num_tx_samps = size_t(samples_per_burst/sizeof(std::complex<float>));
	 while(not md.end_of_burst and not stop_rx) {
		 size_t num_samps_wrote = 0x00;
		 try{
			 num_samps_wrote = tx_stream->send(&buff.front(), num_tx_samps, md);
		 }catch (const std::exception& ex) {
			//auto s = ex.what();
			char msg[100]={0x00};
			sprintf(msg,"error: error while streaming %s", ex.what());
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		 }
	 }
	 vipl_printf("info: downgrader tx streaming stopped!!", error_lvl, __FILE__, __LINE__);
}

int8_t downgrader::set_tx_freq(uhd::usrp::multi_usrp::sptr usrp, double freq, int8_t channel){
	uhd::tune_request_t tune_request(freq, 0.00);
	try{
		usrp->set_tx_freq(tune_request,channel);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: current rx freq set to %fMHz db %d mBoard %d", usrp->get_rx_freq(channel), channel);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
	}catch (const std::exception& ex) {
		//auto s = ex.what();
		char msg[100]={0x00};
		sprintf(msg,"error: error while changing frequency of USRPs %s", ex.what());
		vipl_printf(msg, error_lvl, __FILE__, __LINE__);
	 }
}

int8_t downgrader::set_rx_freq(uhd::usrp::multi_usrp::sptr usrp, uint8_t channel, double freq){
	uhd::tune_request_t tune_request(freq, 0.00);
	try{
		usrp->set_rx_freq(tune_request,channel);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: current tx freq set to %fMHz db %d", usrp->get_tx_freq(channel), channel);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
	}catch (const std::exception& ex) {
		//auto s = ex.what();
		char msg[100]={0x00};
		sprintf(msg,"error: error while changing frequency of USRPs %s", ex.what());
		vipl_printf(msg, error_lvl, __FILE__, __LINE__);
	 }
}

int8_t downgrader::set_rx_rate(uhd::usrp::multi_usrp::sptr usrp, uint8_t channel, double samp_rate) {
	try{
		usrp->set_rx_rate(samp_rate,channel);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: current rx rate set to %fmsps db %d", usrp->get_tx_freq(channel), channel);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
	}catch (const std::exception& ex) {
		//auto s = ex.what();
		char msg[100]={0x00};
		sprintf(msg,"error: error while changing sample rate of USRPs %s", ex.what());
		vipl_printf(msg, error_lvl, __FILE__, __LINE__);
	}
}
int8_t downgrader::set_tx_rate(uhd::usrp::multi_usrp::sptr usrp, uint8_t channel, double samp_rate) {
	try{
		usrp->set_tx_rate(samp_rate,channel);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: current tx rate set to %fmsps db %d", usrp->get_tx_freq(channel), channel);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
	}catch (const std::exception& ex) {
		//auto s = ex.what();
		char msg[100]={0x00};
		sprintf(msg,"error: error while changing sample rate of USRPs %s", ex.what());
		vipl_printf(msg, error_lvl, __FILE__, __LINE__);
	}
}

int8_t downgrader::set_rx_gain(uhd::usrp::multi_usrp::sptr usrp, uint8_t channel, double samp_rate){
	try{
		usrp->set_rx_rate(samp_rate,channel);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: current rx rate set to %f db %d", usrp->get_rx_gain(channel), channel);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
	}catch (const std::exception& ex) {
		//auto s = ex.what();
		char msg[100]={0x00};
		sprintf(msg,"error: error while changing sample rate of USRPs %s", ex.what());
		vipl_printf(msg, error_lvl, __FILE__, __LINE__);
	}
}

int8_t downgrader::set_tx_gain(uhd::usrp::multi_usrp::sptr usrp, uint8_t channel, double samp_rate){
	try{
		usrp->set_tx_rate(samp_rate,channel);
		{
			char msg[100]={0x00};
			sprintf(msg,"info: current rx rate set to %f db %d", usrp->get_rx_gain(channel), channel);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
	}catch (const std::exception& ex) {
		//auto s = ex.what();
		char msg[100]={0x00};
		sprintf(msg,"error: error while changing sample rate of USRPs %s", ex.what());
		vipl_printf(msg, error_lvl, __FILE__, __LINE__);
	}
}


bool downgrader::lock_gps(uhd::usrp::multi_usrp::sptr usrp, int32_t mBoard) {
	char buff[200]={0x00};
	usrp->set_clock_source("gpsdo",mBoard);
	std::string rtnval_clock = usrp->get_clock_source(mBoard);
	if(rtnval_clock.compare("gpsdo")){
		memset(buff, 0x00, sizeof(char)*100);
		sprintf(buff,"error: reference not set to GPSDO, channel %d", mBoard);
		goto error;
	}else{
		memset(buff, 0x00, sizeof(char)*100);
		sprintf(buff,"info: reference set to GPSDO, channel %d",mBoard);
		vipl_printf(buff, error_lvl, __FILE__, __LINE__);
	}
	usrp->set_time_source("gpsdo",mBoard);
	std::string rtnval_time = usrp->get_clock_source(mBoard);
	if(rtnval_time.compare("gpsdo")){
		memset(buff, 0x00, sizeof(char)*100);
		sprintf(buff,"error: reference not set to GPSDO, channel %d", mBoard);
		goto error;
	}else {
		memset(buff, 0x00, sizeof(char)*100);
		sprintf(buff,"info: reference set to GPSDO, channel %d", mBoard);
		vipl_printf(buff, error_lvl, __FILE__, __LINE__);
	}
	std::vector<std::string> sensor_names = usrp->get_mboard_sensor_names(mBoard);
	if (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end()) {
		memset(buff, 0x00, sizeof(char)*100);
		sprintf(buff,"info:waiting for reference lock...",error_lvl, __FILE__, __LINE__);
	    bool ref_locked = false;
	    for (int i = 0; i < 30 and not ref_locked; i++) {
	    	ref_locked = usrp->get_mboard_sensor("ref_locked", mBoard).to_bool();
	        if (not ref_locked) {
	        	sleep(1);
	        }
	    }
	    if (ref_locked) {
	    	memset(buff, 0x00, sizeof(char)*100);
	    	sprintf(buff,"info: locked GPSDO 10 MHz Reference",error_lvl, __FILE__,__LINE__);
	    	vipl_printf(buff, error_lvl, __FILE__, __LINE__);
	    } else {
	    	memset(buff, 0x00, sizeof(char)*100);
	    	sprintf(buff,"error: failed to lock GPSDO 10 MHz Reference",error_lvl, __FILE__,__LINE__);
	    	goto error;
	    }
	}else{
		memset(buff, 0x00, sizeof(char)*100);
		sprintf(buff,"error: ref_locked sensor not present on this board",error_lvl, __FILE__,__LINE__);
		goto error;
	}
	size_t num_gps_locked = 0x00;
	bool gps_locked = usrp->get_mboard_sensor("gps_locked", mBoard).to_bool();
	if(gps_locked){
	  num_gps_locked++;
	  memset(buff, 0x00, sizeof(char)*100);
	  sprintf(buff,"info: GPS locked",error_lvl, __FILE__,__LINE__);
	  vipl_printf(buff, error_lvl, __FILE__, __LINE__);
	}else{
		memset(buff, 0x00, sizeof(char)*100);
		sprintf(buff,"warning: GPS not locked - time will not be accurate until locked",error_lvl, __FILE__,__LINE__);
		vipl_printf(buff, error_lvl, __FILE__, __LINE__);
	}

	/*
	 *  Set to GPS time
	 */

	uhd::time_spec_t gps_time = uhd::time_spec_t( int64_t(usrp->get_mboard_sensor("gps_time", mBoard).to_int()));
	usrp->set_time_next_pps(gps_time + 1.0, mBoard);
	if (num_gps_locked == mBoard and mBoard > 1) {

	/*
	 *  Check to see if all USRP times are aligned
	 *  First, wait for PPS.
	 */

	uhd::time_spec_t time_last_pps = usrp->get_time_last_pps();
	while (time_last_pps == usrp->get_time_last_pps()) {
		usleep(1000);
	}

	/*
	 * Sleep a little to make sure all devices have seen a PPS edge
	 */

	usleep(200000);

	// Compare times across all mboards
	bool all_matched = true;
	uhd::time_spec_t mboard0_time = usrp->get_time_last_pps(0);
	for (size_t mboard = 1; mboard < mBoard; mboard++) {
		uhd::time_spec_t mboard_time = usrp->get_time_last_pps(mBoard);
	    if (mboard_time != mboard0_time) {
	      all_matched = false;
	      memset(buff,0x00, sizeof(char)*100);
	      sprintf(buff, "error: times are not aligned: USRP 0=%0.9f, USRP %d=%0.9f", mboard0_time.get_real_secs(), mboard, mboard_time.get_real_secs());
	      goto error;
	    }
	 }
	 if (all_matched) {
		 vipl_printf("info: USRP times aligned", error_lvl, __FILE__, __LINE__);
	 } else {
		 vipl_printf("error: USRP times are not aligned", error_lvl, __FILE__, __LINE__);
	 }
	}
	return true;
	error:
	vipl_printf(buff, error_lvl, __FILE__, __LINE__);
	return false;
}

