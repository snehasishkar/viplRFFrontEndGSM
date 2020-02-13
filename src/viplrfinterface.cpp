/*
 * viplrfinterface.cpp
 *
 *  Created on: 01-Apr-2019
 *  Author: Snehasish Kar
 */

#include <iostream>
#include <chrono>
#include <complex>
#include <string>
#include <boost/thread.hpp>
#include <sys/time.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <semaphore.h>
#include <stdlib.h>
#include <uhd/convert.hpp>

#include "../include/nmea.h"
#include "../include/nmea/gpgga.h"
#include "../include/viplrfinterface.hpp"
#include "../include/vipl_printf.hpp"
#include "../include/tcpserver.hpp"

struct vipl_rf_tap rftap;

//boost::mutex mutex_lock;

bool ready = false, is_staturated = false, receiving_gps = false, receving_antenna_signal = false;
bool is_tuned = false;
double max_samps_supported_by_usrp = 0x00;

vipl_rf_interface::vipl_rf_interface(){
		no_of_channels  =  0x00;
		freq_rx_board_a = 0.00;
		freq_rx_board_b = 0.00;
		freq_tx_board_a = 0.00;
		freq_tx_board_b = 0.00;
		sample_rate = 0.00;
		bandwidth = 0.00;
		gain = 0x00;
		lo_offset = 0.00;
		channel = 0x00;
		mboardCount = 0x00;
		tuned = false;
}

#if 0
int8_t vipl_rf_interface::set_tx_freq(double freq, int8_t channel){
	char buff[200]={0x00};
	lo_offset = freq;
	uhd::tune_request_t tune_request(freq, lo_offset);
	usrp->set_tx_freq(tune_request, channel);
	if(usrp->get_tx_freq(channel)!=freq){
		memset(buff, 0x00, sizeof(char)*100);
		sprintf(buff,"error: unable to set tx frequency %f, channel %d", freq, channel);
		goto error;
	}
	return 0x00;
	error:
	vipl_printf(buff, error_lvl, __FILE__, __LINE__);
	return 0x01;
}
#endif

int8_t vipl_rf_interface::set_rx_freq(uhd::usrp::multi_usrp::sptr usrp, uint8_t channel, double freq){
	char buff[200]={0x00};
	lo_offset = 0.00;
	uhd::tune_request_t tune_request(freq, lo_offset);
	usrp->set_rx_freq(tune_request, channel);
	//uhd::tune_request_t tune_request_uplink(freq-45e6, lo_offset);
	//usrp->set_rx_freq(tune_request_uplink, channel+1);
	memset(buff, 0x00, sizeof(char)*100);
	sprintf(buff,"info: USRP DL frequency set to %fMHz and UL frequency to %fMHz for channel %d and %d", usrp->get_rx_freq(channel), usrp->get_rx_freq(channel/*+1*/), channel, channel+1);
	vipl_printf(buff, error_lvl, __FILE__, __LINE__);
	return 0x00;
}

int8_t vipl_rf_interface::set_rx_rate(uhd::usrp::multi_usrp::sptr usrp, uint8_t channel, double samp_rate){
	char buff[200]={0x00};
	usrp->set_rx_rate(samp_rate, channel);
	memset(buff, 0x00, sizeof(char)*100);
	sprintf(buff,"info: sample rate set to %f, channel %d motherboard",usrp->get_rx_rate(channel), channel);
	vipl_printf(buff, error_lvl, __FILE__, __LINE__);
	return 0x00;
	error:
	vipl_printf(buff, error_lvl, __FILE__, __LINE__);
	return 0x01;
}

int8_t vipl_rf_interface::set_gain(uhd::usrp::multi_usrp::sptr usrp, double gain, int8_t channel){
	char buff[200]={0x00};
	usrp->set_rx_gain(gain, channel);
	memset(buff, 0x00, sizeof(char)*100);
	sprintf(buff,"info: gain set to %f, channel %d",usrp->get_rx_gain(channel), channel);
	vipl_printf(buff, error_lvl, __FILE__, __LINE__);
	return 0x00;
	error:
	vipl_printf(buff, error_lvl, __FILE__, __LINE__);
	return 0x01;
}
#if 0
void vipl_rf_interface::rfnoc_start_streaming(uhd::device3::sptr usrp_rfnoc,  cbuffercf *cb, unsigned long long total_no_samps, struct RFNOC_Config rfnoc_config){
	std::vector<std::string> blocks;
	/*
	 * create a receive streamer
	 */
	std::vector<std::complex<float>> buffer(rfnoc_config.spp);
	std::string streamargs;
	uhd::device_addr_t streamer_args(streamargs);
	switch(rfnoc_config.chain_no){
	case 0: streamer_args["block_id0"] = rfnoc_config.ddc_ctrl[1]->get_block_id();
			streamer_args["block_port0"] = "0";
			break;
	case 1: streamer_args["block_id1"] = rfnoc_config.ddc_ctrl[1]->get_block_id();
			streamer_args["block_port1"] = "1";
			break;
	case 2: streamer_args["block_id2"] = rfnoc_config.ddc_ctrl[2]->get_block_id();
			streamer_args["block_port2"] = "0";
			break;
	case 3: streamer_args["block_id3"] = rfnoc_config.ddc_ctrl[2]->get_block_id();
			streamer_args["block_port3"] = "1";
			break;
	case 4: streamer_args["block_id4"] = rfnoc_config.ddc_ctrl[4]->get_block_id();
			streamer_args["block_port4"] = "0";
			break;
	case 5: streamer_args["block_id5"] = rfnoc_config.ddc_ctrl[4]->get_block_id();
			streamer_args["block_port5"] = "1";
			break;
	case 6: streamer_args["block_id6"] = rfnoc_config.ddc_ctrl[5]->get_block_id();
			streamer_args["block_port6"] = "0";
			break;
	case 7: streamer_args["block_id7"] = rfnoc_config.ddc_ctrl[5]->get_block_id();
			streamer_args["block_port7"] = "1";
			break;
	}
	uhd::stream_args_t stream_args("fc32","sc16");
	stream_args.args = streamer_args;
	stream_args.args["spp"] = boost::lexical_cast<std::string>(rfnoc_config.spp);
	unsigned long long total_samps_recvd =0x00;
	#if 0
		for(int32_t i = 0x00; i< usrp->get_rx_num_channels(); i++){
			channel_nums.push_back(i);
		}
	#endif
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
	uint32_t row_count =0;
	while(!stop_rx and ((total_samps_recvd!=total_no_samps) or (total_no_samps==0x00))) {
		buffer.clear();
		size_t num_rx_samps = rx_stream->recv(&buffer.front(), rfnoc_config.spp, md, 3.0);
		if(error_lvl==3) {
			fprintf(stderr,"\n========================================================\n");
			fprintf(stderr,"  Number of samples received %u db %d \n",num_rx_samps, rfnoc_config.chain_no);
			fprintf(stderr,"\n========================================================\n");
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
	#if 0
		for(uint8_t i= 0;i <usrp->get_rx_num_channels();i++)
			circular_buffer[i]->set_eof();
	#endif
	vipl_printf("warning: stopping streaming mode", error_lvl, __FILE__, __LINE__);
	uhd::stream_cmd_t stream_cmd_stop(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
	rx_stream->issue_stream_cmd(stream_cmd_stop);
	constexpr double timeout { 0.010 }; //10ms
	static std::complex<float> dummy_buffer[50000000];
	static uhd::rx_metadata_t dummy_meta { };
	while (rx_stream->recv(dummy_buffer, 50000000, dummy_meta, timeout)) {}
	rx_stream.reset();
}
void vipl_rf_interface::rfnoc_setup(uhd::device3::sptr usrp_rfnoc, int32_t mBoard, struct GSMConfig config, struct RFNOC_Config rfnoc_config){
	rftap_sync.lock();
	rftap.gain = DEFAULT_GAIN;
	rftap_sync.unlock();
	rfnoc_config.radio_ctrl[rfnoc_config.db_no]->set_rx_frequency(config.dl_freq_a, rfnoc_config.radio_channel_no);
	rfnoc_config.radio_ctrl[rfnoc_config.db_no]->set_rx_bandwidth(config.bandwidth, rfnoc_config.radio_channel_no);
	rfnoc_config.radio_ctrl[rfnoc_config.db_no]->set_rx_gain(DEFAULT_GAIN, rfnoc_config.radio_channel_no);
	switch(rfnoc_config.db_no){
	case 0: switch(rfnoc_config.radio_channel_no){
			case 0: rfnoc_config.radio_ctrl[rfnoc_config.db_no]->set_rx_antenna("RX1",rfnoc_config.radio_channel_no);
					rfnoc_config.ddc_ctrl[0x00]->set_arg<double>("freq", rfnoc_config.primary_ddc_freq, rfnoc_config.radio_channel_no);
					rfnoc_config.ddc_ctrl[0x00]->set_arg<double>("output_rate", rfnoc_config.primary_ddc_decim_rate, rfnoc_config.radio_channel_no);
					break;
			case 1: rfnoc_config.radio_ctrl[rfnoc_config.db_no]->set_rx_antenna("RX2",rfnoc_config.radio_channel_no);
					rfnoc_config.ddc_ctrl[0x00]->set_arg<double>("freq", rfnoc_config.primary_ddc_freq, rfnoc_config.radio_channel_no);
					rfnoc_config.ddc_ctrl[0x00]->set_arg<double>("output_rate", rfnoc_config.primary_ddc_decim_rate, rfnoc_config.radio_channel_no);
					break;
			}
			break;
	case 1: switch(rfnoc_config.radio_channel_no){
				case 0: rfnoc_config.radio_ctrl[rfnoc_config.db_no]->set_rx_antenna("RX1",rfnoc_config.radio_channel_no);
				rfnoc_config.ddc_ctrl[0x03]->set_arg<double>("freq", rfnoc_config.primary_ddc_freq, rfnoc_config.radio_channel_no);
				rfnoc_config.ddc_ctrl[0x03]->set_arg<double>("output_rate", rfnoc_config.primary_ddc_decim_rate, rfnoc_config.radio_channel_no);
				break;
				case 1: rfnoc_config.radio_ctrl[rfnoc_config.db_no]->set_rx_antenna("RX2",rfnoc_config.radio_channel_no);
				rfnoc_config.ddc_ctrl[0x03]->set_arg<double>("freq", rfnoc_config.primary_ddc_freq, rfnoc_config.radio_channel_no);
				rfnoc_config.ddc_ctrl[0x03]->set_arg<double>("output_rate", rfnoc_config.primary_ddc_decim_rate, rfnoc_config.radio_channel_no);
				break;
			}
			break;
	}
	switch(rfnoc_config.chain_no) {
	case 0:	rfnoc_config.ddc_ctrl[0x01]->set_arg<double>("freq", rfnoc_config.secondary_ddc_freq, 0x00);
			rfnoc_config.ddc_ctrl[0x01]->set_arg<double>("output_rate", rfnoc_config.secondary_ddc_decim_rate, 0x00);
			break;
	case 1: rfnoc_config.ddc_ctrl[0x02]->set_arg<double>("freq", rfnoc_config.secondary_ddc_freq, 0x01);
			rfnoc_config.ddc_ctrl[0x02]->set_arg<double>("output_rate", rfnoc_config.secondary_ddc_decim_rate, 0x01);
			break;
	case 2: rfnoc_config.ddc_ctrl[0x04]->set_arg<double>("freq", rfnoc_config.secondary_ddc_freq, 0x00);
			rfnoc_config.ddc_ctrl[0x04]->set_arg<double>("output_rate", rfnoc_config.secondary_ddc_decim_rate, 0x00);
			break;
	case 3: rfnoc_config.ddc_ctrl[0x05]->set_arg<double>("freq", rfnoc_config.secondary_ddc_freq, 0x01);
			rfnoc_config.ddc_ctrl[0x05]->set_arg<double>("output_rate", rfnoc_config.secondary_ddc_decim_rate, 0x01);
			break;
	}
}

void vipl_rf_interface::rfnoc_conn_setup(uhd::device3::sptr usrp_rfnoc, struct RFNOC_Config rfnoc_config){
	for(int32_t i=0;i<NUM_OF_RFNOC_RADIO_BLOCKS;i++){
		uhd::rfnoc::block_id_t radio_ctrl_id(0, "Radio", i);
		uhd::rfnoc::block_id_t splitstream_id(0,"splitstream", i);
		if(usrp_rfnoc->has_block(radio_ctrl_id))
			rfnoc_config.radio_ctrl[i] = usrp_rfnoc->get_block_ctrl<uhd::rfnoc::radio_ctrl>(radio_ctrl_id);
		else
			vipl_printf("error: rfnoc radio not found in the fpga build!!", error_lvl, __FILE__, __LINE__);
		if(usrp_rfnoc->has_block(splitstream_id))
			rfnoc_config.splitstream_ctrl[i] = usrp_rfnoc->get_block_ctrl<uhd::rfnoc::source_block_ctrl_base>(splitstream_id);
	}
	for(int32_t i=0;i<NUM_OF_RFNOC_DDC_BLOCKS;i++){
		uhd::rfnoc::block_id_t ddc_ctrl_id(0, "DDC", i);
		rfnoc_config.ddc_ctrl[i] = usrp_rfnoc->get_block_ctrl<uhd::rfnoc::ddc_block_ctrl>(ddc_ctrl_id);
	}
	uhd::rfnoc::graph::sptr rx_graph = usrp_rfnoc->create_graph("GSM_RF_DUPLEX_CHAIN");

	//First block chain
	rx_graph->connect(rfnoc_config.radio_ctrl[0]->get_block_id(),0,rfnoc_config.ddc_ctrl[0]->get_block_id(),0);
	rx_graph->connect(rfnoc_config.radio_ctrl[0]->get_block_id(),1,rfnoc_config.ddc_ctrl[0]->get_block_id(),1);
	rx_graph->connect(rfnoc_config.ddc_ctrl[0]->get_block_id(),0,rfnoc_config.splitstream_ctrl[0]->get_block_id(),0);
	rx_graph->connect(rfnoc_config.splitstream_ctrl[0]->get_block_id(),0,rfnoc_config.ddc_ctrl[1]->get_block_id(),0);
	rx_graph->connect(rfnoc_config.splitstream_ctrl[0]->get_block_id(),1,rfnoc_config.ddc_ctrl[1]->get_block_id(),1);
	rx_graph->connect(rfnoc_config.ddc_ctrl[0]->get_block_id(),1,rfnoc_config.splitstream_ctrl[1]->get_block_id(),0);
	rx_graph->connect(rfnoc_config.splitstream_ctrl[1]->get_block_id(),0,rfnoc_config.ddc_ctrl[2]->get_block_id(),0);
	rx_graph->connect(rfnoc_config.splitstream_ctrl[1]->get_block_id(),1,rfnoc_config.ddc_ctrl[2]->get_block_id(),1);

	rx_graph->connect(rfnoc_config.radio_ctrl[1]->get_block_id(),0,rfnoc_config.ddc_ctrl[3]->get_block_id(),0);
	rx_graph->connect(rfnoc_config.radio_ctrl[1]->get_block_id(),1,rfnoc_config.ddc_ctrl[3]->get_block_id(),1);
	rx_graph->connect(rfnoc_config.ddc_ctrl[3]->get_block_id(),0,rfnoc_config.splitstream_ctrl[2]->get_block_id(),0);
	rx_graph->connect(rfnoc_config.splitstream_ctrl[2]->get_block_id(),0,rfnoc_config.ddc_ctrl[4]->get_block_id(),0);
	rx_graph->connect(rfnoc_config.splitstream_ctrl[2]->get_block_id(),1,rfnoc_config.ddc_ctrl[4]->get_block_id(),1);
	rx_graph->connect(rfnoc_config.ddc_ctrl[3]->get_block_id(),1,rfnoc_config.splitstream_ctrl[3]->get_block_id(),0);
	rx_graph->connect(rfnoc_config.splitstream_ctrl[3]->get_block_id(),0,rfnoc_config.ddc_ctrl[5]->get_block_id(),0);
	rx_graph->connect(rfnoc_config.splitstream_ctrl[3]->get_block_id(),0,rfnoc_config.ddc_ctrl[5]->get_block_id(),1);
}
#endif

void vipl_rf_interface::get_gps_val(uhd::usrp::multi_usrp::sptr usrp, bool tuned){
	while(!stop_gps){
		int counter =0x00;
		const char s[2] = ",";
		char *token,*nmeaData;
		float latitude, longitude;
		uhd::sensor_value_t gga_string = usrp->get_mboard_sensor("gps_gpgga");
		nmeaData = (char *)malloc(gga_string.to_pp_string().size());
		memset(nmeaData,'\0',sizeof(nmeaData));
		//std::cout<< boost::format("%s")%gga_string.to_pp_string()<<std::endl;
		memcpy(nmeaData,gga_string.to_pp_string().c_str(),gga_string.to_pp_string().size());
		token = strtok(nmeaData,s);
		while(token!=NULL){
			if(counter == 2){
			 	 latitude = atof(token);
			 	 //latitude = convertFromNmeaSentenceToDecimalCoord(latitude);
			 	 {
			 		int32_t degree;  //to store the degrees
			 		float decimal; //to store de decimal
			 		degree = latitude/100; // 51 degrees
			 		decimal = (latitude/100 - degree)*100 ; //(51.536605 - 51)* 100 = 53.6605
			 		decimal /= 60; // 53.6605 / 60 = 0.8943417
			 		decimal += degree; // 0.8943417 + 51 = 51.8943417
			 		latitude = decimal;
			 	 }
			}
			if(counter == 4){
			 	 longitude = atof(token);
			 	 //longitude = convertFromNmeaSentenceToDecimalCoord(longitude);
			 	 {
			 		 int32_t degree;  //to store the degrees
			 		 float decimal; //to store de decimal
			 		 degree = longitude/100; // 51 degrees
			 		 decimal = (longitude/100 - degree)*100 ; //(51.536605 - 51)* 100 = 53.6605
			 		 decimal /= 60; // 53.6605 / 60 = 0.8943417
			 		 decimal += degree; // 0.8943417 + 51 = 51.8943417
			 		 longitude = decimal;
			 	 }
			 }
			 token = strtok(NULL,s);
			 counter++;
		}
		rftap.is_antenna_connected = receving_antenna_signal;
		rftap.is_saturating = is_staturated;
		if((latitude != 0x00) && (longitude != 0x00)){
			rftap_sync.lock();
			rftap.latitude = latitude;
			rftap.longitude = longitude;
			rftap.is_receiving_gps = true;
			rftap_sync.unlock();
			if(error_lvl==3){
				fprintf(stderr,"\n=================================================================================\n");
				fprintf(stderr,"  Latitude: %lf Longitude: %lf \n", rftap.latitude, rftap.longitude);
				fprintf(stderr,"\n==================================================================================\n");
			}
		}else{
			rftap_sync.lock();
			rftap.latitude = 0.00;
			rftap.longitude = 0.00;
			rftap.is_receiving_gps = false;
			rftap_sync.unlock();
			if(error_lvl==3)
				vipl_printf("warning: GPS Fix lost", error_lvl, __FILE__, __LINE__);
		}
		free(nmeaData);
		if(!is_tuned){
			char filename[100]={0x00};
			sprintf(filename, "%s/gpsval.bin",ntwrkscan);
			FILE *fp = fopen(filename, "w+");
			fwrite(&rftap, sizeof(rftap), 1, fp);
			fclose(fp);
		}

		sleep(GPS_WAIT_TIME_IN_SECONDS);
	}
	if(error_lvl==3)
		vipl_printf("info: GPS Stopped", error_lvl, __FILE__, __LINE__);
}

bool vipl_rf_interface::lock_gps(uhd::usrp::multi_usrp::sptr usrp, int32_t mBoard) {
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
	usrp->set_time_source("gpsdo",config.mBoard);
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
	//usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
	fprintf(stderr,"info: waiting for reference lock...");
	std::vector<std::string> sensor_names = usrp->get_mboard_sensor_names(mBoard);
	if (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end()) {
		memset(buff, 0x00, sizeof(char)*100);
		fprintf(stderr,"+");
		bool ref_locked = false;
	    for (int i = 0; i < 30 and not ref_locked; i++) {
	    	ref_locked = usrp->get_mboard_sensor("ref_locked", mBoard).to_bool();
	        if (not ref_locked) {
	        	sleep(1);
	        }
	    }
	    if (ref_locked) {
	    	fprintf(stderr,"\n");
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
	gps_time = uhd::time_spec_t(time_t(usrp->get_mboard_sensor("gps_time", mBoard).to_int()));
	uhd::time_spec_t time_last_pps = usrp->get_time_last_pps(mBoard);
	std::cout << "USRP time: " << (boost::format("%0.9f") % time_last_pps.get_real_secs()) << std::endl;
	std::cout << "GPSDO time: " << (boost::format("%0.9f") % gps_time.get_real_secs()) << std::endl;
	if (gps_time.get_real_secs() == time_last_pps.get_real_secs())
		std::cout << std::endl << "SUCCESS: USRP time synchronized to GPS time" << std::endl << std::endl;
	else
	    std::cerr << std::endl << "ERROR: Failed to synchronize USRP time to GPS time" << std::endl << std::endl;
	if (num_gps_locked == mBoard+1 and mBoard+1 > 0) {

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
	for (size_t counter = 0; counter < mBoard+1; counter++) {
		uhd::time_spec_t mboard_time = usrp->get_time_last_pps(mBoard);
	    if (mboard_time != mboard0_time) {
	      all_matched = false;
	      memset(buff,0x00, sizeof(char)*100);
	      sprintf(buff, "error: times are not aligned: USRP 0=%0.9f, USRP %d=%0.9f", mboard0_time.get_real_secs(), mboard_time.get_real_secs());
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
void vipl_rf_interface::setup(uhd::usrp::multi_usrp::sptr usrp, int32_t mBoard, struct GSMConfig config_obj, int32_t mode, int8_t chain_num) {
try{
	for(uint8_t i = chain_num; i<chain_num+1/*usrp->get_rx_num_channels()*/; i++) {
			rftap_sync.lock();
			rftap.gain = DEFAULT_GAIN_DWNLINK;
			rftap_sync.unlock();
			usrp->set_rx_gain(DEFAULT_GAIN_DWNLINK, i);
			{
				char msg[100]={0x00};
				sprintf(msg,"info: current gain set to %fdb to channel %d db %d mBoard %d", usrp->get_rx_gain(i), i, i/2, mBoard);
				vipl_printf(msg, error_lvl, __FILE__, __LINE__);
			}
			usrp->set_rx_bandwidth(config_obj.bandwidth, i);
			{
				char msg[100]={0x00};
				sprintf(msg,"info: current bandwidth set to %fMHz to channel %d db %d mBoard %d", usrp->get_rx_bandwidth(i), i, i/2, mBoard);
				vipl_printf(msg, error_lvl, __FILE__, __LINE__);
			}
			usrp->set_rx_rate(config_obj.sampleRate, i);
			{
				char msg[100]={0x00};
				sprintf(msg,"info: Sample Rate set to %f to channel %d db %d mBoard %d", usrp->get_rx_rate(i), i, i/2, mBoard);
				vipl_printf(msg, error_lvl, __FILE__, __LINE__);
			}
			usrp->set_rx_dc_offset(true, i);
			usrp->set_rx_iq_balance(true, i);
			if(mode==MODE_SIMPLEX){
			uhd::tune_request_t tune_request(config.dl_freq_a, lo_offset);
			switch(i) {
				case 0: usrp->set_rx_antenna("RX1", i);
				{
					char msg[100]={0x00};
					sprintf(msg,"info: antenna set to %s to channel %d db %d mBoard %d", usrp->get_rx_antenna(i).c_str(), i, i/2, mBoard);
					vipl_printf(msg, error_lvl, __FILE__, __LINE__);
				}
				usrp->set_rx_freq(tune_request, i);
				{
					char msg[100]={0x00};
					sprintf(msg,"info: Frequency set to %FMHz to channel %d db %d mBoard %d", usrp->get_rx_freq(i), i, i/2, mBoard);
					vipl_printf(msg, error_lvl, __FILE__, __LINE__);
				}
				break;
			case 1: usrp->set_rx_antenna("RX2", i);
				{
					char msg[100]={0x00};
					sprintf(msg,"info: antenna set to %s to channel %d db %d mBoard %d", usrp->get_rx_antenna(i).c_str(), i, i/2, mBoard);
					vipl_printf(msg, error_lvl, __FILE__, __LINE__);
				}
				usrp->set_rx_freq(tune_request, i);
				{
					char msg[100]={0x00};
					sprintf(msg,"info: Frequency set to %FMHz to channel %d db %d mBoard %d", usrp->get_rx_freq(i), i, i/2, mBoard);
					vipl_printf(msg, error_lvl, __FILE__, __LINE__);
				}
				break;
			case 2: usrp->set_rx_antenna("RX1", i);
				{
					char msg[100]={0x00};
					sprintf(msg,"info: antenna set to %s to channel %d db %d mBoard %d", usrp->get_rx_antenna(i).c_str(), i, i/2, mBoard);
					vipl_printf(msg, error_lvl, __FILE__, __LINE__);
				}
				usrp->set_rx_freq(tune_request, i);
				{
					char msg[100]={0x00};
					sprintf(msg,"info: Frequency set to %FMHz to channel %d db %d mBoard %d", usrp->get_rx_freq(i), i, i/2, mBoard);
					vipl_printf(msg, error_lvl, __FILE__, __LINE__);
				}
				break;
			case 3: usrp->set_rx_antenna("RX2", i);
				{
					char msg[100]={0x00};
					sprintf(msg,"info: antenna set to %s to channel %d db %d mBoard %d", usrp->get_rx_antenna(i).c_str(), i, i/2, mBoard);
					vipl_printf(msg, error_lvl, __FILE__, __LINE__);
				}
				usrp->set_rx_freq(tune_request, i);
				{
					char msg[100]={0x00};
					sprintf(msg,"info: Frequency set to %FMHz to channel %d db %d mBoard %d", usrp->get_rx_freq(i), i, i/2, mBoard);
					vipl_printf(msg, error_lvl, __FILE__, __LINE__);
				}
				break;
			}
		}else {
			uhd::tune_request_t tune_request(config.dl_freq_a, lo_offset);
			switch(i) {
			case 0: usrp->set_rx_antenna("RX1", i);
					{
						char msg[100]={0x00};
						sprintf(msg,"info: antenna set to %s to channel %d db %d mBoard %d", usrp->get_rx_antenna(i).c_str(), i, i/2, mBoard);
						vipl_printf(msg, error_lvl, __FILE__, __LINE__);
					}
					usrp->set_rx_freq(tune_request, i);
					{
						char msg[100]={0x00};
						sprintf(msg,"info: Frequency set to %FMHz to channel %d db %d mBoard %d", usrp->get_rx_freq(i), i, i/2, mBoard);
						vipl_printf(msg, error_lvl, __FILE__, __LINE__);
					}
					break;
			case 1: usrp->set_rx_antenna("RX2", i);
					{
						char msg[100]={0x00};
						sprintf(msg,"info: antenna set to %s to channel %d db %d mBoard %d", usrp->get_rx_antenna(i).c_str(), i, i/2, mBoard);
						vipl_printf(msg, error_lvl, __FILE__, __LINE__);
					}
					usrp->set_rx_freq(tune_request, i);
					{
						char msg[100]={0x00};
						sprintf(msg,"info: Frequency set to %FMHz to channel %d db %d mBoard %d", usrp->get_rx_freq(i), i, i/2, mBoard);
						vipl_printf(msg, error_lvl, __FILE__, __LINE__);
					}
					break;
			 case 2: usrp->set_rx_antenna("RX1", i);
					{
						char msg[100]={0x00};
						sprintf(msg,"info: antenna set to %s to channel %d db %d mBoard %d", usrp->get_rx_antenna(i).c_str(), i, i/2, mBoard);
						vipl_printf(msg, error_lvl, __FILE__, __LINE__);
					}
					usrp->set_rx_freq(tune_request, i);
					{
						char msg[100]={0x00};
						sprintf(msg,"info: Frequency set to %FMHz to channel %d db %d mBoard %d", usrp->get_rx_freq(i), i, i/2, mBoard);
						vipl_printf(msg, error_lvl, __FILE__, __LINE__);
				    }
					break;
			case 3: usrp->set_rx_antenna("RX2", i);
					{
						char msg[100]={0x00};
						sprintf(msg,"info: antenna set to %s to channel %d db %d mBoard %d", usrp->get_rx_antenna(i).c_str(), i, i/2, mBoard);
						vipl_printf(msg, error_lvl, __FILE__, __LINE__);
					}
					usrp->set_rx_freq(tune_request, i);
					{
						char msg[100]={0x00};
						sprintf(msg,"info: Frequency set to %FMHz to channel %d db %d mBoard %d", usrp->get_rx_freq(i), i, i/2, mBoard);
						vipl_printf(msg, error_lvl, __FILE__, __LINE__);
					}
					break;
			}
		}
	}
 }catch(const std::exception& ex){
	 char msg[100]={0x00};
	 sprintf(msg,"error: %s",ex.what());
	 vipl_printf(msg,error_lvl, __FILE__, __LINE__);
	 usrp->clear_command_time(0);
	 usrp->~multi_usrp();
 }
}
#if 0
void vipl_rf_interface::setup(uhd::device3::sptr usrp, int32_t num_channel, int32_t mBoard_no, double *freq, double *samp_rate_downsampler, bool fft) {
	uhd::rfnoc::radio_ctrl::sptr radio_ctrl[num_channel];
	uhd::rfnoc::ddc_block_ctrl::sptr ddc_block_ctrl[num_channel];
	std::vector<std::string> flowgraph;
	usrp->clear();
	uhd::rfnoc::graph::sptr rx_graph = usrp->create_graph("GSM_RF_QUAD_CHAIN");
	for(uint8_t i = 0; i<num_channel; i++) {
		uhd::rfnoc::block_id_t radio_ctrl_id(mBoard_no, "Radio", i);
		if(!(usrp->has_block(radio_ctrl_id))){
			char msg[100]={0x00};
			sprintf(msg,"error: radio block not found in the mboard %d and dboard %d", mBoard_no, i);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
			exit (EXIT_FAILURE);
		}
		radio_ctrl[i] = usrp->get_block_ctrl< uhd::rfnoc::radio_ctrl >(radio_ctrl_id);
		flowgraph.push_back(radio_ctrl[i]->get_block_id());

		//Samplerate done, considering usrp 2955R;

		radio_ctrl[i]->set_rate(SAMPLE_RATE_RFNOC_RADIO_2955R);
		radio_ctrl[i]->set_rx_gain(DEFAULT_GAIN, i);
		radio_ctrl[i]->set_rx_bandwidth(samp_rate_downsampler[i], i);

		//Antenna and Frequency allignment done, considering usrp 2955R;
		//If any other USRP is being used changed the loop alignment; dont change entire function

		switch(i){
		case 0: radio_ctrl[i]->set_rx_frequency(freq[i], i);
				radio_ctrl[i]->set_rx_antenna("RX1",i);
				break;
		case 1: radio_ctrl[i]->set_rx_frequency(freq[i], i);
				radio_ctrl[i]->set_rx_antenna("RX2",i);
				break;
		case 2: radio_ctrl[i]->set_rx_frequency(freq[i], i);
				radio_ctrl[i]->set_rx_antenna("RX1",i);
				break;
		case 3: radio_ctrl[i]->set_rx_frequency(freq[i], i);
				radio_ctrl[i]->set_rx_antenna("RX2",i);
				break;
		}

		uhd::rfnoc::block_id_t ddc_block_ctrl_id(mBoard_no, "DDC", i);
		if(!(usrp->has_block(ddc_block_ctrl_id))){
			char msg[100]={0x00};
			sprintf(msg,"error: DDC block not found in the mboard %d and dboard %d", mBoard_no, i);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
			exit (EXIT_FAILURE);
		}
		ddc_block_ctrl[i] = usrp->get_block_ctrl(ddc_block_ctrl_id);
		flowgraph.push_back(ddc_block_ctrl[i]->get_block_id());
		ddc_block_ctrl[i]->set_arg("input_rate", (float) SAMPLE_RATE_RFNOC_RADIO_2955R , i);
		ddc_block_ctrl[i]->set_arg("output_rate", (float)  samp_rate_downsampler[i], i);

 		//if(error_lvl==3)

	}
}
#endif

void flush(int32_t _nchan, uhd::rx_streamer::sptr rxstream) {
    const size_t nbytes = 4096;
    std::vector<void *> outputs;
    uhd::rx_metadata_t _metadata;
    std::vector<std::vector<char>> buffs(_nchan, std::vector<char>(nbytes));
    for (size_t i = 0; i < _nchan; i++) {
        outputs.push_back(&buffs[i].front());
    }
    while (true) {
        const size_t bpi = uhd::convert::get_bytes_per_item("fc32");
        if (rxstream)
            // get the remaining samples out of the buffers
            rxstream->recv(outputs, nbytes / bpi, _metadata, 0.0);
        else
            // no rx streamer -- nothing to flush
            break;

        if (_metadata.error_code == ::uhd::rx_metadata_t::ERROR_CODE_TIMEOUT)
            break;
    }
}

void vipl_rf_interface::start_streaming(uhd::usrp::multi_usrp::sptr usrp, double samples_per_burst, int32_t channel_no, cbuffercf *cb, unsigned long long total_no_samps) {
	std::vector<std::string> blocks;

	    /*
	     * create a receive streamer
	     */
	uhd::stream_args_t stream_args("fc32","sc16");
	std::vector<size_t> channel_nums;
	std::vector<std::complex<float>> buffer(samples_per_burst);
	unsigned long long total_samps_recvd =0x00;
#if 0
	for(int32_t i = 0x00; i< usrp->get_rx_num_channels(); i++){
		channel_nums.push_back(i);
	}
#endif
	channel_nums.push_back(0);
	 if(!total_no_samps)
	channel_nums.push_back(1);
	stream_args.channels = channel_nums;
	uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);
	uhd::rx_metadata_t md;
	bool overflow_message = true;
	uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
	stream_cmd.num_samps  = 0x00;
	stream_cmd.stream_now = false;
	stream_cmd.time_spec =  usrp->get_time_now() + ::uhd::time_spec_t(0.1);//time_spec;
	rx_stream->issue_stream_cmd(stream_cmd);
	//sem_wait(&sem_status_stream);
	bool discard_first_set_of_samples = false;
	uint32_t row_count =0;
	int32_t n_chan =1;
	if(!total_no_samps)
		n_chan = 2;
	int32_t max_samps = samples_per_burst;
	std::vector<std::vector<std::complex<float>>> buffer_data(n_chan, std::vector<std::complex<float> >(max_samps));
	std::vector<void *> buffs(2);
	buffs[0] = &buffer_data[0].front();
	if(!total_no_samps)
		buffs[1] = &buffer_data[1].front();
	while(!stop_rx and ((total_samps_recvd!=total_no_samps) or (total_no_samps==0x00))) {
		buffer.clear();
		size_t num_rx_samps;
		try{
			num_rx_samps = rx_stream->recv(buffs, max_samps, md, 1.0);
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
		if(((cbuffercf_size(*cb)-max_samps)>=(cbuffercf_max_size(*cb))||(cbuffercf_size(*cb)-max_samps)==(0))&&(cbuffercf_size(*cb)!=0)) {
			cbuffercf_release(*cb,(uint32_t)(max_samps));
		}
		cbuffercf_write(*cb, &buffer_data[0].front(), max_samps);
		if(!total_no_samps)
			cbuffercf_write(cb4, &buffer_data[1].front(), max_samps);
		total_samps_recvd+=num_rx_samps;
		buffer_data[0].clear();
		buffer_data[1].clear();
#endif
	}
#if 0
	for(uint8_t i= 0;i <usrp->get_rx_num_channels();i++)
		circular_buffer[i]->set_eof();
#endif
	uhd::stream_cmd_t stream_cmd_stop(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
	rx_stream->issue_stream_cmd(stream_cmd_stop);
	constexpr double timeout { 0.010 }; //10ms
	static std::complex<float> dummy_buffer[50000000];
	static uhd::rx_metadata_t dummy_meta { };
	while (rx_stream->recv(dummy_buffer, 50000000, dummy_meta, timeout)) {}
	vipl_printf("warning: stopping streaming mode", error_lvl, __FILE__, __LINE__);
	//flush(1, rx_stream);
	//rx_stream.reset();
}

vipl_rf_interface::~vipl_rf_interface() {
	vipl_printf("info: ViplRFInterface Object closed", error_lvl, __FILE__, __LINE__);
}
