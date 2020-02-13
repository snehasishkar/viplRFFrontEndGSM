/*
 * downgraderdemod.cpp
 *
 *  Created on: 22-Jan-2020
 *      Author: Snehasish Kar
 */

#include <unistd.h>
#include <fcntl.h>

#include "../include/grgsm/misc_utils/msg_to_tag.h"
#include "../include/grgsm/misc_utils/controlled_rotator_cc.h"
#include "../include/grgsm/misc_utils/controlled_fractional_resampler_cc.h"
#include "../include/grgsm/demapping/universal_ctrl_chans_demapper.h"
#include "../include/grgsm/decoding/control_channels_decoder.h"
#include "../include/grgsm/receiver/receiver.h"
#include "../include/grgsm/receiver/clock_offset_control.h"

#include "../include/downgraderdemod.h"

cbuffercf cb_downgrader;

downgrader_demod::downgrader_demod() {
	// TODO Auto-generated constructor stub

}

downgrader_demod::~downgrader_demod() {
	// TODO Auto-generated destructor stub
}

void downgrader_demod::transmit_receive_burst(uhd::usrp::multi_usrp::sptr usrp, double samples_per_burst, char *fifo_name) {
	std::vector<size_t> channel_nums;
	uhd::stream_args_t stream_args("fc32", "sc16");
	channel_nums.push_back(0x00);
	uhd::tx_metadata_t md;
	md.start_of_burst = false;
	md.end_of_burst = false;
	std::complex<float> buffer[(int32_t)samples_per_burst];
	uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);
	int32_t num_samples_up = 0x00, total_no_samps = 0x00;
	int32_t fd = 0x00;
	label_file:
	fd = open(fifo_name, O_RDWR);
	if (fd<=0x00) {
		if(error_lvl==3){
			fprintf(stderr,"error: unable to open FIFO for downgrader!!\n");
		}
		usleep(100000);
		goto label_file;
	}else{
		fprintf(stderr,"debug: FIFO for downgrader successfully opened!!\n");
	}

	while(!stop_rx or not md.end_of_burst){
		label:
		total_no_samps = num_samples_up = 0x00;
		while((num_samples_up=read(fd, buffer, (sizeof(std::complex<float>)*samples_per_burst))-total_no_samps)>0x00){
			total_no_samps+= num_samples_up;
		}
		if(num_samples_up!=(uint32_t)samples_per_burst) {
			usleep(1000);
			goto label;
		}
		if(error_lvl==3){
			fprintf(stderr,"\n======================================================\n");
			fprintf(stderr,"Number of samples read from circular buffer A5/3 downgrader %d", num_samples_up);
			fprintf(stderr,"\n======================================================\n");
		}
		try {
			size_t num_samples_transmitted = tx_stream->send(buffer, num_samples_up, md);
			if(error_lvl==3){
				fprintf(stderr, "\n=================================================\n");
				fprintf(stderr, "Number of samples transmitted %d A5/3 downgrader", num_samples_transmitted);
				fprintf(stderr, "\n=================================================\n");
			}
		}
		catch(const std::exception& ex){
			char msg[100]={0x00};
			sprintf(msg, "error while transmitting %s",  ex.what());
		}
		memset(buffer, 0x00, sizeof(std::complex<float>)*samples_per_burst);
	}
	vipl_printf("info: A5/3 downgrader successfully stopped!!", error_lvl, __FILE__, __LINE__);
}

void downgrader_demod::init_downgrader(enum band_details band, int16_t arfcn, uhd::usrp::multi_usrp::sptr usrp, int32_t mBoard, double samples_per_burst, int8_t chain_num, int8_t num_chain){
	downgrader downgrader_obj;
	struct GSMConfig_downgrader config;
	cb_downgrader = cbuffercf_create(sizeof(std::complex<float>)*samples_per_burst);
	config.bandwidth = 0x00;
	config.sampleRate = 13e6/12;
	config.mBoard = mBoard;
	config.rx_gain = 20e6;
	config.tx_gain = 10e6;
	config.samples_per_burst = samples_per_burst;
	config.dl_freq_a = convert_arfcn_to_freq(band, arfcn, false);
	config.ul_freq_a = convert_arfcn_to_freq(band, arfcn, true);
	usrp->set_rx_subdev_spec(uhd::usrp::subdev_spec_t("A:B"), config.mBoard);
	usrp->set_tx_subdev_spec(uhd::usrp::subdev_spec_t("A:A"), config.mBoard);
	downgrader_obj.setup(usrp, mBoard, config, 0x00);
	bool gps_synced = downgrader_obj.lock_gps(usrp, config.mBoard);
	if(!gps_synced)
		vipl_printf("error: unable to synchronize USRP to GPS", error_lvl, __FILE__, __LINE__);
	else
		vipl_printf("info: successfully synchronized USRP to GPS", error_lvl, __FILE__, __LINE__);
	boost::thread t1(transmit_receive_burst, usrp,  config.samples_per_burst, "");
	vipl_printf("info: A5/3 downgrader successfully started!!", error_lvl, __FILE__, __LINE__);
}




