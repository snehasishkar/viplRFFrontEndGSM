/*
 *  tcpserver.h
 *
 *  Created on: 31-Jul-2019
 *  Author: Snehasish Kar
 */

#ifndef INCLUDE_TCPSERVER_HPP_
#define INCLUDE_TCPSERVER_HPP_

#include <inttypes.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <queue>
#include <semaphore.h>
#include <boost/thread.hpp>
#include "vipl_printf.hpp"

#define max_queue_size 64
#define DEFAULT_PROTOCOL_STACK_SERVER_IP "127.0.0.1"
#define DEFAULT_PROTOCOL_STACK_SERVER_PORT 1528


class tcp_server {
	int32_t serverSocket;
	int32_t clientSocket;
	struct sockaddr_in serv_addr, client_addr;
public:
	tcp_server(char *BIND_IP, int16_t BIND_PORT);
	virtual ~tcp_server();
	int32_t read_action(void);
	void write_action(uint8_t *writeBuffer);
};

struct GSM_PARAM_FROM_UI{
	int8_t hsn;
	int8_t maio;
	int8_t tseq;
	int8_t timeslot;
	int8_t subslot;
	char channel_type[7];
	int16_t hopping_list[16];
};

struct COMMAND_FROM_GUI{
	bool tune_request;
	bool tx_tune_request;
	bool tune_gps;
	bool perform_network_scan;
	bool stop_gps;
	bool untune;
	bool change_gain;
	int8_t mode;
	int16_t number_of_arfcn_to_scan;
	int16_t number_of_ip_to_connect;
	int16_t arfcn_list[124];
	int32_t gain;
	char band[20];
	char mboard_ip[16];
	struct GSM_PARAM_FROM_UI gsm_param;
};


struct RESPONSE_TO_GUI {
	bool init_dsp;
	bool tune_request;
	bool tune_gps;
	bool perform_network_scan;
	bool already_tuned;
	int16_t number_of_arfcn_scanned;
	int16_t arfcn_list[124];
};

extern struct COMMAND_FROM_GUI commandfromgui;
extern bool stop_read_tcp;
extern std::queue<struct COMMAND_FROM_GUI> command_queue;
extern sem_t command_wait;
extern boost::mutex tcp_write_sync;

#endif /* INCLUDE_TCPSERVER_HPP_ */
