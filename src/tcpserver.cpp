/*
 * tcpserver.cpp
 *
 *  Created on: 31-Jul-2019
 *      Author: root
 */

#include "../include/tcpserver.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <arpa/inet.h>

bool stop_read_tcp = false;
struct COMMAND_FROM_GUI commandfromgui;
std::queue<struct COMMAND_FROM_GUI> command_queue;
boost::mutex tcp_write_sync;

tcp_server::tcp_server(char *BIND_IP, int16_t BIND_PORT) {
	serverSocket = 0x00;
	clientSocket = 0x00;
	memset(&serv_addr, 0x00, sizeof(struct sockaddr_in));
	memset(&client_addr, 0x00, sizeof(struct sockaddr_in));
	if((serverSocket = socket(AF_INET, SOCK_STREAM, 0))<0){
		vipl_printf("error: Socket creation failed",error_lvl,__FILE__,__LINE__);
	    exit(EXIT_FAILURE);
	}
	if(error_lvl==3){
		vipl_printf("debug: Socket created",error_lvl,__FILE__,__LINE__);
	}
	int32_t flags = 1;
	if (setsockopt(serverSocket, SOL_SOCKET, SO_KEEPALIVE, (void *)&flags, sizeof(flags))){
		vipl_printf("error: unable to set flag for KEEPALIVE flag", error_lvl, __FILE__, __LINE__);
		exit(EXIT_FAILURE);
	}
	if(error_lvl==3){
		vipl_printf("debug: TCP Keepalive done",error_lvl,__FILE__,__LINE__);
	}
	flags = 10;
	if (setsockopt(serverSocket, SOL_TCP, TCP_KEEPIDLE, (void *)&flags, sizeof(flags))) {
		vipl_printf("error: unable to set flag for TCP_KEEPIDLE flag",error_lvl,__FILE__,__LINE__);
		exit(EXIT_FAILURE);
	};

	vipl_printf("debug: TCP TCP_KEEPIDLE done",error_lvl,__FILE__,__LINE__);
	bzero((char *)&serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(BIND_IP);
	serv_addr.sin_port = htons(BIND_PORT);

#if 0
	if(bind(serverSocket, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0){
		vipl_printf("error: Bind Failed",error_lvl,__FILE__,__LINE__);
	    exit(EXIT_FAILURE);
	}
	if(error_lvl==3){
		vipl_printf("debug: Bind done",error_lvl,__FILE__,__LINE__);
	}
	listen(serverSocket,2);
#endif
	label:
	if(connect(serverSocket , (struct sockaddr *)&serv_addr , sizeof(serv_addr)) < 0) {
		vipl_printf("error: unable to connect to GSM protocol stack",error_lvl,__FILE__,__LINE__);
		//exit(EXIT_FAILURE);
		//Waiting time on connection failure
		sleep(4);
		goto label;
	}
	if(error_lvl==3){
		vipl_printf("debug: connection done",error_lvl,__FILE__,__LINE__);
	}
}
/*
 * Return error codes:
 * 0: Close due to connection termination..re-connect
 * 1: Success
 * -1: close due failure
 */
int32_t tcp_server::read_action(void){
	socklen_t c = sizeof(client_addr);
	struct timeval tv;
	uint8_t *buffer = (uint8_t *)malloc(sizeof(commandfromgui));
	memset(buffer, 0x00, sizeof(uint8_t)*sizeof(commandfromgui));

	tv.tv_sec = 1;
	tv.tv_usec = 0;
	if(setsockopt(serverSocket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv)) {
		vipl_printf("error: unable to set flag for TCP_RECEIVE_TIMEOUT flag",error_lvl,__FILE__,__LINE__);
		exit(EXIT_FAILURE);
	};
	int32_t flag = 1;
	if(setsockopt(serverSocket, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int32_t))){
		vipl_printf("error: unable to disable Naggle's algorithm",error_lvl,__FILE__,__LINE__);
		exit(EXIT_FAILURE);
	}
	uint8_t check_buffer[sizeof(struct RESPONSE_TO_GUI)]={0x00};
	for(int32_t i=0;i<sizeof(struct RESPONSE_TO_GUI);i++)
		check_buffer[i]=0x2b;
	clock_t begin_time = std::clock();
	while(!stop_read_tcp){
		re_read:
		//Check socket status every 5 seconds
		if((float(std::clock()-begin_time)/CLOCKS_PER_SEC)>=10){
			int32_t n = send(serverSocket, check_buffer, sizeof(uint8_t)*sizeof(struct RESPONSE_TO_GUI), MSG_NOSIGNAL);
			if(n == -1){
				vipl_printf("error: unable to connect to GSM Protocol Stack....re-connecting", error_lvl, __FILE__, __LINE__);
				close(serverSocket);
				return 0x00;
			}
			begin_time = 0x00;
			begin_time = std::clock();
		}
		int32_t nBytesRead = 0x00, nBytesAlreadyRead =0x00;
		memset(buffer, 0x00, sizeof(uint8_t)*sizeof(commandfromgui));
		while((nBytesRead = recv(serverSocket,(uint8_t *)buffer+nBytesAlreadyRead,(sizeof(uint8_t)*sizeof(commandfromgui))-nBytesAlreadyRead,0))>0){
			nBytesAlreadyRead+= nBytesRead;
		}
		if(nBytesAlreadyRead==0x00){
			goto re_read;
		}
		if(nBytesAlreadyRead!=sizeof(commandfromgui)){
			char msg[300]={0x00};
			sprintf(msg,"error: improper read should have read %d Bytes but have read %d Bytes from protocol stack", sizeof(commandfromgui), nBytesAlreadyRead);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
			continue;
		}else{
			char msg[300]={0x00};
			sprintf(msg,"info: read %d Bytes from protocol stack", nBytesAlreadyRead);
			vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		}
		if(buffer[0]==0x2b && buffer[1]==0x2b){
			memset(buffer, 0x00, sizeof(uint8_t)*sizeof(commandfromgui));
		}
		memcpy(&commandfromgui, buffer, sizeof(commandfromgui));
		if(command_queue.size()<max_queue_size){
			command_queue.push(commandfromgui);
			try{
				sem_post(&command_wait);
			}catch(int32_t error) {
				vipl_printf("error: unable to post using using semaphores", error_lvl, __FILE__, __LINE__);
			}
		}
	}
	return 1;
}

void tcp_server::write_action(uint8_t *buffer) {
	int32_t nBytesWrote = 0x00, nBytesAlreadyWrote=0x00;
	struct RESPONSE_TO_GUI responsetogui;
	while((nBytesWrote = send(serverSocket,(uint8_t *)buffer+nBytesAlreadyWrote,(sizeof(uint8_t)*sizeof(responsetogui))-nBytesAlreadyWrote,0))>0){
		nBytesAlreadyWrote+= nBytesWrote;
	}
	if(nBytesAlreadyWrote!=sizeof(responsetogui)){
		char msg[300]={0x00};
		sprintf(msg,"error: improper write should have wrote %d Bytes but have wrote %d Bytes", sizeof(responsetogui), nBytesAlreadyWrote);
		vipl_printf(msg, error_lvl, __FILE__, __LINE__);
	}else{
		char msg[300]={0x00};
		sprintf(msg,"info: wrote %d Bytes to protocol stack", nBytesAlreadyWrote);
		vipl_printf(msg, error_lvl, __FILE__, __LINE__);
	}
}

tcp_server::~tcp_server() {
	// TODO Auto-generated destructor stub
	close(serverSocket);
}
