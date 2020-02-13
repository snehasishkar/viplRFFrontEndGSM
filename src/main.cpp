/*
 * main.cpp
 *
 *  Created on: 03-Apr-2019
 *      Author: Snehasish Kar
 */

#include <iostream>
#include <string>
#include <queue>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <getopt.h>
#include <boost/thread.hpp>
#include <semaphore.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <thread>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/mount.h>

#include <sys/types.h>
#include <sys/capability.h> //TODO: sudo apt-get install libcap-dev
#include <sys/prctl.h>

#include "../include/tcpserver.hpp"
#include "../include/vipl_printf.hpp"
#include "../include/viplrfinterface.hpp"
#include "../include/viplscannerinit.h"


int32_t error_lvl = 0x00;

sem_t lock;
sem_t stop_process;
boost::mutex rftap_sync;
sem_t command_wait;
sem_t stop_receive_gr;
bool ENABLE_RFNOC = false;

extern int errno;

bool wait_done = false;
bool stop_rx = false;
bool stop_gps = false;
bool stop_read_fifo = false;

void printUsage(void){
	fprintf(stderr,"\t ./viplrfFrontend -e <1/2/3>\n");
	fprintf(stdout,"\t -e:   error level (1/2/3)\n");
	fprintf(stdout,"\t -i:   IP to listen for GUI commands\n");
	fprintf(stdout,"\t -p:   PORT number to listen GUI commands\n");
	fprintf(stdout,"\t -v:   version\n");
	fprintf(stdout,"\t -r:   to enable the use of RFNOC\n");
	fprintf(stdout,"\t -m:   1: mount nfs; 0: dont mount nfs\n");
}

void version(void){
	fprintf(stderr,"Build on %s:%s by Snehasish Kar",__DATE__, __TIME__);
}

void intHandler(int32_t dummy) {
	stop_rx = true;
	stop_gps = true;
	stop_read_tcp = true;
	vipl_printf("ALERT: sigint got..exiting!!", error_lvl, __FILE__, __LINE__);
	sem_post(&stop_process);
	sleep(4);
	sem_destroy(&stop_process);
	exit(EXIT_SUCCESS);
}

void clearApp() {
	stop_rx = true;
	stop_gps = true;
	stop_read_tcp = true;
	sem_post(&command_wait);
	sleep(5);
	//stop_rx = false;
	stop_gps = false;
	stop_read_tcp = false;
}


bool set_permissions(){
	uid_t       user;
	cap_value_t root_caps[4] = { CAP_SYS_NICE, CAP_IPC_LOCK, CAP_SETUID, CAP_DAC_READ_SEARCH };
	cap_value_t user_caps[3] = { CAP_SYS_NICE, CAP_IPC_LOCK, CAP_DAC_READ_SEARCH };
	cap_t       capabilities;

	/*
	 * Including CAP_DAC_READ_SEARCH here is a major security problem, but
	 * the entries in /proc/self are created with root as owner and are
	 * therefore inaccessible after root privileges are dropped..
	 */

	/* Get real user ID. */
	user = getuid();

	/*
	 * Get full root privileges. Normally being effectively root
	 * (see man 7 credentials, User and Group Identifiers, for explanation
	 *  for effective versus real identity) is enough, but some security
	 * modules restrict actions by processes that are only effectively root.
	 * To make sure we don't hit those problems, we switch to root fully.
	 */

	if(setresuid(0, 0, 0)){
		char msg[200]={0x00};
		sprintf(msg,"error: cannot switch to root: %s", strerror(errno));
		vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		return false;
	}

	/*
	 * We can still switch to a different user due to having the CAP_SETUID
	 * capability. Let's clear the capability set, except for the CAP_SYS_NICE
	 * in the permitted and effective sets.
	 */

	if(cap_clear(capabilities)){
		char msg[200]={0x00};
		sprintf(msg,"error: cannot clear capability data structure: %s", strerror(errno));
		vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		return false;
	}

	if(cap_set_flag(capabilities, CAP_PERMITTED, sizeof user_caps / sizeof user_caps[0], user_caps, CAP_SET) ||
		cap_set_flag(capabilities, CAP_EFFECTIVE, sizeof user_caps / sizeof user_caps[0], user_caps, CAP_SET)){
		char msg[200]={0x00};
		sprintf(msg,"error: cannot manipulate capability data structure as user: %s", strerror(errno));
		vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		return false;
	}

	/*
	 * 	Apply modified capabilities.
	 */
	if(cap_set_proc(capabilities)){
		char msg[200]={0x00};
		sprintf(msg,"error: Cannot set capabilities as user: %s", strerror(errno));
		vipl_printf(msg, error_lvl, __FILE__, __LINE__);
		return false;
	}
	if(error_lvl==3)
		vipl_printf("info: Capabilities were successfully set", error_lvl, __FILE__, __LINE__);
	cap_free(capabilities);
	return true;
}

void dequeue_init(tcp_server *tcpServer_gui){
	vipl_scanner_init viplScannerInit;
	viplScannerInit.dequeue(tcpServer_gui);
}



char *ntwrkscan ="/mnt/viplrfinterface_writter/ntwrk_scan";
char *signalling ="/mnt/viplrfinterface_writter/signalling";
char *traffic ="/mnt/viplrfinterface_writter/traffic";

int32_t main(int argc, char *argv[]){
	int32_t opt = 0x00;
	bool use_mount = true;
	char GUI_SERVER_IP_DEFINED[16] = {0x00};
	int16_t GUI_SERVER_PORT_DEFINED = 0x00;
	if(argc<=1){
		fprintf(stderr,"No parameters found\n");
		return (EXIT_FAILURE);
	}
	while((opt = getopt(argc, argv, "e:i:hv:p:r:m:"))!= -1) {
		switch(opt) {
			case 'h': printUsage();
	            	  exit(EXIT_SUCCESS);
	            	  break;
	        case 'e': sscanf(optarg, "%d", &error_lvl);
	        	 	  break;
	        case 'v': version();
	        		  break;
	        case 'i': bzero(GUI_SERVER_IP_DEFINED ,sizeof(char)*300);
	        		  strcpy(GUI_SERVER_IP_DEFINED, optarg);
	        	      break;
	        case 'p': sscanf(optarg, "%d", &GUI_SERVER_PORT_DEFINED);
	        		  break;
	        case 'r': sscanf(optarg, "%d", &ENABLE_RFNOC);
	        	      break;
	        case 'm': sscanf(optarg, "%d", &use_mount);
	        	      break;
	        default:  exit(EXIT_FAILURE);
	            	  break;
	    }
	}

	//initialize the signal handler
	signal(SIGINT, intHandler);
	sem_init(&stop_process, 0, 1);
	sem_init(&command_wait, 0, 1);

	//check whether nfs folder exists or not
	char *directory_path= "/mnt/viplrfinterface_writter";
	DIR *dir = opendir(directory_path);
	if(dir==NULL){
		char mount_command[100]={0x00};
		//sprintf(mount_command,"mount %s:/mnt/viplrfinterface %s",GUI_SERVER_IP_DEFINED, directory_path);
		sprintf(mount_command,"nolock,addr=%s",GUI_SERVER_IP_DEFINED);
		if((mkdir(directory_path,S_IRWXU)==-1) && (mkdir(ntwrkscan,S_IRWXU)!=EEXIST)){
			vipl_printf("error: unable to create folder for mount target..exiting", error_lvl, __FILE__, __LINE__);
			exit(EXIT_FAILURE);
		}
		if(!use_mount){
		retry_mount:
			if(mount("/mnt/viplrfinterface", directory_path,"nfs",0x00,mount_command)!=0){
				vipl_printf("error: unable to mount file system...exiting", error_lvl, __FILE__, __LINE__);
				goto retry_mount;
			}
		}
		//system(mount_command);
		if(error_lvl==3){
			vipl_printf("info: nfs drive mounted", error_lvl, __FILE__, __LINE__);
		}
		if((mkdir(ntwrkscan,S_IRWXU)==-1) && (mkdir(ntwrkscan,S_IRWXU)!=EEXIST)){
			vipl_printf("error: unable to create folder for dumping network scan..exiting", error_lvl, __FILE__, __LINE__);
			exit(EXIT_FAILURE);
		}
		if((mkdir(signalling,S_IRWXU)==-1) && (mkdir(ntwrkscan,S_IRWXU)!=EEXIST)){
			vipl_printf("error: unable to create folder for dumping network scan..exiting", error_lvl, __FILE__, __LINE__);
			exit(EXIT_FAILURE);
		}
		if((mkdir(traffic,S_IRWXU)==-1) && (mkdir(ntwrkscan,S_IRWXU)!=EEXIST)){
			vipl_printf("error: unable to create folder for dumping network scan..exiting", error_lvl, __FILE__, __LINE__);
			exit(EXIT_FAILURE);
		}
	}
	/*
	 * initialize semaphore for raising a signal only when a queue is to be pushed and popped
	 */
	//So that we dont eatup CPU clocks

	sem_init(&lock, 0, 1);
	sem_init(&stop_receive_gr, 0, 1);
	{
		char msg[200]={0x00};
		sprintf(msg, "info: Process started with pid: %d", getpid());
		vipl_printf(msg, error_lvl, __FILE__, __LINE__);
	}

	struct RESPONSE_TO_GUI write_to_gui;
	memset(&write_to_gui, 0x00, sizeof(write_to_gui));
	write_to_gui.init_dsp = true;
	uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t)*sizeof(write_to_gui));
	memset(buffer, 0x00, sizeof(uint8_t)*sizeof(write_to_gui));
	memcpy(buffer, &write_to_gui, sizeof(write_to_gui));
	//boost::thread t0(vocoder_init);
	label:
	if(((strlen(GUI_SERVER_IP_DEFINED)>=8) && (strlen(GUI_SERVER_IP_DEFINED)<=16)) && (GUI_SERVER_PORT_DEFINED)){
		tcp_server tcpserverforgui(GUI_SERVER_IP_DEFINED, GUI_SERVER_PORT_DEFINED);
		boost::thread t1(dequeue_init, &tcpserverforgui);
		tcpserverforgui.write_action(buffer);
		int32_t rtnval = tcpserverforgui.read_action();
		if(rtnval==0) {
			clearApp();
			goto label;
		}
	}else{
		tcp_server tcpserverforgui(DEFAULT_PROTOCOL_STACK_SERVER_IP, DEFAULT_PROTOCOL_STACK_SERVER_PORT);
		tcpserverforgui.write_action(buffer);
		boost::thread t1(dequeue_init, &tcpserverforgui);
		int32_t rtnval = tcpserverforgui.read_action();
		if(rtnval==0){
			clearApp();
			goto label;
		}
	}
	//tcp_serv servport;
	return 0x00;
}
