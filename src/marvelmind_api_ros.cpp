/*
Copyright (c) 2022, Marvelmind Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUfTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "marvelmind_nav/MarvelmindAPI.h"

extern "C" 
{
#include "marvelmind_nav/marvelmind_api.h"
}

#define ROS_NODE_NAME "marvelmind_api_ros"

#define MM_API_SEMAPHORE "/mm_api_semaphore"

///////////////////////////

static uint32_t mm_api_version= 0;
static char ttyFileName[256];
static bool openPortTryNeeded= true;

static sem_t *sem;
struct timespec ts;


//////////////////////////

void semCallback()
{
	sem_post(sem);
}

void tryOpenPort(void) {
	if (ttyFileName[0] == 0) {
		if (mmOpenPort()) {
		    ROS_INFO( "Port opened");
		    openPortTryNeeded= false;
	    }
	    else {
		    ROS_INFO( "Failed to open port. Retrying...");
	    }
	} else {
		char portName[256];
	    strcpy(portName, ttyFileName);
	    if (mmOpenPortByName(portName)) {
		    ROS_INFO( "Port opened: %s", portName);
		    openPortTryNeeded= false;
	    }
	    else {
		    ROS_INFO( "Failed to open port: %s. Retrying...", portName);
	    }
	}
}


static void apiPrepare(int argc, char **argv)
{
	 // get port name from command line arguments (if specified)
	ttyFileName[0] = 0;
    if (argc>=2) {
		strcpy(ttyFileName,argv[1]);
	}  
    mm_api_version = 0;
	openPortTryNeeded= true;
	
	ROS_INFO( "Open Marvelmind API");
	marvelmindAPILoad(NULL);
	
	if (mmAPIVersion(&mm_api_version)) {
		ROS_INFO("Marvelmind API version: %d", (int)mm_api_version);

        tryOpenPort();
	}
	else {
		ROS_INFO("Failed to get Marvelmind API version");
	}
}


bool mmAPIrequestProcess(marvelmind_nav::MarvelmindAPI::Request &req,
	                     marvelmind_nav::MarvelmindAPI::Response &res) {

	std::vector<uint8_t>* pdv = &req.request;
	uint8_t* rq_data = pdv->data();
	uint32_t rq_size = (uint32_t) pdv->size();
	int64_t rq_command = req.command_id;

	uint8_t response_buf[1024];
	uint32_t response_size = 1024;
	int32_t error_code = -1;

    bool result = marvelmindAPICall(rq_command, rq_data, rq_size, &response_buf[0], &response_size, &error_code);
	res.success = result;
	res.error_code = error_code;
	std::vector<uint8_t>* resv = &res.response;
	if (res.success) {
		resv->resize(response_size);
		uint8_t* resp_data = resv->data();
		for (uint32_t i = 0; i < response_size; i++) {
			resp_data[i] = response_buf[i];
		}
	}
	else {
		resv->resize(0);
	}
	
	switch(rq_command) {
		case MM_API_ID_OPEN_PORT:
		case MM_API_ID_OPEN_PORT_BY_NAME:
		case MM_API_ID_OPEN_PORT_UDP:
		case MM_API_ID_CLOSE_PORT: {
			if (openPortTryNeeded) {
				ROS_INFO("Port access function call: automatic port open cancelled");
				openPortTryNeeded= false;
			}
		}
	}


	return true;
}

void timer1Callback(const ros::TimerEvent&event)
{
	if (openPortTryNeeded) {
		tryOpenPort();
	}
	
	if (openPortTryNeeded)
	  return;
	
	MarvelmindDevicesList md;
	mmGetDevicesList(&md);
}


int main(int argc, char * argv[])
{
  // initialize ROS node
  ros::init(argc, argv, ROS_NODE_NAME);
  
  sem = sem_open(MM_API_SEMAPHORE, O_CREAT, 0777, 0);
  
  apiPrepare(argc, argv);
  
  // ROS node reference 
  ros::NodeHandle n;
  
  ros::Timer timer= n.createTimer(ros::Duration(1.0), timer1Callback);
  
  ros::ServiceServer service= n.advertiseService("marvelmind_api", mmAPIrequestProcess);
  
  ros::Rate loop_rate(200);
  while (ros::ok()) {
	if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
     {
        ROS_INFO("clock_gettime");
        return -1;
	 }
    ts.tv_sec += 2;
    sem_timedwait(sem,&ts); 
    
	ros::spinOnce();

    loop_rate.sleep();
  }
  
  mmClosePort();// Close port (if was opened)

  marvelmindAPIFree();// Free Marvelmind API library memory
  
  sem_close(sem);
  
  return 0;
}
