// INCLUDE

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>   // exit
#include <signal.h>
#include <string.h>
#include <stdlib.h>   // strtoul
#include <fcntl.h>    // O_RDWR
#include "common.h"
#include <ctype.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

// DEFINES

#define DEFAULT_NODE "/dev/pcan32"


class pcan_transmit
{
protected:
	HANDLE h;
	ros::NodeHandle n;
	ros::Subscriber pcan_sub;
	std_msgs::String msg;
	std::stringstream ss;
public:
	pcan_transmit();
	void hlpMsg();
	void init(int argc, char **argv);
	void transmit(const std_msgs::String can_message);
	TPCANMsg StringToTPCANRdMsg(std_msgs::String msg);
};
