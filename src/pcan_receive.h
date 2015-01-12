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


class pcan_receive
{
protected:
	HANDLE h;
	ros::NodeHandle n;
	ros::Publisher pcan_pub;
	std_msgs::String msg;
	std::stringstream ss;
public:
	pcan_receive();
	void hlpMsg();
	void init(int argc, char **argv);
	void receive();
	std_msgs::String TPCANRdMsgToString(TPCANRdMsg m);


};
