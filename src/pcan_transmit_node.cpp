// INCLUDE

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>   // exit
#include <signal.h>
#include <string.h>
#include <stdlib.h>   // strtoul
#include <fcntl.h>    // O_RDWR
#include <ctype.h>
#include "common.h"
#include "pcan_transmit.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char** argv)
{
	/*!
			 *\brief The main() function only starts a ROS node, creates and initiates a pcan_transmit object instance, and starts to spin ROS.
			 *
			 *
			 *
			 * */
	ros::init(argc, argv, "pcan_transmit");
	pcan_transmit pcan_transmit_1;
	pcan_transmit_1.init(argc, argv);
	//ros::Rate(10000);
	ros::spin();
}

