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
#include "pcan_transmit.h"



void pcan_transmit::hlpMsg()
{
	/*!
		 *\brief The help message that is displayed when using the "-?" argument.
		 *
		 * The help message only prints a number of strings.
		 *
		 * */
	printf("pcan_transmit - a small node, based on transmitest , which is subscribed to a ROS topic and sends CAN messages.\n");
	printf("usage:   pcan_transmit {[-f=devicenode]  | {[-b=BTR0BTR1]  [-?]}} \n");
	printf("options: -f - devicenode, default=%s\n", DEFAULT_NODE);
	printf("         -b - BTR0BTR1 code in hex, e.g.\n");
	printf("		0x0014  =   1 MBit/s\n");
	printf("		0x001C  = 500 kBit/s\n");
	printf("		0x011C  = 250 kBit/s\n");
	printf("		0x031C  = 125 kBit/s\n");
	printf("		0x432F  = 100 kBit/s\n");
	printf("		0x472F  =  50 kBit/s\n");
	printf("		0x532F  =  20 kBit/s\n");
	printf("		0x672F  =  10 kBit/s\n");
	printf("		0x7F7F  =   5 kBit/s\n");
	printf("		(default: 1 Mbit).\n");
	printf("	-? or --help - this help\n");
	printf("\n");
}




pcan_transmit::pcan_transmit()
{
	/*!
			 *\brief The constructor of the pcan_transmit class is based on the first part of the main() function of the transmitest program.
			 *
			 * In the contructor the default parameters of the node are defined.
			 *
			 * */
	//Initialisation
	h = 0 ;
	pcan_sub = n.subscribe("pcan_transmitted", 1, &pcan_transmit::transmit, this);
}

void pcan_transmit::init(int argc, char **argv)
{
	/*!
				 *\brief The init() function is based on the middle part of the main() function of the transmitest program.
				 *
				 * Compared to the transmitest program a lot of cases have been removed.
				 * Therefore, only the PCAN-USB adapter is, and standard messages are, supported.
				 *
				 * */

	int   nExtended = CAN_INIT_TYPE_ST;
	int nType;
	__u32 dwPort;
	__u16 wIrq;
	char *ptr;
	__u16 wBTR0BTR1 = 0x0014;
	// parameter wBTR0BTR1
	// bitrate codes of BTR0/BTR1 registers
	//#define CAN_BAUD_1M     0x0014  //   1 MBit/s
	//#define CAN_BAUD_500K   0x001C  // 500 kBit/s
	//#define CAN_BAUD_250K   0x011C  // 250 kBit/s
	//#define CAN_BAUD_125K   0x031C  // 125 kBit/s
	//#define CAN_BAUD_100K   0x432F  // 100 kBit/s
	//#define CAN_BAUD_50K    0x472F  //  50 kBit/s
	//#define CAN_BAUD_20K    0x532F  //  20 kBit/s
	//#define CAN_BAUD_10K    0x672F  //  10 kBit/s
	//#define CAN_BAUD_5K     0x7F7F  //   5 kBit/s
	const char  *szDevNode = DEFAULT_NODE;
	bool bDevNodeGiven = false;
	bool bTypeGiven = false;
	char txt[VERSIONSTRING_LEN];

	// decode command line arguments
		for (int i = 1; i < argc; i++)
		{
		    char c;

		    ptr = argv[i];

		    while (*ptr == '-')
		      ptr++;

		    c = *ptr;
		    ptr++;

		    if (*ptr == '=')
		      ptr++;

		    switch(tolower(c))
		    {
		      case '?':
		      case 'h':
		    	hlpMsg();
		    	do_exit(errno, h);
		        break;
		      case 'f':
		        szDevNode = ptr;
		        bDevNodeGiven = true;
		        break;
		      case 'b':
		        wBTR0BTR1 = (__u16)strtoul(ptr, NULL, 16);
		        break;
		      default:
		         errno = EINVAL;
		         do_exit(errno, h);;
		        break;
		    }
		}
		  /* open CAN port */
		   if ((bDevNodeGiven) || (!bDevNodeGiven && !bTypeGiven))
		   {
		     h = LINUX_CAN_Open(szDevNode, O_RDWR);
		     if (!h)
		     {
		       printf("pcan_transmit: can't open %s\n", szDevNode);
		       do_exit(errno, h);;
		     }
		   }
		   else {
		     // please use what is appropriate
		     // HW_DONGLE_SJA
		     // HW_DONGLE_SJA_EPP
		     // HW_ISA_SJA
		     // HW_PCI
		     h = CAN_Open(nType, dwPort, wIrq);
		     if (!h)
		     {
		       printf("pcan_transmit: can't open %s device.\n", getNameOfInterface(nType));
		       do_exit(errno, h);;
		     }
		   }
		   /* clear status */
		     CAN_Status(h);
		     // get version info
		     errno = CAN_VersionInfo(h, txt);
		     if (!errno)
		       printf("pcan_transmit: driver version = %s\n", txt);
		     else {
		       perror("pcan_transmit: CAN_VersionInfo()");
		       do_exit(errno, h);;
		     }
		     // init to a user defined bit rate
		     if (wBTR0BTR1)
		     {
		       errno = CAN_Init(h, wBTR0BTR1, nExtended);
		       if (errno)
		       {
		         perror("pcan_transmit: CAN_Init()");
		         do_exit(errno, h);;
		       }
		     }

}
TPCANMsg pcan_transmit::StringToTPCANRdMsg(std_msgs::String msg)
{
	/*!
				 *\brief This function checks whether the string can be converted into a CAN message.
				 *\brief If this is a possibility the corresponding CAN message is send.
				 *
				 *
				 *
				 * */
	TPCANMsg m;
	std::string can_message = msg.data;
	std::stringstream ss;
	std::istringstream iss(can_message);
	std::vector<std::string> tokens;
	std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter<std::vector<std::string> >(tokens));
	if(tokens.size()>2)
	{
		m.ID = (DWORD) std::strtol(tokens.at(0).data(),NULL,0);
		m.LEN = (BYTE) std::strtol(tokens.at(1).data(),NULL,0);
	}
	if(tokens.size()>3 && tokens.size()<11)
	{
		for(int i = 2; i < tokens.size(); i++)
		{
			m.DATA[i-2] = (BYTE) std::strtol(tokens.at(i).data(),NULL,0);
		}

	}
	return m;
}


void pcan_transmit::transmit(const std_msgs::String msg)
{
	/*!
				 *\brief The transmit() function is based on the write_loop() function of the transmitest program.
				 *
				 * In this part the program waits for a string publication on the pcan_transmitted
				 * topic and then sends a CAN message based on this string.
				 *
				 * */
	TPCANMsg m = StringToTPCANRdMsg(msg);

	//send the message
	//CheckTPCANMsg(m) &&
	if (  CAN_Write(h, &m))
	{
		perror("pcan_transmit: CAN_Write()");
	}
}


