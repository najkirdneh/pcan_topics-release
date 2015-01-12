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
#include "pcan_receive.h"

// DEFINES

#define DEFAULT_NODE "/dev/pcan32"

void pcan_receive::hlpMsg()
{
	/*!
	 *\brief The help message that is displayed when using the "-?" argument.
	 *
	 * The help message only prints a number of strings.
	 *
	 * */
	printf("pcan_receive - a small node, based on receivetest , which receives CAN messages and publishes them on a topic.\n");
	printf("usage:   pcan_receive {[-f=devicenode]  | {[-b=BTR0BTR1]  [-?]}}\n");
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


pcan_receive::pcan_receive()
{
	/*!
		 *\brief The constructor of the pcan_receive class is based on the first part of the main() function of the receivetest program.
		 *
		 * In the contructor the default parameters of the node are defined.
		 *
		 * */
	//Initialisation
	h = 0;
	pcan_pub = n.advertise<std_msgs::String>("pcan_received", 1);
}


void pcan_receive::init(int argc, char **argv)
{
	/*!
			 *\brief The init() function is based on the middle part of the main() function of the receivetest program.
			 *
			 * Compared to the receivetest program a lot of cases have been removed.
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
		    char *ptr;
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
		       printf("pcan_receive: can't open %s\n", szDevNode);
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
		       printf("pcan_receive: can't open %s device.\n", getNameOfInterface(nType));
		       do_exit(errno, h);;
		     }
		   }
		   /* clear status */
		     CAN_Status(h);
		     // get version info
		     errno = CAN_VersionInfo(h, txt);
		     if (!errno)
		       printf("pcan_receive: driver version = %s\n", txt);
		     else {
		       perror("pcan_receive: CAN_VersionInfo()");
		       do_exit(errno, h);;
		     }
		     // init to a user defined bit rate
		     if (wBTR0BTR1)
		     {
		       errno = CAN_Init(h, wBTR0BTR1, 0x00);
		       if (errno)
		       {
		         perror("pcan_receive: CAN_Init()");
		         do_exit(errno, h );;
		       }
		     }

}
void pcan_receive::receive()
{
	/*!
			 *\brief The receive() function is based on the read_loop() function of the receivetest program.
			 *
			 * In this part the program enters a loop untill it receives a CAN message.
			 * A string representation of this message is then published on the pcan_received topic.
			 *
			 * */
		// Real code
		TPCANRdMsg m;
		__u32 status;
		if (LINUX_CAN_Read(h, &m))
		{
			perror("pcan_receive: LINUX_CAN_Read()");
		}
		else
		{
			//print_message_ex(&m);
			// check if a CAN status is pending
			if (m.Msg.MSGTYPE & MSGTYPE_STATUS)
			{
				status = CAN_Status(h);
				if ((int)status < 0)
				{
					errno = nGetLastError();
					perror("pcan_receive: CAN_Status()");
				}
				else
					printf("pcan_receive: pending CAN status 0x%04x read.\n", (__u16)status);
			}
		}
		pcan_pub.publish(TPCANRdMsgToString(m));


}





std_msgs::String pcan_receive::TPCANRdMsgToString(TPCANRdMsg m)
{
	/*!
			 *\brief The TPCANRdMsgToString() function converts an object instance of a received CAN message into a string.
			 *
			 * The representing string also contains the time at which the message is received. First the time, and then the message, is converted to a string.
			 *
			 * */

	std_msgs::String msg;
	std::stringstream ss;
	ss << m.dwTime << ".";
	if(m.wUsec>=100)
	{
		ss << m.wUsec << " ";
	}
	else
	{
		if(m.wUsec>=10)
		{
			ss << "0" <<m.wUsec << " ";
		}
		else
		{
			ss << "00" <<m.wUsec << " ";
		}
	}
	char IDENT[4];
	sprintf(IDENT, "%02X", m.Msg.ID);
	ss << "0x" ;
	if(m.Msg.ID<16)
	{
		ss <<"0";
	}
	ss<< IDENT << " ";
	char LEN[3];
	sprintf(LEN, "%02X", m.Msg.LEN);
	ss << "0x" << LEN << " ";
	for (int i=0; i<sizeof(m.Msg.DATA); i++)
	{
		char DATA[3];
		sprintf(DATA, "%02X", m.Msg.DATA[i]);
		ss <<"0x" << DATA << " ";
	}
	msg.data=ss.str();
	return msg;
}
