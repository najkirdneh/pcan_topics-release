//****************************************************************************
// INCLUDES
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>   // exit
#include <signal.h>
#include <string.h>
#include <stdlib.h>   // strtoul
#include "common.h"


//****************************************************************************
// CODE 



void signal_handler(int signal)
{
	/*!
					 *\brief The function is based on the signal_handler() function from the receivetest program.
					 *
					 *
					 *
					 * */
	HANDLE h;
	do_exit(0, h);
}

void do_exit(int error, HANDLE h)
{
	/*!
					 *\brief The function is based on the do_exit() function from both the transmi- and receivetest programs.
					 *
					 *
					 *
					 * */
	if (h)
	{
		print_diag("pcan_topics", h);
		CAN_Close(h);
	}
	printf("pcan_topics: finished (%d).\n\n", error);
	exit(error);
}


// the opposite: lookup for device string out of HW_.. constant
char const * getNameOfInterface(int nType)
{
	/*!
				 *\brief The function is copied from the common.cpp file used by both the transmi- and receivetest
				 *
				 *
				 *
				 * */
	switch (nType)
	{
    	case HW_PCI:            return "pci";
    	case HW_ISA_SJA:        return "isa";
    	case HW_DONGLE_SJA:     return "sp";
    	case HW_DONGLE_SJA_EPP: return "epp";
    	case HW_USB:            return "usb";
    	case HW_USB_PRO:        return "usbpro";
    	case HW_PCCARD:         return "pccard";
    	default:                return "unknown";
	}
}

// print out device and channel diagnostics
void print_diag(const char *prgName, HANDLE h)
{
	/*!
					 *\brief The function is copied from the common.cpp file used by both the transmi- and receivetest
					 *
					 *
					 *
					 * */
	int err;
	TPDIAG diag;
	err = LINUX_CAN_Statistics(h, &diag);
	if (err)
		printf("%s: can't read diagnostics, error %d!\n", prgName, err);
	else
	{
		printf("%s: type            = %s\n",              prgName, getNameOfInterface(diag.wType));
		if ((diag.wType == HW_USB) || (diag.wType == HW_USB_PRO))
		{
			printf("             Serial Number   = 0x%08x\n", diag.dwBase);
			printf("             Device Number   = %d\n",     diag.wIrqLevel);
		}
		else
		{
			printf("             io              = 0x%08x\n", diag.dwBase);
			printf("             irq             = %d\n",     diag.wIrqLevel);
		}
		printf("             count of reads  = %d\n",     diag.dwReadCounter);
		printf("             count of writes = %d\n",     diag.dwWriteCounter);
		printf("             count of errors = %d\n",     diag.dwErrorCounter);
		printf("             count of irqs   = %d\n",     diag.dwIRQcounter);
		printf("             last CAN status = 0x%04x\n", diag.wErrorFlag);
		printf("             last error      = %d\n",     diag.nLastError);
		printf("             open paths      = %d\n",     diag.nOpenPaths);
		printf("             driver version  = %s\n",     diag.szVersionString);
	}
}

