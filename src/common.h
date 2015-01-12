//****************************************************************************
// INCLUDES
#include <libpcan.h>



#ifdef __cplusplus
extern "C" 
{
#endif


void signal_handler(int signal);

void do_exit(int error, HANDLE h);

// the opposite: lookup for device string out of HW_.. constant
char const* getNameOfInterface(int nType);

// print out device and channel diagnostics
void print_diag(const char *prgName, HANDLE h);
#ifdef __cplusplus
}
#endif

