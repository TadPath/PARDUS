/******************************************************************************/
/*                                                                            */
/* PARDserver                                                                 */
/*                                                                            */
/* Copyright (c) Dr Paul J. Tadrous 2000-2026   All rights reserved.          */
 /*                                                                          */
  /*                                                                        */
   /*                                                                      */
    /* This program is distributed under the terms of the GNU General     */
    /* Public License (GPL v3 or later).                                  */
    /*                                                                    */
    /* This program is free software: you can redistribute it and/or      */
    /* modify it under the terms of the GNU General Public License as     */
    /* published by the Free Software Foundation, either version 3 of the */
    /* License, or (at your option) any later version.                    */
    /*                                                                    */
    /* This program is distributed in the hope that it will be useful,    */
    /* but WITHOUT ANY WARRANTY; without even the implied warranty of     */
    /* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU   */
    /* General Public License for more details.                           */
    /*                                                                    */
    /* You should have received a copy of the GNU General Public License  */
    /* along with this program. If not, see                               */
    /* <https://www.gnu.org/licenses/>.                                   */
   /*                                                                      */
  /*                                                                        */
 /*                                                                          */
/* pardserver is the main central control server of the PARDUS microscope /   */
/* robotics control software. It acts as a conduit of communication between   */
/* the user (who logs into pardserver via the pardclient application which    */
/* may be on a remote computer over the internet and all comms between the    */
/* pardclient and pardserver are end-to-end encrypted with elliptical curve   */
/* cryptography using Zero MQ), the motion control breakout board (BOB) of    */
/* the microscope system (which, in the current implementation, is an Arduino */
/* Mega 2560 compatible microcontroller) and, optionally, one or more pardcap */
/* image servers to enable image capture and processing abilities.            */
/*                                                                            */
/* Pardserver acts as a client in communications with pardcap server(s) and   */
/* each padrcap server may be on the same computer as parserver or on         */
/* different computers. In each case, connection to a pardcap server is made  */
/* via (unencrypted) TCP/IP socket comms.                                     */
/* The BOB will be connected to the same computer running pardserver for two- */
/* way serial communication.                                                  */
/* In the case of the BOB being an Arduino, this communication will be done   */
/* via the USB cable connecting the BOB to the PC.                            */
/*                                                                            */
/* pardserver generates 'orders' to send to the BOB. These orders make the    */
/* microscope (or other robot) do something or get some data from the BOB     */
/* (like the current position of an axis) or set some data on the BOB (like   */
/* settings for the motors; for example the number of backlash correction     */
/* steps each motor needs or the microstep size used by a motor). These       */
/* orders are formulated by pardserver based on some combination of           */
/* information pardserver receives from the user, a camera and data fed back  */
/* from the BOB.                                                              */
/*                                                                            */
/* After receiving an order, the BOB will attempt to fulfil it and send back  */
/* a report on what it has done (which will include an error status code that */
/* will take the value PARDU_ERR_NONE if all went well or one of the other    */
/* PARDU_ERR_ codes describing the error if there was a problem).             */
/*                                                                            */
/* In a similar way, pardserver generates 'instructions' to send to connected */
/* pardcap servers to tell the pardcap server what do do - like capture an    */
/* image and send it back to the pardserver or to change the value of some    */
/* camera setting, etc.                                                       */
/*                                                                            */
/* See also the PARDUS documentation and the programs PARDcap, PARDclient and */
/* PARDuino.ino                                                               */
/*                                                                            */
/*                                               PJT 17.07.2025 - 26.10.2026  */
/*                                                                            */
/* Cross-platform port (MS Windows via MSYS2 UCRT64 and macOS via Xcode/clang)*/
/* 12.11.2026: Every function that touches an operating-system service which  */
/* differs between Linux, Windows and macOS (the Arduino serial port, the     */
/* PARDcapture TCP sockets, the Ctrl+C handler, sleeping, privilege checks,   */
/* the system security log and file permissions / atomic file replacement) is */
/* now provided in whole-function pre-processor branches:                     */
/*                                                                            */
/*    #if IS_WINDOWS_NATIVE   ...  Windows version (Win32 / Winsock 2 API)    */
/*    #elif defined(__APPLE__) ... macOS version (only where macOS differs)   */
/*    #else                   ...  original Linux version, left unchanged     */
/*                                                                            */
/* Where macOS behaves exactly as Linux (POSIX) the original function is used */
/* for both and the branch reads  #if IS_WINDOWS_NATIVE ... #else ... #endif  */
/* All OS-neutral code, including all of the ZeroMQ routines used to talk to  */
/* pardclient, is shared and unchanged on every operating system.             */
/*                                                                            */
/* 17.11.2026: ps_recv_dc() reports a 0-byte recv() (an orderly close by the  */
/* PARDcapture server) as a hang-up (code 5) on Linux and macOS, as it already*/
/* was on Windows (the Prof's choice N1; the same rule as in pardcap).        */
/*                                                                            */
/******************************************************************************/

// Version number
#define PARDSERVER_VERSION "1.0.0"

// Compile with:
//   Linux (GCC):
//     gcc -O2 -march=native pardserver.c -Wall -o pardserver -lzmq -lsodium -lm
//
//   MS Windows (in an MSYS2 UCRT64 shell, GCC; type all on one line):
//     gcc -O2 -march=native pardserver.c -Wall -o pardserver.exe -lzmq -lsodium
//         -lws2_32 -lsetupapi -ladvapi32 -lm
//   (-lsetupapi since 14.11.26: SetupAPI lists the COM ports for bob_path auto)
//
//   macOS (Terminal, Xcode / Command Line Tools clang, Homebrew packages
//          zeromq, libsodium and pkgconf; type all on one line):
//     clang -std=gnu2x -O2 -Wall pardserver.c -o pardserver
//         $(pkg-config --cflags --libs libzmq libsodium) -lm
//   (-std=gnu2x: the shared code uses two C23 features - unnamed parameters
//    in function definitions and labels at the end of a compound statement -
//    which clang's default gnu17 mode accepts only with -Wc23-extensions
//    warnings; GCC accepts both without a warning in its default mode.)

////////////////////////////////////////////////////////////////////////////////
//////////                                                            //////////
///////                   OPERATING SYSTEM DETECTION                   /////////
//                                                                            //
// IS_WINDOWS_NATIVE is 1 only for a native Windows tool-chain (MinGW-w64 as
// used by the MSYS2 UCRT64 environment, or MSVC). Cygwin and the MSYS runtime
// (msys-2.0.dll) define __unix__ and are therefore NOT native Windows; WSL is
// Linux. macOS is recognised by __APPLE__ and everything else is Linux.
#if !defined(__unix__) && !defined(__linux__) && !defined(__APPLE__) && defined(_WIN32)
    #define IS_WINDOWS_NATIVE 1
#else
    #define IS_WINDOWS_NATIVE 0
#endif
//                                                                            //
///////                END OF: OPERATING SYSTEM DETECTION              /////////
//////////                                                            //////////
////////////////////////////////////////////////////////////////////////////////

// _.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._. //


////////////////////////////////////////////////////////////////////////////////
//////////                                                            //////////
///////                           HEADER FILES                         /////////
//                                                                            //
#if IS_WINDOWS_NATIVE
  //-----------------------------------------------------------------------\\.
 //                        MS Windows (native Win32)                        \\.
//---------------------------------------------------------------------------\\.
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN  // Exclude rarely used parts of windows.h (rpc, ole,
#endif                       // shell, etc.) - also keeps 'byte' etc. free for us.
#ifndef NOMINMAX
#define NOMINMAX             // Stop windows.h from defining min() and max().
#endif
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0A00  // Windows 10 or later: WSAPoll(), inet_ntop(),
#endif                       // and CreateWaitableTimerExW().
#ifndef __USE_MINGW_ANSI_STDIO
#define __USE_MINGW_ANSI_STDIO 1 // Use the MinGW-w64 C99 printf/scanf family so
#endif                       // that the C99 formats of the shared code (%zu,
                             // %lld, %hhu via SCNu8, %la) behave identically
                             // in every MSYS2 environment (UCRT64, MINGW64,
                             // CLANG64) and are checked correctly by GCC.
#include <winsock2.h>        // Winsock 2 sockets (MUST come before windows.h)
#include <ws2tcpip.h>        // For getaddrinfo(), inet_ntop(), INET6_ADDRSTRLEN
#include <windows.h>         // Win32 API: COM ports, Event Log, timers, tokens
#include <setupapi.h>        // SetupAPI: list of present COM ports ('auto' search)
#include <sddl.h>            // For the restricted security descriptor (key file)
#include <io.h>              // For _open_osfhandle(), _commit(), _close()
#include <fcntl.h>           // For _O_WRONLY and _O_TEXT
#include <stdio.h>           // For printf() etc.
#include <stdlib.h>          // For exit() function
#include <string.h>          // For strcmp() etc.
#include <errno.h>           // For strerror() etc.
#include <stdint.h>          // To harmonise integer sizes between PC and Arduino
#include <stdarg.h>          // For the variadic Event Log writer win_syslog()
#include <time.h>            // For time stamps and (MinGW-w64) clock_gettime()
#include <math.h>            // For fabs
#include <sodium.h>          // For password hashing
#include <inttypes.h>        // For the PRIu32 etc. format specifiers
#include "pardcapdefs.h"     // Some common definitions for client and server

// To set up ctrl-c interrupt handler (sig_atomic_t and SIGINT)
#include <signal.h>

// To get user input
#include <ctype.h>

// For PATH_MAX and INT_MAX
#include <limits.h>

// This is needed for the ZeroMQ network communications protocol
#include <zmq.h>

// For PARDUS-specific structs and #defines
#include "parddefs.h"

//---------------------------------------------------------------------------//.
 //                        MS Windows (native Win32)                        //.
  //-----------------------------------------------------------------------//.
#elif defined(__APPLE__)
  //-----------------------------------------------------------------------\\.
 //                                  macOS                                  \\.
//---------------------------------------------------------------------------\\.
#include <stdio.h>        // For printf() etc.
#include <stdlib.h>       // For exit() function
#include <string.h>       // For strcmp() etc.
#include <unistd.h>       // For sleep(), read/write/close() etc.
#include <fcntl.h>        // For open() and fcntl(F_FULLFSYNC)
#include <termios.h>      // For serial comms functions
#include <errno.h>        // For strerror() etc.
#include <stdint.h>       // To harmonise integer sizes between PC and Arduino
#include <time.h>         // To enable msleep function and datestamping logs
#include <math.h>         // For fabs
#include <sys/ioctl.h>    // For serial comms functions
#include <IOKit/serial/ioss.h> // For IOSSIOSPEED (any baud rate, e.g. 500000)
                          // and IOSSDATALAT (low receive latency) ioctls.
#include <glob.h>         // For auto-detection of /dev/cu.usbmodem* etc.
#include <sys/stat.h>     // For the encryption key generator
#include <sodium.h>       // For password hashing
#include <syslog.h>       // For logging security notices on the system log

// For PARD Client (PC) comms with a PARDcapture server
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <inttypes.h>
#include "pardcapdefs.h" // Some common definitions for client and server

// To set up ctrl-c interrupt handler
#include <signal.h>

// To get user input
#include <ctype.h>

// For PATH_MAX (via sys/syslimits.h) and INT_MAX
#include <limits.h>

// This is needed for the ZeroMQ network communications protocol
#include <zmq.h>

// For PARDUS-specific structs and #defines
#include "parddefs.h"

//---------------------------------------------------------------------------//.
 //                                  macOS                                  //.
  //-----------------------------------------------------------------------//.
#else
  //-----------------------------------------------------------------------\\.
 //                       Linux (original, unchanged)                       \\.
//---------------------------------------------------------------------------\\.
#ifdef __linux__
#include <stdio.h>        // For printf() etc.
#include <stdlib.h>       // For exit() function
#include <string.h>       // For strcmp() etc.
#include <unistd.h>       // For sleep(), read/write/close() etc. 
#include <fcntl.h>        // For open()
#include <termios.h>      // For serial comms functions
#include <errno.h>        // For strerror() etc.
#include <stdint.h>       // To harmonise integer sizes between PC and Arduino
#include <time.h>         // To enable msleep function and datestamping logs
#include <math.h>         // For fabs
#include <linux/serial.h> // For serial comms functions
#include <sys/ioctl.h>    // For serial comms functions
#include <sys/stat.h>     // For the encryption key generator
#include <sodium.h>       // For password hashing
#include <syslog.h>       // For logging security notices on the system log

// For PARD Client (PC) comms with a PARDcapture server
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/poll.h>
#include <inttypes.h>
#include "pardcapdefs.h" // Some common definitions for client and server

// To set up ctrl-c interrupt handler
#include <signal.h>

// To get user input
#include <ctype.h>

// For PATH_MAX and INT_MAX
#include <linux/limits.h>
#include <limits.h>

// This is needed for the ZeroMQ network communications protocol
#include <zmq.h>

// For PARDUS-specific structs and #defines
#include "parddefs.h"


#endif
//---------------------------------------------------------------------------//.
 //                       Linux (original, unchanged)                       //.
  //-----------------------------------------------------------------------//.
#endif

// Safety net: PATH_MAX is defined by <linux/limits.h> (4096) on Linux, by
// <limits.h> on MinGW-w64 (260 = MAX_PATH) and by <sys/syslimits.h> (1024) on
// macOS. Should a tool-chain omit it, fall back to the Win32 MAX_PATH or 4096.
#ifndef PATH_MAX
    #if defined(MAX_PATH)
        #define PATH_MAX MAX_PATH
    #else
        #define PATH_MAX 4096
    #endif
#endif
//                                                                            //
///////                       END OF: HEADER FILES                     /////////
//////////                                                            //////////
////////////////////////////////////////////////////////////////////////////////

// _.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._. //


////////////////////////////////////////////////////////////////////////////////
//////////                                                            //////////
///////      CONSTANTS, GLOBAL VARIABLES and FUNCTION DECLARATIONS     /////////
//                                                                            //
#ifdef PARDUS
  //-----------------------------------------------------------------------\\.
 //                          OS portability layer                           \\.
//---------------------------------------------------------------------------\\.
// Declarations of the small helper functions (and their globals) that supply,
// on MS Windows and macOS, the operating-system services that the Linux code
// obtains directly from Linux/POSIX. Their definitions are in the 'OS
// portability layer' part of the FUNCTION DEFINITIONS section. Nothing here is
// compiled on Linux.
#ifdef PARDUS
#if IS_WINDOWS_NATIVE

// Win32 error code to text (the Windows counterpart of strerror()). Returns a
// pointer to a static buffer that is overwritten by the next call.
const char *win_errstr(DWORD err);

// Windows Event Log: the counterpart of openlog() / syslog() / closelog().
// Security notices go to the 'Application' log under the source name below.
#define WIN_EVENT_SOURCE  "pardserver" // Same identity tag as openlog() on Linux
#define WIN_EVENT_ID      1            // 1..1000 display cleanly if the source
                                       // is registered with eventcreate.exe
HANDLE win_EventSource = NULL;         // Handle from RegisterEventSourceA()
void win_openlog(const char *ident);
void win_syslog(WORD evtype, const char *fmt, ...);
void win_closelog(void);

// Process-wide start-up and shut-down of Windows services that have no Linux
// counterpart: Winsock 2 (WSAStartup/WSACleanup) and the console code page
// (UTF-8 so that the program's UTF-8 messages display correctly).
int  win_WsaStarted    = 0;            // 1 once WSAStartup() has succeeded
UINT win_SavedOutputCP = 0;            // Console output code page at start-up
int  win_platform_init(void);
void win_platform_shutdown(void);

// Privilege test - the counterpart of 'geteuid() == 0' (running as root).
// Returns 1 if the process runs with Administrator (elevated) or SYSTEM
// rights, 0 if it runs as an ordinary (unprivileged) user, -1 if this could
// not be determined.
int  win_privilege_level(void);

// High resolution relative sleep in units of 100 ns (used by msleep/musleep).
HANDLE win_SleepTimer = NULL;          // Re-usable waitable timer
int  win_sleep_100ns(LONGLONG ticks);

#elif defined(__APPLE__)

// Candidate serial devices for the macOS 'auto' Arduino search (see
// as_mac_list_candidates): every match of the glob(3) patterns, in the order
// in which they are tried.
#define AS_MAC_MAX_CANDIDATES 32
typedef struct {
  char path[PATH_MAX];     // Device node, e.g. /dev/cu.usbmodem14101
  int  rank;               // 0 = USB CDC-ACM (cf. /dev/ttyACM*), 1 = USB-serial
 } AsMacCandidate;         //     bridge (cf. /dev/ttyUSB*)
int as_mac_list_candidates(AsMacCandidate *cand, int maxcand);
int as_mac_try_port(const char *devpath);

#endif
//---------------------------------------------------------------------------//.
 //                          OS portability layer                           //.
  //-----------------------------------------------------------------------//.
#endif

  //-----------------------------------------------------------------------\\.
 //                            General utilities                            \\.
//---------------------------------------------------------------------------\\.
#ifdef PARDUS
#define PROGNAME argv[0]
#define SETTINGS argv[1]
#define ARGCSTD 2

int num_words(char *test, const char *sep);
int str_to_uchar(char *buf,unsigned char *ival);
int str_to_uint32(char *buf,uint32_t *ival);
int msleep(long msec);   // millisecond sleep
int musleep(long musec); // microsecond sleep
int bytechecksum(int benacted, int brequested, char *direction);
void tidy_up(void); 
void get_keyboard_input(char *receptstr, size_t receptlen);
uint8_t generate_version_string(char *vstring, size_t max_len);

int Pard_Err;             // A global error reporter
char Pard_Err_Str[256];   // To contain the error message formed in Pard_err_str
char *Pard_err_str(void); // Forms and returns a pointer to Pard_Err_Str
uint8_t Pard_Err_Checksum;// Alters the client if a checksum error has occured
                          // at the server side which may explain otherwise odd
                          // error messages recieved by the client such as 'info
                          // item not recognised' when in fact a valid info item
                          // sent but the Arduino serial connection was dodgy so
                          // could be properly communicated to it.

#define ERRMSG_LEN 512

// Error Message Defs. 
#define PARD_OK				0
// Values 0 to 63 are reserved for PARDU_ERR codes (errors from the Arduino).
// Do not use them for PARD_ERR constants.
#define PARD_ERR_MSDENOM	 64
#define PARD_ERR_AXIS   	 65
#define PARD_ERR_LIMA 	  	66
#define PARD_ERR_LIMU 	  	67
#define PARD_ERR_DIRN 	  	68
#define PARD_ERR_SUFBKL 	 69
#define PARD_ERR_NOHOME	 	71
#define PARD_ERR_NEG	     72
#define PARD_ERR_LIMSWU	  75
#define PARD_ERR_CALIB	   76
#define PARD_ERR_ARGC	    77
#define PARD_ERR_COORD	   78
#define PARD_ERR_IMDEV	   79
#define PARD_ERR_MEM	     80
#define PARD_ERR_ACNOM    81
#define PARD_ERR_ACMEMNOM 82
#define PARD_ERR_ACMEMLIM 83
#define PARD_ERR_ACMEMCAL 84
#define PARD_ERR_BUG      85
#define PARD_ERR_IMMATURE 86

#if IS_WINDOWS_NATIVE
//////// MS Windows: Ctrl+C interrupt handler via the Win32 console API ////////
//         (initialised by calling setSignalHandler(SIGINT, stop);)           //
//                                                                            //
// Windows has no sigaction(). The native mechanism is a console control      //
// handler (SetConsoleCtrlHandler) which the system calls in a NEW THREAD     //
// when Ctrl+C is pressed. setSignalHandler() keeps its Linux name and        //
// signature so that main() is unchanged: it registers a console control      //
// handler that forwards CTRL_C_EVENT to the supplied function as SIGINT.     //
// Ctrl+Break, closing the console window, log-off and shut-down are NOT      //
// intercepted (default action: the process ends), just as Linux intercepts  //
// only SIGINT (Ctrl+C) and leaves SIGQUIT, SIGHUP and SIGTERM at default.    //
//                                                                            //
// Global which, if set to 0 will cause the server loop to end                //
volatile sig_atomic_t sv_Server_run;                                          //
                                                                              //
static void sv_interrupt_stop(int signum)                                     //
{                                                                             //
 const char msg[] =                                                           //
       "\nCTRL+C INTERRUPT - SERVER WILL SHUT DOWN AT NEXT ITERATION...\n\n"; //
 DWORD unused_bytes = 0;                                                      //
 HANDLE herr = GetStdHandle(STD_ERROR_HANDLE);                                //
                                                                              //
 // WriteFile() bypasses the C run-time stream buffers and locks (the         //
 // equivalent of the async-signal-safe write(STDERR_FILENO,...) on Linux).   //
 if(herr != NULL && herr != INVALID_HANDLE_VALUE)                             //
   WriteFile(herr, msg, (DWORD)(sizeof(msg)-1), &unused_bytes, NULL);         //
 (void)unused_bytes; // Quietens any "unused variable" warnings safely.       //
 (void)signum;       // Always SIGINT here.                                   //
 sv_Server_run = 0; // This is to interrupt the server loop.                  //
 return;                                                                      //
}                                                                             //
                                                                              //
typedef void (*signalFunc_t) (int signum);                                    //
static signalFunc_t win_SigintHandler = NULL; // Function to call on Ctrl+C   //
                                                                              //
static BOOL WINAPI win_console_ctrl_handler(DWORD ctrl_type)                  //
// Called by Windows (in its own thread) for console control events.          //
// Returns TRUE if the event was handled here (the process carries on) or     //
// FALSE to pass it on to the next handler (ultimately ExitProcess()).        //
{                                                                             //
 if(ctrl_type == CTRL_C_EVENT && win_SigintHandler != NULL){                  //
   win_SigintHandler(SIGINT);                                                 //
   return TRUE;                                                               //
  }                                                                           //
 return FALSE;                                                                //
}                                                                             //
                                                                              //
static void setSignalHandler(int signum, signalFunc_t sigHandler)             //
{                                                                             //
 static int registered = 0; // main() calls this again on every restart, so  //
                            // register the console handler only once.        //
 if(signum != SIGINT) return; // Only SIGINT (= Ctrl+C) exists in a console.  //
 win_SigintHandler = sigHandler;                                              //
 if(!registered){                                                             //
   if(SetConsoleCtrlHandler(win_console_ctrl_handler, TRUE)) registered = 1;  //
  }                                                                           //
}                                                                             //
//                                                                            //
//////  End of MS Windows Ctrl+C interrupt handler (console API) functions //////
#else
///////////// This code set up a ctrl+c interrupt signal handler ///////////////
//         (initialised by calling setSignalHandler(SIGINT, stop);)           //
//                                                                            //
// Global which, if set to 0 will cause the server loop to end                //
volatile sig_atomic_t sv_Server_run;                                          // 
                                                                              //
static void sv_interrupt_stop(int signum)                                     //
{                                                                             //
 const char msg[] =                                                           //
       "\nCTRL+C INTERRUPT - SERVER WILL SHUT DOWN AT NEXT ITERATION...\n\n"; //
                                                                              //
 size_t unused_bytes = write(STDERR_FILENO, msg, sizeof(msg)-1);              //
 fflush(stderr);                                                              //
 (void)unused_bytes; // Quietens any "unused variable" warnings safely.       //
 sv_Server_run = 0; // This is to interrupt the server loop.                  //
 return;                                                                      //
}                                                                             //
                                                                              //
typedef void (*signalFunc_t) (int signum);                                    //
static void setSignalHandler(int signum, signalFunc_t sigHandler)             //
{                                                                             //
 struct sigaction new;                                                        //
 CLEAR(new);                                                                  //
 new.sa_handler = sigHandler;                                                 //
 sigaction(signum, &new, NULL);                                               //
}                                                                             //
//                                                                            //
//////////    End of ctrl+c  interrupt signal handler functions     ////////////
#endif

//---------------------------------------------------------------------------//.
 //                            General utilities                            //.
  //-----------------------------------------------------------------------//.
#endif

  //-----------------------------------------------------------------------\\.
 //                           Actuator Management                           \\.
//---------------------------------------------------------------------------\\.
#ifdef PARDUS

// Variables to manage the list of Stepper actuators
Act_StepperNode   *Stepperlist[STEPPERSMAX]; // For a list of stepper nodes
uint8_t            NSteppers=0;              // Number of nodes in the list
uint8_t            StepperID[STEPPERSMAX+2]; // List of stepper ID numbers
int8_t Act_Stepper_create(uint8_t,const char*);  // Creates a stepper actuator
int8_t Act_Stepper_destroy(uint8_t id); // Destroys a stepper actuator
void Act_Stepper_freecommon(uint8_t id); // Some common memory release code.

// Special flag for multiple steppers of the XYZ stage
uint8_t StageStatus = 0; // This will equal ACT_ID_STAGE_XY if both the X and Y
                         // motors are defined. It will equal ACT_ID_STAGE_XYZ
                         // if all three motors are defined. This does not take
                         // into account maturity - only existence.



// Variables to manage the list of Binary actuators
Act_BinaryNode    *Binarylist[BINARIESMAX];  // For a list of Binary nodes
uint8_t            NBinaries=0;              // Number of nodes in the list
uint8_t            BinaryID[BINARIESMAX];    // List of Binary ID numbers
int8_t Act_Binary_create(uint8_t,const char*);   // Creates a Binary actuator
int8_t Act_Binary_destroy(uint8_t id);      // Destroys a Binary actuator

// Variables to manage the list of Mlevel actuators
Act_MlevelNode    *Mlevellist[MLEVELSMAX];   // For a list of Mlevel nodes
uint8_t            NMlevels=0;               // Number of nodes in the list
uint8_t            MlevelID[MLEVELSMAX];     // List of Mlevel ID numbers
int8_t Act_Mlevel_create(uint8_t,const char*);   // Creates a Mlevel actuator
int8_t Act_Mlevel_destroy(uint8_t id);      // Destroys a Mlevel actuator

uint8_t mlevel_pins_have_gaps(uint8_t id);   // Helper for pin management
// Need to save the last known (LK) mlevel sent by the client because this gets
// interpreted prior to use and we need the original value for the purpose of
// syncing:
uint32_t LK_mlevel = 0;
uint32_t LK_freq = UINT32_MAX;
uint16_t  LK_duty = 5000;

// Variables to manage the list of Servo actuators
Act_ServoNode     *Servolist[SERVOSMAX];   // For a list of Servo nodes
uint8_t            NServos=0;              // Number of nodes in the list
uint8_t            ServoID[SERVOSMAX];     // List of Servo ID numbers
int8_t Act_Servo_create(uint8_t,const char*);  // Creates a Servo actuator
int8_t Act_Servo_destroy(uint8_t id);     // Destroys a Servo actuator


// Variables to manage the list of Sensor actuators
Act_SensorNode     *Sensorlist[SENSORSMAX]; // For a list of Sensor nodes
uint8_t            NSensors=0;              // Number of nodes in the list
uint8_t            SensorID[SENSORSMAX];    // List of Sensor ID numbers
int8_t Act_Sensor_create(uint8_t,const char*);  // Creates a Sensor actuator
int8_t Act_Sensor_destroy(uint8_t id);     // Destroys a Sensor actuator

// Common functions (for managing actuators of any type)
uint8_t actuator_exists(uint8_t actype,uint8_t actid);


// To receive information from a PARD Move Report (PMR)
uint32_t PMRep1,PMRep2,PMRep3,PMRep4;


//---------------------------------------------------------------------------//.
 //                           Actuator Management                           //.
  //-----------------------------------------------------------------------//.
#endif

  //-----------------------------------------------------------------------\\.
 //                         PARDCapture server comms                        \\.
//---------------------------------------------------------------------------\\.
#ifdef PARDUS

// Global to store data for current instruction (from pardcapdefs.h)
ps_InstructData ps_Instruction; 

#define PS_MAXDATASIZE 128 // max number of bytes we can get at once 
#define PS_MSGLEN 12       // Size of handshake from server

#define PARDCAP_TIMEOUT 600 // Limit for hard timeout error report in comms with
                            // any pardcapture server in seconds.
typedef struct {
 char ipaddr[16];
 char port[8];
 int  fd;
} ConnectNode;

ConnectNode* PSlist = NULL; // To store a list of connected PARDcap servers
unsigned int NPSs = 0;      // The number of servers in the list.
size_t    SizePSs = 0;      // The current size of the list

// The preview image pixel array dimensions
int32_t PreviewWd,PreviewHt;

int ps_fmtchoice(char *argstr); // Not the same as fmtchoice function in
                                // pardclient because that one includes the
                                // options of dou and do3 for formats returned
                                // to pardcap from pardclient.
// The 'save as' formats received from a client use only these
// values (they are #defined in pardcapdefs.h):
// SAF_YUYV 0  // Raw YUYV 16 bpp array
// SAF_YP5  1  // Y-only (greyscale) 8bpp pgm p5 format
// SAF_BM8  2  // Y-only (greyscale) 8bpp bmp format
// SAF_INT  3  // Insensity from RGB/YUYV raw doubles monochrome
// SAF_RGB  4  // Full colour RGB array saved in PPM p6 format
// SAF_BMP  5  // Full colour RGB array saved in 24bpp BMP format
// SAF_PNG  6  // Full colour PNG file
// SAF_JPG  7  // Full colour JPEG file

// Return values for ps_connect_new()
#define CNE_SUCCESS 0  // No error. New connection created OK
#define CNE_MEMFAIL 10 // Failed. Couldn't get memory for the new connection.
#define CNE_CONFAIL 11 // Failed. Couldn't connect to the server.
#define CNE_DUPNAME 12 // Failed. The proferred ip + port combo is already taken.

void   *ps_get_in_addr(struct sockaddr *sa);
int     ps_send_dc(int socketfd, char *databuff, int *datalen);
int     ps_connect(char *ps_ipaddress, char *ps_port);
void    ps_hangup(unsigned int idx);

#if IS_WINDOWS_NATIVE
// On Windows a socket is a SOCKET (UINT_PTR handle) rather than an int file
// descriptor. The shared code (and pardcapdefs.h, which is also used by the
// cross-platform pardclient) stores connections in int variables, with -1
// meaning 'not connected'. Microsoft guarantees that the kernel handles which
// back Winsock sockets have only 32 significant bits (they may be truncated
// and sign-extended without loss - see 'Interprocess Communication Between
// 32-bit and 64-bit Applications'), so the conversion below is exact, and
// INVALID_SOCKET maps to -1. ps_connect() refuses any handle above INT_MAX.
static inline SOCKET ps_win_socket(int fd)  { return (SOCKET)(intptr_t)fd; }
static inline int    ps_win_fd(SOCKET sock) { return (int)(intptr_t)sock; }

// The Winsock error code captured at the moment a socket call fails (Winsock
// reports errors via WSAGetLastError(), not errno). Read by the Windows
// version of ps_print_recv_error() and the connection error messages.
int ps_win_LastSockErr = 0;
#endif


//---------------------------------------------------------------------------//.
 //                         PARDCapture server comms                        //.
  //-----------------------------------------------------------------------//.
#endif

  //-----------------------------------------------------------------------\\.
 //                           Arduino serial comms                          \\.
//---------------------------------------------------------------------------\\.
#ifdef PARDUS

uint32_t CurrMsg; // Current message serial number
uint8_t  Vb1 = 0; // Current message system version number

char as_portErr=0;       // Serial port error type marker. See AS_ERR_ #defines
int  as_portName;        // File descriptor for the serial port
long as_portTimeout=750; // Timeout in ms of any one serial port transaction
int  as_connected;       // Flag so we know if we are connected to Arduino.
uint8_t as_NoWait=0;     // Whether to wait for serial input when checking.

#define AS_ERR_NONE   0  // All good. No error.
#define AS_ERR_TIME   1  // Time out error (no response for as_portTimeout ms).

#if IS_WINDOWS_NATIVE
// On Windows the serial (COM) port is a Win32 file HANDLE, not an int file
// descriptor, so the Windows serial functions use as_hCom in place of the
// as_portName descriptor used on Linux and macOS.
HANDLE as_hCom = INVALID_HANDLE_VALUE;

// Classes of serial port by kernel device name (registry SERIALCOMM list),
// used by the fall-back search when SetupAPI lists no port (see below).
#define AS_WIN_CLASS_NONE 0 // Not a USB serial port (motherboard UART, modem...)
#define AS_WIN_CLASS_ACM  1 // USB CDC-ACM (usbser.sys) - Arduino Uno/Mega etc.
#define AS_WIN_CLASS_USB  2 // USB-serial bridge: FTDI, CH34x, CP210x, PL2303

// Ranks of the candidate ports of the 'auto' search: lower ranks are tried
// first; within a rank the lower COM number is tried first.
#define AS_WIN_RANK_ARDUINO  0  // USB vendor ID of Arduino (2341, 2A03)
#define AS_WIN_RANK_ACM      1  // other USB CDC-ACM port (cf. /dev/ttyACM*)
#define AS_WIN_RANK_BRIDGE   2  // USB-serial bridge (cf. /dev/ttyUSB*): CH34x,
                                // FTDI, CP210x, PL2303
#define AS_WIN_RANK_OTHERUSB 3  // any other USB-attached serial port
#define AS_WIN_RANK_NONE    99  // never tried: motherboard UART, Bluetooth,
                                // modem, virtual (e.g. com0com) ports
#define AS_WIN_MAX_CANDIDATES 32
typedef struct {
  char port[16];            // "COM4"
  unsigned long num;        // 4
  int  rank;                // AS_WIN_RANK_...
  char desc[200];           // Friendly name and hardware ID, for the messages
 } AsWinCandidate;

int  as_win_write(const void *buf, DWORD nbytes);
void as_win_device_path(const char *port, char *devpath, size_t pathlen);
int  as_win_classify_device(const char *kernel_device_name);
int  as_win_rank_device(const char *hwid, const char *service,
                        const char *compatids, const char *enumerator);
int  as_win_list_candidates(AsWinCandidate *cand, int maxcand);
int  as_win_try_port(const char *port);
#endif

// Global order forms to be filled in for the various order type.
IDNode          Order_ID;        // Used for all orders
DirStepsNode    Order_DirSteps;  // Used for StageMove and StepperMove
MotionNode      Order_Motion;    // Used for StageMove and StepperMove
HomeNode        Order_Home;      // Used for StageHome and StepperHome
InfoNode        Order_Info;      // Used for Get and Set orders


int     as_open_comms_with_Arduino(char *);
size_t  as_read_bytes(char *buff,int length);
size_t  as_read_bnum(void *iptr, int ilen);
int     as_read_sigstr(char *receptbuff, char *expectstr);
int     as_send_order(void);
uint8_t as_check_move_status(uint8_t *actype, uint8_t *actid, uint8_t *pmerr,
                                                                        char *);

//---------------------------------------------------------------------------//.
 //                            Arduino serial comms                         //.
  //-----------------------------------------------------------------------//.
#endif

  //-----------------------------------------------------------------------\\.
 //                       Logging and local messaging                       \\.
//---------------------------------------------------------------------------\\.
#ifdef PARDUS
// I channel all messages to user via this one function. It allows me to control
// whether a GUI popup box should be used and/or whether the message should go
// to the command line (stdout) and/or to stderr and/or to the log file and/or
// be sent over the network - depending on global switches.
// When modifying the source code you should use this function instead of using
// printf / fprintf directly.
void show_message(const char *,const char *,int,int);

#define fnn __func__

int Verbosity = 2;    // Verbosity level, set on server startup config.
                      // Level 2 means show/log all messages.
                      // Level 1 means only show/log error messages.
                      // Level 0 means show/log security notices.

// Message types used with function show_message
#define MT_INFO   1 // Equivalent of printf - usually for info only
#define MT_ERR    2 // Equivalent of fprintf(sderr, - to indicate an error
#define MT_SEC    3 // As for MT_ERR but will print even in silent mode.
#define MT_QUIT   4 // Message given upon program quit request.
#define MT_SESS_S 5 // Message for a user session - from server
#define MT_SESS_C 6 // Message for a user session - from client
#define MT_SESS_T 7 // Message for a user session - translation into English
#define MT_SESS_E 8 // Message for a user session - communication error

char Log_SessionFilename[PATH_MAX];   // Name of file used to store log data
                                      // pertaining to a logged-in user session.
FILE* FP_Sessionlog;                  // For the user session log file. Always
                                      // set to NULL when not in use so it can
                                      // also be used as a flag to let us know.

void close_session_logfile(void);
void open_session_logfile(void);



//---------------------------------------------------------------------------//.
 //                       Logging and local messaging                       //.
  //-----------------------------------------------------------------------//.
#endif

  //-----------------------------------------------------------------------\\.
 //                          Server-side Auto-focus                         \\.
//---------------------------------------------------------------------------\\.
#ifdef PARDUS

int Autofocussing;       // Flag that gets set when the autofocus procedure is
                         // triggered and unset when it ends.
double   AF_issz;        // Initial step size for AF search.
double   AF_centre;      // Current best focus Z position.
uint32_t AF_max_retries; // The maximum number of attempts AF makes to get best
                         // focus if it fails first time.
uint32_t AF_max_itn;     // The maximum number of iterations in an AF hill-
                         // climbing search before giving up. Must be >1. 
long AF_timeout;         // Maximum allowed time (in ms) for the AF search.
                         // 0 means no time limit.
double AF_maxdiff;       // The maximum difference between current and found
                         // 'best focus' Z position to allow before AF failure is
                         // declared (0= no limit). 
double AF_lk_maxdiff;    // The last known maxdiff value found in an autofocus
                         // search - used to provide feedback to client after AF
                         // failure.
double AF_min_peak;      // The minimum size of a true maximum in the AF
                         // function. Must not be negative. Optimum value is 
                         // found empirically. 
double AF_lk_peak;       // The last known maximum value of the AF function found
                         // in an autofocus search - used to provide feedback to
                         // the client after an AF 'false peak' failure.
double AF_tc_threshlow;  // Tissue content lower threshold value
double AF_tc_threshupp;  // Tissue content upper threshold value
double AF_tc_curr;       // The current tissue content value
uint8_t AF_ref_set;      // This is set to 1 if 'get_preview_tc OFF' was run
uint8_t AF_colchan;      // Colour channel to get use for focus values
uint8_t AF_dirn_up;      // What direction (0 or 1) moves the stage up.

#define USING_TC_THRESHOLDING (AF_tc_threshlow >= 0.0 && AF_ref_set)

// Values returned by the auto_focus procedure:
#define AF_SUCCESS      0 // All went well (as far the AF processes can tell).
#define AF_IMPOSSIBLE   1 // Failure due to inability to even start the AF
                          // search (e.g. no camera stream detected) - no point
                          // re-trying until the issue is fixed
#define AF_FALSEPEAK    2 // AF went to completion but probably has reached a
                          // false focal point - might succeed if the procedure
                          // is iterated without alteration.
#define AF_CCRITERR     3 // AF was initiated but the iterative loop terminated
                          // in an abnormal way - might succeed if the procedure
                          // is iterated with some paramters modified.
#define AF_CCRITITN     4 // AF was initiated but the iterative loop terminated
                          // due to reaching the limit of allowed iterations -
                          // might succeed if the procedure is iterated with
                          // some paramters modified.
#define AF_CCRITMDF     5 // AF 'succeeded' but the new best value for Z
                          // exceeded the previous one by more than the allowed
                          // amount - might succeed if the procedure is iterated
                          // with some paramters modified.
#define AF_CCRITTMO     6 // AF was initiated but failed to conclude within the
                          // maximum allowed time - might succeed if the
                          // procedure is iterated with some paramters modified.
#define AF_MOVEERR      7 // An error was returned from the movement routines
                          // when trying to move the Z axis.

// Values for AF failure choice
#define AFFA_IGNORE     1
#define AFFA_EXIT       2

void af_vars_init(void);
uint8_t af_get_current_tc(char *errmsg);
uint8_t af_get_current_focus(char *errmsg, double *fval, uint8_t colchan);
int af_tc_threshold_check(void);
int af_auto_focus(char *aferr);


//---------------------------------------------------------------------------//.
 //                          Server-side Auto-focus                         //.
  //-----------------------------------------------------------------------//.
#endif

  //-----------------------------------------------------------------------\\.
 //                             The PARD Server                             \\.
//---------------------------------------------------------------------------\\.
#ifdef PARDUS

// Global server variables
#define TXT_BUFMAX 128 // Maximum size of text buffers for the message header
#define TRIADMAX   640 // Maximum size of the sv_Triad (main text message sent
                       // and received). Although the sv_Triad is dynamically
                       // allocated, I set an upper limit to avoid accidentally
                       // allocating a sv_Blob to the sv_Triad and this serves
                       // as another check on correct message formulation.
                       // The sv_Triad has enough room for any message pardus
                       // and client might send to each other including a
                       // standard file name (but no path names are allowed).

char sv_Msghdr[3][TXT_BUFMAX]; // Header information for messages sent/received.
char *sv_Triad=NULL;    // Textual component of a message (not the header/blob).
char *sv_Triadarg=NULL; // To get a selected argument from the sv_Triad string
                        // using char *sv_triad_argv(int);
void *sv_Blob=NULL;     // For the blob.
size_t sv_Blobsize;     // Size of the blob in bytes
char sv_Username[TXT_BUFMAX],sv_Pardname[TXT_BUFMAX],sv_Client_id[TXT_BUFMAX];
int sv_Client_logged_in;// Flag indicating if a client is logged into the server
int sv_Mdx,sv_Climdx;   // Message index for server and client
int sv_Dont_update_Mdx; // A flag that allows a function to re-send a message
                        // with the same message index.
int sv_Merrdx;          // Allow the server to close (or reboot) after
int sv_Merrmax;         // sv_Merrmax comms errors.
// int sv_Server_run;   // Defined above under Ctrl+C interrupt handler. This
                        // global, if set to 0, will cause the server loop 
                        // to end (and so transfer control back to the local
                        // pardserver user rather than remote users).
int sv_Wrong_pard;      // Flag that is set if a message was received for the
                        // wrong server (i.e. not addressed to this instance of
                        // pardserver).                     
int sv_Wrong_client;    // Flag that is set if a message was sent by a user
                        // other than the one currently logged in. 
int sv_Wrong_version;   // Flag that is set to a negative number if there is a
                        // misunderstanding - see sv_receive_data_zmq().
int sv_Resendidx;       // For primitive handling of client error reports. See
int sv_Resendmax;       // sv_receive_data_zmq().                        
int sv_Previous_blob;   // Flag set if a sv_Blob was sent (useful if you need to
                        // resend last message). 

int Pardport = 5555;    // The port the server will be on


// Registered users credentials management
#define MAX_USER_LEN  64
#define MAX_PASS_LEN  256
#define MAX_TOTAL_USERS 512


// Security policy globals with their default value set to high. The sysadmin
// can change these to even higher or lower security value by means of settings
// in the server config file:

// Number of wrong password attempts before being struck out:
uint8_t Strike_limit = 3;

// Time in seconds to re-instate a struck out user (900 = 15 minutes) 
time_t  Strikeout_delay = 900;

// Time in seconds for a password to expire and need resetting (90 days)
time_t Password_expiry_timeout = 7776000; 

// Minimum password length in bytes:
size_t Password_min_length = 12;

// Password complexity requirements
uint8_t Pwd_needs_upper   = 1;
uint8_t Pwd_needs_lower   = 1;
uint8_t Pwd_needs_digit   = 1;
uint8_t Pwd_needs_special = 1;



typedef struct {
    char username[MAX_USER_LEN];
    char password[MAX_PASS_LEN];
    uint8_t  strike_count;
    time_t   lockout_until;
    time_t   time_created;
    uint8_t  single_use;  
} UserCred;

// Global tracking registers for dynamically loaded system users
UserCred *sv_UserDb = NULL;
int       sv_Upllen = 0; // Tracks the current active number of loaded users

// File for the user database
char sv_Userfile[PATH_MAX];

// ZeroMQ metadata strings look like "tcp://192.168.1.1:54321". 64 bytes is OK.
char sv_Client_IP[64] = "Unknown IP"; 


// Global buffers to store the server credentials in RAM
char server_pub[41] = {0};
char server_sec[41] = {0};


// Motion-related arrays (each member is for a different actuator type)
 uint8_t sv_Unitype[6];
 AccPNode sv_Accparam[6];

// For the PARD Motion Report state machine
uint8_t sv_PMR_null,sv_PMR_null_actype,sv_PMR_null_actid;

// Message Error States used by the function sv_update_message_header
#define sv_MES_NONE     1 // A normal message to a logged in user (no error)
#define sv_MES_ANON_ERR 2 // An error message sent to a non-logged caller
#define sv_MES_USER_ERR 3 // An error message sent to a logged-in user

// sv_Triad Type constants
#define sv_TT_UNDEF    0
#define sv_TT_OK       1
#define sv_TT_ERROR    2

// sv_Triad State constants
#define sv_TS_UNDEF    0
#define sv_TS_READY    1
#define sv_TS_BYE      2

// Return values for sv_switch_string are in pardcapdefs.h

// Values for cmdstatus
#define CMD_OK  1
#define CMD_ERR 2

// Return values for sv_process_cuss
#define CUSS_VAL_INT1 1 // The cmdval string contains an integer to go in ival1
#define CUSS_VAL_DOU1 2 // The cmdval string contains an double to go in dval1
#define CUSS_VAL_FRMT 3 // The cmdval string contains an format string which
                        // will be converted to a uin8_t to go in safmt
#define CUSS_VAL_BYTE 4 // The cmdval string contains an uin8_t to go in safmt
#define CUSS_VAL_ONFF 5 // The cmdval string contains "ON" or OFF" to be
                        // interpreted as a value of 1 or 0 to go in ival1
#define CUSS_VAL_ONFT 6 // The cmdval string contains "ON", OFF" or "FITS"
#define CUSS_VAL_NONE 7 // The cmdval string is ignored and may be NULL

// Return values for sv_wait_for_pmr
#define SV_PMR_FOUND    1
#define SV_PMR_TIMEOUT  0
#define SV_PMR_ERROR    2


int        sv_process_cuss(uint8_t siid, char *cmdnom, char *cmdval,
             char *errmsg,char *cmdtriad, int *sending_bloberr,uint8_t valtype);
uint8_t    sv_moveto_to_moveby(uint8_t id, uint32_t *dirsteps, char *errmsg,
                          uint8_t *reqdirn, uint8_t *infotype, double position);
double     sv_posdist_to_maxmsdenomsteps(uint8_t id, double posdist);
int        sv_stepper_move_to(uint8_t id, double posn, uint32_t duration,
                                                                  char *errmsg);
int        sv_stepper_move_by(uint8_t id, uint32_t dirsteps, uint32_t duration,
                                                                  char *errmsg);
int        sv_stepper_set_dirn(uint8_t id, uint8_t dirn, char *errmsg);
int        sv_wait_for_pmr(double timeout_musecs, char *errmsg);
int        sv_send_message(void *socket,int have_blob);
int        sv_receive_message(void *socket,int *have_blob);
int        sv_transmit_data_zmq(void *socket,int have_blob);
static int sv_recv_data(void *socket,int *have_blob);
static int sv_receive_data_zmq(void *socket);
void       sv_update_message_header(int msg_error_state);

int   sv_triad_set(char *tstring);
int   sv_triad_resize(size_t bsz);
int   sv_triadarg_resize(size_t bsz);
int   sv_triad_argc(void);
char *sv_triad_argv(size_t narg);
int   sv_triad_type(void);
int   sv_triad_state(void);

int  sv_blob_resize(size_t bsz);

int  sv_match_credentials(char *unamestr, char *passwdstr,int *udx);
int  sv_is_password_strong(const char *password, char *errmsg); 
int  sv_change_password(const char *current_plaintext, const char *new_plaintext, int user_index,char *errmsg);
int  sv_save_user_file(const char *user_file_path);
void sv_reset_strike_counts(void);
void sv_set_log_off_flags(void);
void sv_log_off_client(void *socket,char *reason,int due_to_error);
void sv_client_id_gen(char *unamestr, char *sv_Client_id);

int sv_pard_server(int port);
int sv_switch_string(const char *sst);
int sv_evalonoff(const char *sst);
uint8_t sv_file_to_safmt(char *ffile);
int  sv_init_or_load_keys(const char *key_file_path);
int  sv_load_user_file(const char *user_file_path);
int  sv_setup_via_file(const char *config_file_path);
int  sv_build_user_database(const char *filepath);
int  sv_gen_keys(const char *filepath);

//---------------------------------------------------------------------------//.
 //                             The PARD Server                             //.
  //-----------------------------------------------------------------------//.
#endif



#endif
//                                                                            //
/////// END OF: CONSTANTS, GLOBAL VARIABLES and FUNCTION DECLARATIONS  /////////
//////////                                                            //////////
////////////////////////////////////////////////////////////////////////////////

// _.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._. //


////////////////////////////////////////////////////////////////////////////////
//////////                                                            //////////
///////                               MAIN                             /////////
//                                                                            //

#if IS_WINDOWS_NATIVE
int main(int argc, char *argv[])
// MS Windows version of main(). It is identical to the Linux main() except:
//  1. The Linux syslog connection (openlog) is replaced by win_platform_init()
//     which starts Winsock 2 (needed before any PARDcapture socket call),
//     sets the console output code page to UTF-8 and opens the Windows Event
//     Log source used for security notices.
//  2. The 'running as root' test (geteuid() == 0) is replaced by the Windows
//     equivalent: running with Administrator (elevated) or SYSTEM rights. If
//     that cannot be determined the server refuses to start (fails closed).
//  3. The 200 ms pause before a restart uses the Win32 Sleep().
{
 char invalue[16];
 int verbackup;
 size_t inlen;
 uint8_t ui8;
 char ver_num_str[32] = {0};
 char keyfile[PATH_MAX] = {0};
 uint8_t veresult;
 int main_retval = 0;


 veresult = generate_version_string(ver_num_str, sizeof(ver_num_str));

 if(argc>ARGCSTD){
   startup_help_message:
   fprintf(stderr,
   "\nPARDserver, Copyright (c) 1994-2026 Dr P. J. Tadous"
   "\nVersion %s"
   "\nThis program is open source free software released under license"
   "\n(GPL v.3 or later) and comes WITHOUT ANY WARRANTY."
   "\nFor the full license see <https://www.gnu.org/licenses/>\n",veresult?
   "[Version string failed to generate]":ver_num_str);
   fprintf(stderr,"\nUsage: %s [settings_file]\n\n",PROGNAME);
   fprintf(stderr,
   "If no [settings_file] is supplied, interactive setup will begin.\n\n");
   exit(1);
  }

 // Some initialisations:

 ps_CliSrv_connected=0;
 as_connected=0;
 FP_Sessionlog = NULL;
 sv_Client_logged_in=0;
 sv_Dont_update_Mdx = 0;
 ps_initialise_instruction();
 if(ps_initialise_blob()==0){
   show_message(": Could not allocate RAM for the PARDcap server blob.",
                                                                 fnn,MT_ERR,0);
   return 1;
  }

 // Initialise Stepper Actuator list IDs and pointers
 for(ui8=0;ui8<STEPPERSMAX;ui8++){
   StepperID[ui8]=STEPPER_ID_FREE;
   Stepperlist[ui8]=NULL;
  }
 StageStatus=0;

 // Initialise Binary Actuator list IDs and pointers
 for(ui8=0;ui8<BINARIESMAX;ui8++){
   BinaryID[ui8]=BINARY_ID_FREE;
   Binarylist[ui8]=NULL;
  }

 // Initialise MLevel Actuator list IDs and pointers
 for(ui8=0;ui8<MLEVELSMAX;ui8++){
   MlevelID[ui8]=MLEVEL_ID_FREE;
   Mlevellist[ui8]=NULL;
  }

 // Initialise servo Actuator list IDs and pointers
 for(ui8=0;ui8<SERVOSMAX;ui8++){
   ServoID[ui8]=SERVO_ID_FREE;
   Servolist[ui8]=NULL;
  }

 // Initialise sensor Actuator list IDs and pointers
 for(ui8=0;ui8<SENSORSMAX;ui8++){
   SensorID[ui8]=SENSOR_ID_FREE;
   Sensorlist[ui8]=NULL;
  }

 // Initialise autofocus parameters
 af_vars_init();

 // Allocate memory for server sv_Triad and the argument pointer - this is just
 // an initial allocation because I always free(sv_Triad); before doing a new
 // allocation.
 sv_Triad=calloc(2,sizeof(char));
 if(sv_Triad==NULL){
  show_message(": Could not allocate RAM for the server sv_Triad.",fnn,MT_ERR,0);
  return 1;
 }
 sv_Triadarg=calloc(2,sizeof(char));
 if(sv_Triadarg==NULL){
  show_message(": Could not allocate RAM for the server sv_Triad argument pointer.",fnn,MT_ERR,0);
  return 1;
 }
 // Allocate memory for sv_Blob - this is just an initial allocation
 // because I always free(sv_Blob); before doing a new allocation.
 sv_Blobsize=2;
 sv_Blob=calloc(sv_Blobsize,sizeof(char));
 if(sv_Blob==NULL){
  show_message(": Could not allocate RAM for the sv_Blob.",fnn,MT_ERR,0);
  return 1;
 }

 // Preview dims
 PreviewWd = 640;
 PreviewHt = 480;

 // Initialise serial message number counter:
 CurrMsg=0;

 // Initialise the Windows platform services (the counterpart of the Linux
 // openlog() call): Winsock 2, UTF-8 console output and the Event Log source
 // (identity tag "pardserver", Application log) for security notices.
 if(win_platform_init()){
   fprintf(stderr, "\nCRITICAL ERROR: Could not initialise the Windows socket"
                   " (Winsock 2) library.\n\n");
   main_retval = 1; Verbosity = 0; goto end_server_prog;
  }


 if(argc==ARGCSTD){
   if(!(strncmp(SETTINGS,"-h",2)) || !(strncmp(SETTINGS,"--help",6)))
                                                      goto startup_help_message;
   else if(!(strncmp(SETTINGS,"-v",2)) || !(strncmp(SETTINGS,"--version",9))){
       printf("\nPARDserver version %s\n\n",veresult?
                             "[Version string failed to generate]":ver_num_str);
     main_retval = 0; Verbosity = 0; goto end_server_prog;
    }
   // A settings file was provided so attempt setup via the file - but only if
   // the program does not have Administrator (elevated) privileges.

   // =========================================================================
   // SECURITY AUDIT: PREVENT THE DAEMON FROM RUNNING AS ADMINISTRATOR/SYSTEM
   // =========================================================================
   if (win_privilege_level() != 0) { // 1 = elevated, -1 = unknown: fail closed
       fprintf(stderr, "\nCRITICAL ERROR: PARDserver cannot be run with Administrator (elevated) or SYSTEM\n");
       fprintf(stderr, "                privileges for security reasons.\n");
       fprintf(stderr, "                Please launch the network daemon using an unprivileged account:\n");
       fprintf(stderr, "                > runas /user:pardserver \"\\\"C:\\Program Files\\PARDserver\\pardserver.exe\\\" [settings_file]\"\n\n");

       main_retval = 1;
       Verbosity = 0;
       goto end_server_prog; // Cleanly dump RAM structures before bailing out
   }
   // =========================================================================


   if(sv_setup_via_file(SETTINGS) == 0) goto start_the_server;
   else {main_retval = 1; Verbosity = 0; goto end_server_prog;}
  }

 show_message("","",MT_INFO,0);
 show_message("#####################################################","",MT_INFO,0);
 show_message("#                                                   #","",MT_INFO,0);
 // Print the version number banner line to the same aligned width as the rest
 if (veresult == 0)  {
     // Create the full banner line dynamically
     char banner_line[128] = {0};

     char text_prefix[64] = {0};
     snprintf(text_prefix, sizeof(text_prefix), "PARDserver v%s", ver_num_str);

     // The inner padding field is exactly 49 characters wide
     const int target_inner_width = 49;

     // Build the final line: %-*s pads out the remainder of the 49-char box
     // with spaces
     snprintf(banner_line, sizeof(banner_line), "#  %-*s#",
              target_inner_width, text_prefix);

     // Pass the perfectly aligned string to the display function
     show_message(banner_line, "", MT_INFO, 0);
 }
 else {
     // Fallback error-handling banner if the version function fails
 show_message("#  PARDserver [Version Error]                       #", "",
                                                                    MT_INFO, 0);
  }

 show_message("#                                                   #","",MT_INFO,0);
 show_message("#  Welcome administrator. Please supply the setup   #","",MT_INFO,0);
 show_message("#  information required by the following prompts.   #","",MT_INFO,0);
 show_message("#                                                   #","",MT_INFO,0);
 show_message("#####################################################","",MT_INFO,0);

 printf("\n");

 // Only allow encryption key generation option if the user has Administrator
 // (elevated) privileges - the Windows counterpart of root privileges.
 if (win_privilege_level() == 1){
   // Generate encryption keys file
   show_message("? ","  > Would you like to generate encryption keys (<key_file>/N)",MT_INFO,1);
   // Get <file>/N input from user
   get_keyboard_input(keyfile, sizeof(keyfile));
   inlen = strlen(keyfile);
   if(strcmp(keyfile,"n") && strcmp(keyfile,"N") && inlen>=1){
      main_retval = sv_gen_keys(keyfile);
      Verbosity = 0; goto end_server_prog;
     }
 }


 // Add or manage users to/in the user database file
 show_message("? ","  > Add or manage users? (<user_database_file>/N)",MT_INFO,1);
 // Get <file>/N input from user
 get_keyboard_input(sv_Userfile, sizeof(sv_Userfile));
 inlen = strlen(sv_Userfile);
 if(strcmp(sv_Userfile,"n") && strcmp(sv_Userfile,"N") && inlen>=1){
   main_retval = sv_build_user_database(sv_Userfile);
   Verbosity = 0; goto end_server_prog;
  } else {
   printf("\nPARDserver says: Bye!\n\n");
   main_retval = 0;
   Verbosity = 0;
   goto end_server_prog; // Cleanly dump RAM structures before bailing out
  }



 start_the_server:
 // Start the main PARD server:
 // 1. Set the 'let the server loop continue' flag to 1
       sv_Server_run=1;
 // 2. Register the ctrl+c override handler function (see 'Note' below)
       setSignalHandler(SIGINT, sv_interrupt_stop);
 // 3. Call the server infinite loop function
       sv_pard_server(Pardport);
 // Note: On Windows the console control handler stays registered for the life
 // of the process (it is registered once only). A Ctrl+C pressed while this
 // program waits at the 'Selection:' prompt below ends that console read (as
 // end-of-file) so, as on Linux, a second Ctrl+C terminates the program.

 verbackup = Verbosity; // Backup Verbosity in case the server re-starts
 Verbosity = 2;

 // Reset any blocked user's strike count
 sv_reset_strike_counts();

 show_message("#####################################################","",MT_INFO,0);
 show_message("#                                                   #","",MT_INFO,0);
 show_message("# Server functions have ceased but connections are  #","",MT_INFO,0);
 show_message("# still active.                                     #","",MT_INFO,0);
 show_message("#                                                   #","",MT_INFO,0);
 show_message("# You should close any pardcap servers before       #","",MT_INFO,0);
 show_message("# exiting this program entirely (which will close   #","",MT_INFO,0);
 show_message("# all connections).                                 #","",MT_INFO,0);
 show_message("#                                                   #","",MT_INFO,0);
 show_message("# Press [Return] to close all connections and exit, #","",MT_INFO,0);
 show_message("# or type 'r' and press [Return] to RESTART server. #","",MT_INFO,0);
 show_message("#                                                   #","",MT_INFO,0);
 show_message("#####################################################","",MT_INFO,0);
 show_message("\nSelection: ","",MT_INFO,1);

 // Use get_keyboard_input for robust, overflow-protected terminal reading
 get_keyboard_input(invalue, sizeof(invalue));

 // Check the first element of the character array
 if(invalue[0] == 'r' || invalue[0] == 'R'){
   show_message("\n>>> Re-arming and restarting PARD server... <<<\n\n", "",
                                                                    MT_INFO, 0);

   // Standard OS precaution: A brief 200ms delay helps clear any lingering
   // socket states in the Winsock TCP stack before the new bind request.
   Sleep(200);

   Verbosity = verbackup; // Re-enstate the user's choice for Verbosity

   goto start_the_server;
  }

 end_server_prog:

 tidy_up();

	return main_retval;
}
#else
int main(int argc, char *argv[])
{
 char invalue[16];
 int verbackup;
 size_t inlen;
 uint8_t ui8;
 char ver_num_str[32] = {0};
 char keyfile[PATH_MAX] = {0};
 uint8_t veresult;
 int main_retval = 0;


 veresult = generate_version_string(ver_num_str, sizeof(ver_num_str));

 if(argc>ARGCSTD){
   startup_help_message:
   fprintf(stderr,
   "\nPARDserver, Copyright (c) 1994-2026 Dr P. J. Tadous"
   "\nVersion %s"
   "\nThis program is open source free software released under license"
   "\n(GPL v.3 or later) and comes WITHOUT ANY WARRANTY."
   "\nFor the full license see <https://www.gnu.org/licenses/>\n",veresult?
   "[Version string failed to generate]":ver_num_str);  
   fprintf(stderr,"\nUsage: %s [settings_file]\n\n",PROGNAME);
   fprintf(stderr,
   "If no [settings_file] is supplied, interactive setup will begin.\n\n");
   exit(1);
  }

 // Some initialisations:

 ps_CliSrv_connected=0;
 as_connected=0;
 FP_Sessionlog = NULL;
 sv_Client_logged_in=0;
 sv_Dont_update_Mdx = 0;
 ps_initialise_instruction();
 if(ps_initialise_blob()==0){
   show_message(": Could not allocate RAM for the PARDcap server blob.",
                                                                 fnn,MT_ERR,0);
   return 1;
  }

 // Initialise Stepper Actuator list IDs and pointers
 for(ui8=0;ui8<STEPPERSMAX;ui8++){
   StepperID[ui8]=STEPPER_ID_FREE;
   Stepperlist[ui8]=NULL;
  }
 StageStatus=0;

 // Initialise Binary Actuator list IDs and pointers
 for(ui8=0;ui8<BINARIESMAX;ui8++){
   BinaryID[ui8]=BINARY_ID_FREE;
   Binarylist[ui8]=NULL;
  }

 // Initialise MLevel Actuator list IDs and pointers
 for(ui8=0;ui8<MLEVELSMAX;ui8++){
   MlevelID[ui8]=MLEVEL_ID_FREE;
   Mlevellist[ui8]=NULL;
  }

 // Initialise servo Actuator list IDs and pointers
 for(ui8=0;ui8<SERVOSMAX;ui8++){
   ServoID[ui8]=SERVO_ID_FREE;
   Servolist[ui8]=NULL;
  }

 // Initialise sensor Actuator list IDs and pointers
 for(ui8=0;ui8<SENSORSMAX;ui8++){
   SensorID[ui8]=SENSOR_ID_FREE;
   Sensorlist[ui8]=NULL;
  }

 // Initialise autofocus parameters
 af_vars_init();

 // Allocate memory for server sv_Triad and the argument pointer - this is just 
 // an initial allocation because I always free(sv_Triad); before doing a new
 // allocation.
 sv_Triad=calloc(2,sizeof(char));
 if(sv_Triad==NULL){
  show_message(": Could not allocate RAM for the server sv_Triad.",fnn,MT_ERR,0);
  return 1;
 }
 sv_Triadarg=calloc(2,sizeof(char));
 if(sv_Triadarg==NULL){
  show_message(": Could not allocate RAM for the server sv_Triad argument pointer.",fnn,MT_ERR,0);
  return 1;
 }
 // Allocate memory for sv_Blob - this is just an initial allocation
 // because I always free(sv_Blob); before doing a new allocation.
 sv_Blobsize=2;
 sv_Blob=calloc(sv_Blobsize,sizeof(char));
 if(sv_Blob==NULL){
  show_message(": Could not allocate RAM for the sv_Blob.",fnn,MT_ERR,0);
  return 1;
 }

 // Preview dims
 PreviewWd = 640;
 PreviewHt = 480;

 // Initialise serial message number counter:
 CurrMsg=0;

 // Initialize the Linux Syslog Connection
 openlog(
     "pardserver",                 // The identity tag printed next to every log entry
     LOG_PID | LOG_CONS,           // Flags: Include Process ID, fall back to console on error
     LOG_DAEMON                    // Facility: Classifies this as system daemon traffic
 );

 // Ignore SIGPIPE (added 14.11.26). A send() to a PARDcapture server that has
 // closed its connection would otherwise raise SIGPIPE, whose default action
 // terminates pardserver at once - without the orderly shut-down and with the
 // loss of the Arduino state. With SIGPIPE ignored, send() fails with EPIPE,
 // which ps_send_dc() already reports as an error. (ZeroMQ protects its own
 // sockets; MS Windows has no SIGPIPE. This main() is used by Linux and macOS.)
 signal(SIGPIPE, SIG_IGN);


 if(argc==ARGCSTD){
   if(!(strncmp(SETTINGS,"-h",2)) || !(strncmp(SETTINGS,"--help",6)))
                                                      goto startup_help_message;
   else if(!(strncmp(SETTINGS,"-v",2)) || !(strncmp(SETTINGS,"--version",9))){
       printf("\nPARDserver version %s\n\n",veresult?
                             "[Version string failed to generate]":ver_num_str);
     main_retval = 0; Verbosity = 0; goto end_server_prog;
    }
   // A settings file was provided so attempt setup via the file - but only if
   // the program does not have root privileges.

   // =========================================================================
   // SECURITY AUDIT: PREVENT THE DAEMON FROM RUNNING AS ROOT
   // =========================================================================
   if (geteuid() == 0) { // 
       fprintf(stderr, "\nCRITICAL ERROR: PARDserver cannot be run as the root user for security reasons.\n");
       fprintf(stderr, "                  Please launch the network daemon using an unprivileged account:\n");
       fprintf(stderr, "                  $ sudo -u pardserver pardserver [settings_file]\n\n");
       
       main_retval = 1; 
       Verbosity = 0; 
       goto end_server_prog; // Cleanly dump RAM structures before bailing out
   }
   // =========================================================================


   if(sv_setup_via_file(SETTINGS) == 0) goto start_the_server;
   else {main_retval = 1; Verbosity = 0; goto end_server_prog;} 
  }

 show_message("","",MT_INFO,0);
 show_message("#####################################################","",MT_INFO,0);
 show_message("#                                                   #","",MT_INFO,0);
 // Print the version number banner line to the same aligned width as the rest
 if (veresult == 0)  {
     // Create the full banner line dynamically
     char banner_line[128] = {0};
     
     char text_prefix[64] = {0};
     snprintf(text_prefix, sizeof(text_prefix), "PARDserver v%s", ver_num_str);
     
     // The inner padding field is exactly 49 characters wide
     const int target_inner_width = 49;
     
     // Build the final line: %-*s pads out the remainder of the 49-char box
     // with spaces
     snprintf(banner_line, sizeof(banner_line), "#  %-*s#", 
              target_inner_width, text_prefix);
 
     // Pass the perfectly aligned string to the display function
     show_message(banner_line, "", MT_INFO, 0);
 }
 else {
     // Fallback error-handling banner if the version function fails
 show_message("#  PARDserver [Version Error]                       #", "",
                                                                    MT_INFO, 0);
  }

 show_message("#                                                   #","",MT_INFO,0);
 show_message("#  Welcome administrator. Please supply the setup   #","",MT_INFO,0);
 show_message("#  information required by the following prompts.   #","",MT_INFO,0);
 show_message("#                                                   #","",MT_INFO,0);
 show_message("#####################################################","",MT_INFO,0);

 printf("\n");

 // Only allow encryption key generation option if user has root privileges
 if (geteuid() == 0){
   // Generate encryption keys file
   show_message("? ","  > Would you like to generate encryption keys (<key_file>/N)",MT_INFO,1);
   // Get <file>/N input from user
   get_keyboard_input(keyfile, sizeof(keyfile));
   inlen = strlen(keyfile);
   if(strcmp(keyfile,"n") && strcmp(keyfile,"N") && inlen>=1){
      main_retval = sv_gen_keys(keyfile);
      Verbosity = 0; goto end_server_prog;
     }
 }


 // Add or manage users to/in the user database file
 show_message("? ","  > Add or manage users? (<user_database_file>/N)",MT_INFO,1);
 // Get <file>/N input from user
 get_keyboard_input(sv_Userfile, sizeof(sv_Userfile));
 inlen = strlen(sv_Userfile);
 if(strcmp(sv_Userfile,"n") && strcmp(sv_Userfile,"N") && inlen>=1){
   main_retval = sv_build_user_database(sv_Userfile); 
   Verbosity = 0; goto end_server_prog;
  } else {
   printf("\nPARDserver says: Bye!\n\n");
   main_retval = 0; 
   Verbosity = 0; 
   goto end_server_prog; // Cleanly dump RAM structures before bailing out
  }



 start_the_server:
 // Start the main PARD server:
 // 1. Set the 'let the server loop continue' flag to 1
       sv_Server_run=1;
 // 2. Register the ctrl+c override signal handler function (see 'Note' below) 
       setSignalHandler(SIGINT, sv_interrupt_stop); 
 // 3. Call the server infinite loop function
       sv_pard_server(Pardport);
 // Note: The library only overrides the SIGINT on the first occasion that the
 // signal is encountered. After that the default action is re-instated for any
 // subsequent SIGINT signals received.

 verbackup = Verbosity; // Backup Verbosity in case the server re-starts
 Verbosity = 2;

 // Reset any blocked user's strike count
 sv_reset_strike_counts();

 show_message("#####################################################","",MT_INFO,0);
 show_message("#                                                   #","",MT_INFO,0);
 show_message("# Server functions have ceased but connections are  #","",MT_INFO,0);
 show_message("# still active.                                     #","",MT_INFO,0);
 show_message("#                                                   #","",MT_INFO,0);
 show_message("# You should close any pardcap servers before       #","",MT_INFO,0);
 show_message("# exiting this program entirely (which will close   #","",MT_INFO,0);
 show_message("# all connections).                                 #","",MT_INFO,0);
 show_message("#                                                   #","",MT_INFO,0);
 show_message("# Press [Return] to close all connections and exit, #","",MT_INFO,0);
 show_message("# or type 'r' and press [Return] to RESTART server. #","",MT_INFO,0);
 show_message("#                                                   #","",MT_INFO,0);
 show_message("#####################################################","",MT_INFO,0);
 show_message("\nSelection: ","",MT_INFO,1);

 // Use get_keyboard_input for robust, overflow-protected terminal reading
 get_keyboard_input(invalue, sizeof(invalue));

 // Check the first element of the character array
 if(invalue[0] == 'r' || invalue[0] == 'R'){
   show_message("\n>>> Re-arming and restarting PARD server... <<<\n\n", "",
                                                                    MT_INFO, 0);
   
   // Standard OS precaution: A brief 200ms delay helps clear any lingering 
   // socket states in the Linux TCP stack before the new bind request.
   usleep(200000); 
   
   Verbosity = verbackup; // Re-enstate the user's choice for Verbosity

   goto start_the_server;
  }

 end_server_prog:

 tidy_up();

	return main_retval;
}
#endif

//                                                                            //
///////                           END OF:  MAIN                        /////////
//////////                                                            //////////
////////////////////////////////////////////////////////////////////////////////

// _.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._. //


////////////////////////////////////////////////////////////////////////////////
//////////                                                            //////////
///////                       FUNCTION DEFINITIONS                     /////////
//                                                                            //
#ifdef PARDUS
  //-----------------------------------------------------------------------\\.
 //                          OS portability layer                           \\.
//---------------------------------------------------------------------------\\.
// Helper functions which supply, on MS Windows and macOS, services that the
// Linux code obtains directly from the operating system. None of this is
// compiled on Linux.
#ifdef PARDUS
#if IS_WINDOWS_NATIVE

const char *win_errstr(DWORD err)
// Translates a Win32 (or Winsock) error code into text - the Windows
// counterpart of strerror(). The system message (without its trailing white
// space) and the numeric code are written into a static buffer whose address
// is returned. The buffer is overwritten by the next call.
{
 static char errtxt[320];
 char syserr[256];
 DWORD len;

 len = FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS
                      | FORMAT_MESSAGE_MAX_WIDTH_MASK, NULL, err,
                      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                      syserr, (DWORD)sizeof(syserr), NULL);
 if(len >= (DWORD)sizeof(syserr)) len = (DWORD)sizeof(syserr) - 1;

 // Strip the trailing white space (FormatMessage ends messages with a space or
 // with CR LF):
 while(len > 0 && (syserr[len-1] == ' '  || syserr[len-1] == '\t' ||
                   syserr[len-1] == '\r' || syserr[len-1] == '\n')) len--;
 syserr[len] = '\0';

 if(len == 0) snprintf(errtxt, sizeof(errtxt), "Unknown error (Win32 error %lu)",
                                                           (unsigned long)err);
 else snprintf(errtxt, sizeof(errtxt), "%s (Win32 error %lu)", syserr,
                                                           (unsigned long)err);
 return errtxt;
}

void win_openlog(const char *ident)
// Windows counterpart of openlog(ident, LOG_PID | LOG_CONS, LOG_DAEMON):
// obtains a handle to the Event Log source 'ident' in the Application log.
// Every event record carries the process ID (cf. LOG_PID). If the source
// cannot be opened then security notices still reach the terminal (they are
// always printed to stderr by show_message) but not the Event Log.
{
 if(win_EventSource == NULL) win_EventSource = RegisterEventSourceA(NULL, ident);
 return;
}

void win_syslog(WORD evtype, const char *fmt, ...)
// Windows counterpart of syslog(): formats the message like printf() and writes
// it to the Windows Event Log as an event of type evtype. The Linux LOG_WARNING
// priority corresponds to EVENTLOG_WARNING_TYPE.
{
 char logmsg[1024];
 const char *strings[1];
 va_list ap;

 if(win_EventSource == NULL) return;

 va_start(ap, fmt);
 vsnprintf(logmsg, sizeof(logmsg), fmt, ap);
 va_end(ap);

 strings[0] = logmsg;
 ReportEventA(win_EventSource, evtype, 0, WIN_EVENT_ID, NULL, 1, 0, strings,
                                                                          NULL);
 return;
}

void win_closelog(void)
// Windows counterpart of closelog().
{
 if(win_EventSource != NULL){
   DeregisterEventSource(win_EventSource);
   win_EventSource = NULL;
  }
 return;
}

int win_platform_init(void)
// Process-wide initialisation of Windows services which Linux does not need.
// 1. Winsock 2.2 must be started before getaddrinfo() / socket() are used for
//    the PARDcapture server connections. (ZeroMQ starts its own reference-
//    counted Winsock instance internally, so this does not affect it.)
// 2. The console output code page is set to UTF-8 because some of the
//    program's messages contain UTF-8 characters (e.g. the warning sign). The
//    original code page is restored by win_platform_shutdown().
// 3. The Event Log source for security notices is opened (cf. openlog).
// Returns 0 on success, 1 if Winsock could not be started (fatal).
{
 WSADATA wsadata;

 if(!win_WsaStarted){
   if(WSAStartup(MAKEWORD(2,2), &wsadata) != 0) return 1;
   if(LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wVersion) != 2){
     WSACleanup();
     return 1;
    }
   win_WsaStarted = 1;
  }

 if(win_SavedOutputCP == 0){
   win_SavedOutputCP = GetConsoleOutputCP(); // 0 if there is no console
   if(win_SavedOutputCP != 0) SetConsoleOutputCP(CP_UTF8);
  }

 win_openlog(WIN_EVENT_SOURCE);

 return 0;
}

void win_platform_shutdown(void)
// Releases everything acquired by win_platform_init() and by win_sleep_100ns().
{
 if(win_SleepTimer != NULL){
   CloseHandle(win_SleepTimer);
   win_SleepTimer = NULL;
  }
 win_closelog();
 if(win_WsaStarted){
   WSACleanup();
   win_WsaStarted = 0;
  }
 if(win_SavedOutputCP != 0){
   fflush(stdout); fflush(stderr); // Emit pending UTF-8 text first
   SetConsoleOutputCP(win_SavedOutputCP);
   win_SavedOutputCP = 0;
  }
 return;
}

int win_privilege_level(void)
// Windows counterpart of the Linux test 'geteuid() == 0' (running as root).
// The process is privileged if its access token contains the group
// BUILTIN\Administrators in the ENABLED state. This is true for an elevated
// ('Run as administrator') process and for the SYSTEM account, and false for
// an ordinary user and for an administrator's filtered (non-elevated, UAC)
// token, in which that group is present only as 'deny-only'.
// Returns 1 = privileged, 0 = unprivileged, -1 = could not be determined.
{
 SID_IDENTIFIER_AUTHORITY nt_authority = {SECURITY_NT_AUTHORITY};
 PSID admins_sid = NULL;
 BOOL is_member = FALSE;
 int level = -1;

 if(AllocateAndInitializeSid(&nt_authority, 2, SECURITY_BUILTIN_DOMAIN_RID,
                      DOMAIN_ALIAS_RID_ADMINS, 0, 0, 0, 0, 0, 0, &admins_sid)){
   // A NULL token means: the effective token of the calling thread.
   if(CheckTokenMembership(NULL, admins_sid, &is_member))
     level = is_member ? 1 : 0;
   FreeSid(admins_sid);
  }

 return level;
}

#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002 // Windows 10 1803+
#endif

int win_sleep_100ns(LONGLONG ticks)
// Suspends the calling thread for 'ticks' x 100 ns (the Win32 time unit).
// A high-resolution waitable timer (Windows 10 version 1803 and later) gives
// sub-millisecond resolution without raising the system-wide timer frequency
// (as timeBeginPeriod() would). On older systems a standard waitable timer is
// used (resolution: the system clock tick, ~15.6 ms by default) and, should
// timers be unavailable, Sleep() rounded up to whole milliseconds.
// Returns 0.
{
 LARGE_INTEGER due;

 if(ticks <= 0) return 0;

 // Create the (re-usable) timer on first use. It is closed at program end by
 // win_platform_shutdown().
 if(win_SleepTimer == NULL){
   win_SleepTimer = CreateWaitableTimerExW(NULL, NULL,
                        CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
   if(win_SleepTimer == NULL)
     win_SleepTimer = CreateWaitableTimerExW(NULL, NULL, 0, TIMER_ALL_ACCESS);
  }

 due.QuadPart = -ticks; // A negative due time is an interval relative to now
 if(win_SleepTimer != NULL &&
    SetWaitableTimer(win_SleepTimer, &due, 0, NULL, NULL, FALSE)){
   WaitForSingleObject(win_SleepTimer, INFINITE);
  } else {
   Sleep((DWORD)((ticks + 9999) / 10000)); // Round up to whole milliseconds
  }

 return 0;
}

int as_win_write(const void *buf, DWORD nbytes)
// Windows counterpart of write(as_portName, buf, nbytes) for the Arduino serial
// port. Returns the number of bytes written or -1 on error - exactly as write()
// does - so that the byte-count checksums of the serial code work unchanged.
// (The port's write time-out is as_portTimeout ms, see the Windows version of
// as_open_comms_with_Arduino, after which fewer bytes than requested may be
// reported, just as a non-blocking write() on Linux may write fewer.)
{
 DWORD written = 0;

 if(!WriteFile(as_hCom, buf, nbytes, &written, NULL)) return -1;

 return (int)written;
}

void as_win_device_path(const char *port, char *devpath, size_t pathlen)
// Builds the Win32 device path for a COM port name as written in the config
// file, e.g. "COM5" -> "\\.\COM5". The "\\.\" prefix is mandatory for COM10
// and above and harmless below. A name already beginning with "\\.\" is used
// as given and a trailing ':' (as in "COM5:") is removed.
{
 size_t len;

 if(strncmp(port, "\\\\.\\", 4) == 0) snprintf(devpath, pathlen, "%s", port);
 else snprintf(devpath, pathlen, "\\\\.\\%s", port);

 len = strlen(devpath);
 if(len > 0 && devpath[len-1] == ':') devpath[len-1] = '\0';

 return;
}

int as_win_classify_device(const char *kernel_device_name)
// Classifies a serial port by the kernel device name under which its driver
// lists it in the registry key HKLM\HARDWARE\DEVICEMAP\SERIALCOMM. The two
// classes mirror the two paths tried by the Linux 'auto' detection:
//  AS_WIN_CLASS_ACM : \Device\USBSER000, ... (Microsoft usbser.sys = USB
//                     CDC-ACM, i.e. Linux /dev/ttyACM*): genuine Arduino Uno
//                     R3 and Mega 2560 (ATmega16U2) and native-USB boards.
//  AS_WIN_CLASS_USB : \Device\VCP0 (FTDI), \Device\Silabser0 (CP210x),
//                     \Device\CH341SER... / WCH (CH340/CH341) and
//                     \Device\ProlificSerial0 (PL2303): USB-serial bridges as
//                     used on many Arduino clones (Linux /dev/ttyUSB*).
//  AS_WIN_CLASS_NONE: anything else, e.g. \Device\Serial0 (a motherboard UART,
//                     Linux /dev/ttyS*), Bluetooth serial links and modems -
//                     ports which the Linux 'auto' detection never tries.
{
 char uname[256];
 size_t idx;

 // Upper-case copy for a case-blind comparison
 for(idx = 0; idx < sizeof(uname) - 1 && kernel_device_name[idx]; idx++)
    uname[idx] = (char)toupper((unsigned char)kernel_device_name[idx]);
 uname[idx] = '\0';

 if(strstr(uname, "USBSER")) return AS_WIN_CLASS_ACM;
 if(strstr(uname, "\\DEVICE\\VCP") || strstr(uname, "SILABSER") ||
    strstr(uname, "CH341SER")      || strstr(uname, "WCH")      ||
    strstr(uname, "PROLIFIC")) return AS_WIN_CLASS_USB;

 return AS_WIN_CLASS_NONE;
}

static void as_win_upper(char *dst, const char *src, size_t dstlen)
// Upper-case copy (for case-blind comparisons of identifiers), truncating.
{
 size_t idx;
 if(dstlen == 0) return;
 for(idx = 0; idx < dstlen - 1 && src[idx]; idx++)
    dst[idx] = (char)toupper((unsigned char)src[idx]);
 dst[idx] = '\0';
}

int as_win_rank_device(const char *hwid, const char *service,
                       const char *compatids, const char *enumerator)
// Ranks a present serial (COM) port for the 'auto' Arduino search, from the
// strings which SetupAPI reports for its device:
//   hwid       first hardware ID, e.g. "USB\VID_2341&PID_0042&REV_0001",
//              "FTDIBUS\COMPORT&VID_0403&PID_6001", "BTHENUM\{...}", "ACPI\PNP0501"
//   service    driver service name, e.g. "usbser", "CH341SER_A64", "FTSER2K"
//   compatids  compatible IDs, all strings joined by spaces (a USB CDC-ACM
//              device has "USB\Class_02&SubClass_02")
//   enumerator bus enumerator, e.g. "USB", "FTDIBUS", "BTHENUM", "ACPI", "ROOT"
// Any of them may be empty (""). Returns an AS_WIN_RANK_ value. Only USB-
// attached ports are ranked for trial, mirroring the Linux 'auto' search which
// only tries the USB device nodes /dev/ttyACM0 and /dev/ttyUSB0.
{
 char uh[512], us[128], uc[1024], ue[64], *vp;
 unsigned long vid = 0;
 int usb;

 as_win_upper(uh, hwid ? hwid : "", sizeof(uh));
 as_win_upper(us, service ? service : "", sizeof(us));
 as_win_upper(uc, compatids ? compatids : "", sizeof(uc));
 as_win_upper(ue, enumerator ? enumerator : "", sizeof(ue));

 // Never tried: Bluetooth links, modems and com0com-style virtual ports
 if(strncmp(uh, "BTHENUM\\", 8) == 0 || strcmp(ue, "BTHENUM") == 0 ||
    strstr(uh, "COM0COM") || strstr(us, "BTHMODEM")) return AS_WIN_RANK_NONE;

 // USB vendor ID, if any (e.g. "VID_2341")
 vp = strstr(uh, "VID_");
 if(vp) vid = strtoul(vp + 4, NULL, 16);

 usb = (strncmp(uh, "USB\\", 4) == 0 || strncmp(uh, "FTDIBUS\\", 8) == 0 ||
        strcmp(ue, "USB") == 0 || strcmp(ue, "FTDIBUS") == 0 || vp != NULL);
 if(!usb) return AS_WIN_RANK_NONE;              // motherboard, PCI, ACPI ...

 if(vid == 0x2341 || vid == 0x2A03) return AS_WIN_RANK_ARDUINO; // Arduino
 if(strcmp(us, "USBSER") == 0 || strstr(uc, "USB\\CLASS_02&SUBCLASS_02"))
   return AS_WIN_RANK_ACM;                                      // CDC-ACM
 if(vid == 0x1A86 || vid == 0x0403 || vid == 0x10C4 || vid == 0x067B ||
    strncmp(us, "CH341SER", 8) == 0 || strcmp(us, "FTSER2K") == 0 ||
    strcmp(us, "SILABSER") == 0 || strcmp(us, "SER2PL") == 0 ||
    strncmp(us, "SER2PL", 6) == 0 || strcmp(ue, "FTDIBUS") == 0)
   return AS_WIN_RANK_BRIDGE;                                   // bridges
 return AS_WIN_RANK_OTHERUSB;
}

static void as_win_add_candidate(AsWinCandidate *cand, int *ncand, int maxcand,
                          const char *port, int rank, const char *desc)
// Adds a port to the candidate list, or improves the rank of a port that is
// already listed (the same port can be reported by two enumerations). Ports
// of rank AS_WIN_RANK_NONE and names that are not "COM<number>" are ignored.
{
 char *endp;
 unsigned long num;
 int idx;

 if(rank >= AS_WIN_RANK_NONE) return;
 if(_strnicmp(port, "COM", 3) != 0) return;
 num = strtoul(port + 3, &endp, 10);
 if(endp == port + 3 || *endp != '\0') return;
 for(idx = 0; idx < *ncand; idx++){
    if(cand[idx].num == num){
      if(rank < cand[idx].rank){
        cand[idx].rank = rank;
        snprintf(cand[idx].desc, sizeof(cand[idx].desc), "%s", desc);
       }
      return;
     }
   }
 if(*ncand >= maxcand) return;
 snprintf(cand[*ncand].port, sizeof(cand[*ncand].port), "COM%lu", num);
 cand[*ncand].num  = num;
 cand[*ncand].rank = rank;
 snprintf(cand[*ncand].desc, sizeof(cand[*ncand].desc), "%s", desc);
 (*ncand)++;
}

static void as_win_setupapi_scan(const GUID *guid, DWORD flags,
                          AsWinCandidate *cand, int *ncand, int maxcand)
// Lists the present devices of one SetupAPI set (a device class or a device
// interface class), reads each device's COM port name ("PortName" in its
// device registry key) and identifying strings, ranks it and adds it.
{
 HDEVINFO hdi;
 SP_DEVINFO_DATA dev;
 DWORD idx, type, len, k;
 HKEY hkey;
 char port[64], hwid[512], service[128], compat[1024], enumer[64], fname[160];
 char desc[200];
 BYTE buf[1024];

 hdi = SetupDiGetClassDevsA(guid, NULL, NULL, DIGCF_PRESENT | flags);
 if(hdi == INVALID_HANDLE_VALUE) return;

 for(idx = 0; ; idx++){
    memset(&dev, 0, sizeof(dev));
    dev.cbSize = sizeof(dev);
    if(!SetupDiEnumDeviceInfo(hdi, idx, &dev)) break; // ERROR_NO_MORE_ITEMS

    // The COM port name lives in the device's hardware registry key
    hkey = SetupDiOpenDevRegKey(hdi, &dev, DICS_FLAG_GLOBAL, 0, DIREG_DEV,
                                                               KEY_QUERY_VALUE);
    if(hkey == INVALID_HANDLE_VALUE) continue;
    len = (DWORD)(sizeof(port) - 1);
    type = 0;
    if(RegQueryValueExA(hkey, "PortName", NULL, &type, (LPBYTE)port, &len)
                                != ERROR_SUCCESS || type != REG_SZ) len = 0;
    port[len] = '\0';
    RegCloseKey(hkey);
    if(len == 0) continue;                    // not a COM port (e.g. LPT1)

    // First hardware ID (REG_MULTI_SZ: the first string is the most specific)
    hwid[0] = '\0';
    if(SetupDiGetDeviceRegistryPropertyA(hdi, &dev, SPDRP_HARDWAREID, &type,
                                     buf, sizeof(buf) - 2, NULL)){
      buf[sizeof(buf) - 2] = buf[sizeof(buf) - 1] = 0;
      snprintf(hwid, sizeof(hwid), "%s", (char *)buf);
     }
    // All compatible IDs, joined by spaces
    compat[0] = '\0';
    if(SetupDiGetDeviceRegistryPropertyA(hdi, &dev, SPDRP_COMPATIBLEIDS, &type,
                                     buf, sizeof(buf) - 2, NULL)){
      buf[sizeof(buf) - 2] = buf[sizeof(buf) - 1] = 0;
      for(k = 0; k < sizeof(buf) - 2 && buf[k]; k += (DWORD)strlen((char *)buf + k) + 1){
         if(strlen(compat) + strlen((char *)buf + k) + 2 < sizeof(compat)){
           strcat(compat, (char *)buf + k);
           strcat(compat, " ");
          }
        }
     }
    service[0] = enumer[0] = fname[0] = '\0';
    if(SetupDiGetDeviceRegistryPropertyA(hdi, &dev, SPDRP_SERVICE, &type,
                                     buf, sizeof(buf) - 1, NULL)){
      buf[sizeof(buf) - 1] = 0;
      snprintf(service, sizeof(service), "%s", (char *)buf);
     }
    if(SetupDiGetDeviceRegistryPropertyA(hdi, &dev, SPDRP_ENUMERATOR_NAME, &type,
                                     buf, sizeof(buf) - 1, NULL)){
      buf[sizeof(buf) - 1] = 0;
      snprintf(enumer, sizeof(enumer), "%s", (char *)buf);
     }
    if(SetupDiGetDeviceRegistryPropertyA(hdi, &dev, SPDRP_FRIENDLYNAME, &type,
                                     buf, sizeof(buf) - 1, NULL)){
      buf[sizeof(buf) - 1] = 0;
      snprintf(fname, sizeof(fname), "%s", (char *)buf);
     }

    snprintf(desc, sizeof(desc), "%.90s [%.80s]", fname[0] ? fname : "(no name)",
                                                    hwid[0] ? hwid : "no hardware ID");
    as_win_add_candidate(cand, ncand, maxcand, port,
                         as_win_rank_device(hwid, service, compat, enumer), desc);
   }

 SetupDiDestroyDeviceInfoList(hdi);
}

static void as_win_serialcomm_scan(AsWinCandidate *cand, int *ncand, int maxcand)
// Fall-back used only when SetupAPI lists no port at all: the registry list of
// present serial ports, HKLM\HARDWARE\DEVICEMAP\SERIALCOMM (value name = kernel
// device name, value data = "COMn"), classified by the driver's device name.
{
 HKEY hkey;
 DWORD idx, namelen, datalen, type;
 LONG rc;
 char name[256], data[64], desc[200];
 int cls;

 if(RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0,
                                           KEY_READ, &hkey) != ERROR_SUCCESS)
   return;

 for(idx = 0; ; idx++){
    namelen = (DWORD)sizeof(name);        // (both sizes are in/out parameters
    datalen = (DWORD)(sizeof(data) - 1);  //  and must be reset for every call)
    rc = RegEnumValueA(hkey, idx, name, &namelen, NULL, &type, (LPBYTE)data,
                                                                     &datalen);
    if(rc == ERROR_NO_MORE_ITEMS) break;
    if(rc != ERROR_SUCCESS || type != REG_SZ) continue; // Skip odd entries
    data[datalen] = '\0'; // REG_SZ data need not be terminated in the registry
    cls = as_win_classify_device(name);
    if(cls == AS_WIN_CLASS_NONE) continue;
    snprintf(desc, sizeof(desc), "registry: %.150s", name);
    as_win_add_candidate(cand, ncand, maxcand, data,
           (cls == AS_WIN_CLASS_ACM) ? AS_WIN_RANK_ACM : AS_WIN_RANK_BRIDGE, desc);
   }

 RegCloseKey(hkey);
}

static int as_win_candidate_cmp(const void *a, const void *b)
// qsort order of the candidates: by rank, then by COM port number.
{
 const AsWinCandidate *ca = (const AsWinCandidate *)a;
 const AsWinCandidate *cb = (const AsWinCandidate *)b;
 if(ca->rank != cb->rank) return (ca->rank < cb->rank) ? -1 : 1;
 if(ca->num  != cb->num)  return (ca->num  < cb->num)  ? -1 : 1;
 return 0;
}

int as_win_list_candidates(AsWinCandidate *cand, int maxcand)
// Builds the ordered list of COM ports which the 'auto' search tries, and
// returns their number. The ports present now are listed by SetupAPI from the
// device interface class of COM ports (GUID_DEVINTERFACE_COMPORT) and from the
// device setup class "Ports" (GUID_DEVCLASS_PORTS: some USB-serial drivers do
// not register the interface), ranked by as_win_rank_device() and sorted by
// rank and COM number. If SetupAPI lists nothing (it cannot be used, or the
// system offers no device information, as under Wine), the registry SERIALCOMM
// list is used instead. (The GUID values are those of ntddser.h and devguid.h,
// written out here so that no extra header or initguid.h is required.)
{
 static const GUID guid_devinterface_comport =
   {0x86E0D1E0, 0x8089, 0x11D0, {0x9C, 0xE4, 0x08, 0x00, 0x3E, 0x30, 0x1F, 0x73}};
 static const GUID guid_devclass_ports =
   {0x4D36E978, 0xE325, 0x11CE, {0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18}};
 int ncand = 0;

 as_win_setupapi_scan(&guid_devinterface_comport, DIGCF_DEVICEINTERFACE,
                                                       cand, &ncand, maxcand);
 as_win_setupapi_scan(&guid_devclass_ports, 0, cand, &ncand, maxcand);
 if(ncand == 0) as_win_serialcomm_scan(cand, &ncand, maxcand);
 if(ncand > 1) qsort(cand, (size_t)ncand, sizeof(AsWinCandidate),
                                                        as_win_candidate_cmp);
 return ncand;
}

#elif defined(__APPLE__)

int as_mac_list_candidates(AsMacCandidate *cand, int maxcand)
// Builds the ordered list of device nodes which the macOS 'auto' Arduino
// search tries, and returns their number: every match of
//   /dev/cu.usbmodem*        rank 0: USB CDC-ACM (genuine Arduino; cf. /dev/ttyACM0)
//   /dev/cu.usbserial*       rank 1: USB-serial bridges with Apple's drivers
//   /dev/cu.wchusbserial*    rank 1: CH34x with the WCH driver
//   /dev/cu.SLAB_USBtoUART*  rank 1: CP210x with the Silicon Labs driver
// in this order (glob() sorts the matches of each pattern alphabetically).
// The 'call-out' (cu.*) nodes are used, never the 'dial-in' tty.* nodes.
{
 static const char *const patterns[] = {"/dev/cu.usbmodem*", "/dev/cu.usbserial*",
                                       "/dev/cu.wchusbserial*",
                                       "/dev/cu.SLAB_USBtoUART*"};
 static const int ranks[] = {0, 1, 1, 1};
 size_t ip, im;
 glob_t gl;
 int ncand = 0;

 for(ip = 0; ip < sizeof(patterns)/sizeof(patterns[0]); ip++){
    memset(&gl, 0, sizeof(gl));
    if(glob(patterns[ip], 0, NULL, &gl) == 0){
      for(im = 0; im < gl.gl_pathc && ncand < maxcand; im++){
         snprintf(cand[ncand].path, sizeof(cand[ncand].path), "%s", gl.gl_pathv[im]);
         cand[ncand].rank = ranks[ip];
         ncand++;
        }
     }
    globfree(&gl);
   }

 return ncand;
}

#endif
//---------------------------------------------------------------------------//.
 //                          OS portability layer                           //.
  //-----------------------------------------------------------------------//.
#endif

  //-----------------------------------------------------------------------\\.
 //                            General utilities                            \\.
//---------------------------------------------------------------------------\\.
#ifdef PARDUS

uint8_t generate_version_string(char *vstring, size_t max_len)
// Compiles the version and padded compile-date into vstring.
// Returns 0 on success, 1 on failure. Does NOT include a newline.
{
 // Capture the static ANSI C compile date macro string ("Mmm DD YYYY")
 const char *compile_date = __DATE__; 
 
 char month_str[4] = {0};
 int day = 0;
 int year = 0;
 int month_num = 1;
 char temp_buf[128] = {0};

 // 1. Extract the string components safely using sscanf
 if (sscanf(compile_date, "%3s %d %d", month_str, &day, &year) != 3) {
     return 1; // Macro formatting extraction mismatch
 }

 // 2. Map the 3-letter month string to its numeric tracking index
 const char *months[] = {
     "Jan", "Feb", "Mar", "Apr", "May", "Jun", 
     "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
 };
 
 uint8_t month_matched = 0;
 for (int i = 0; i < 12; i++) {
     if (strcmp(month_str, months[i]) == 0) {
         month_num = i + 1;
         month_matched = 1;
         break;
     }
 }
 
 if (!month_matched) {
     return 1; // Month token was unrecognised
 }

 // 3. Extract the last two digits of the year (modulo 100)
 int short_year = year % 100;

 // 4. Safely format the output string into our temporary staging buffer
 // %02d ensures single digit days or months carry a leading zero padding!
 snprintf(temp_buf, sizeof(temp_buf), "%s.%02d.%02d.%02d", 
          PARDSERVER_VERSION, day, month_num, short_year);

 // 5. Memory Fence Guard: Protect the calling function from an array overflow
 if (strlen(temp_buf) >= max_len) {
     return 1; // Targeted buffer space is too small
 }

 // 6. Commit the formatted dataset to your destination array pointer
 strcpy(vstring, temp_buf);
 return 0; // Success
}

#if IS_WINDOWS_NATIVE
void tidy_up(void)
// Release resources prior to ending the program.
// MS Windows version: the serial port is a Win32 HANDLE (closed with
// CloseHandle), and the Linux closelog() is replaced by win_platform_shutdown()
// which closes the Event Log source and also releases Winsock, the sleep timer
// and restores the console code page.
{
 unsigned int idx;

 show_message("","",MT_INFO,0);
 show_message("Tidying up ...","",MT_INFO,0);


	// Close communication port with Arduino
 if(as_connected){
   show_message(": > Closing serial port with Arduino ...",fnn,MT_INFO,0);
   if(as_hCom != INVALID_HANDLE_VALUE){
     CloseHandle(as_hCom);
     as_hCom = INVALID_HANDLE_VALUE;
    }
   show_message(": > Done.",fnn,MT_INFO,0);
  }
 // Hang up on PARD Capture server(s) (if connected)

 for(idx=0;idx<NPSs;idx++){
    show_message(": > ",fnn,MT_INFO,1);
    ps_hangup(idx);
   }

 if(sv_Triad!=NULL){
  show_message(": > Freeing message sv_Triad.",fnn,MT_INFO,0);
  free(sv_Triad);
 }
 if(sv_Triadarg!=NULL){
  show_message(": > Freeing message sv_Triad arguments.",fnn,MT_INFO,0);
  free(sv_Triadarg);
 }
 if(sv_Blob!=NULL){
  show_message(": > Freeing sv_Blob.",fnn,MT_INFO,0);
  free(sv_Blob);
 }
 if(sv_UserDb != NULL) {
  show_message(": > Freeing sv_UserDb.",fnn,MT_INFO,0);
  free(sv_UserDb);
  sv_UserDb = NULL;
 }
 show_message("\nThe PARDUS server says: Bye!\n","",MT_INFO,0);

 // Terminate logging activities and Windows platform services
 close_session_logfile();
 win_platform_shutdown();

 return;
}
#else
void tidy_up(void) 
// Release resources prior to ending the program.
{
 unsigned int idx;

 show_message("","",MT_INFO,0);
 show_message("Tidying up ...","",MT_INFO,0);

	
	// Close communication port with Arduino
 if(as_connected){
   show_message(": > Closing serial port with Arduino ...",fnn,MT_INFO,0);
	  close(as_portName);
   show_message(": > Done.",fnn,MT_INFO,0);
  }
 // Hang up on PARD Capture server(s) (if connected)
 
 for(idx=0;idx<NPSs;idx++){
    show_message(": > ",fnn,MT_INFO,1);
    ps_hangup(idx);
   }

 if(sv_Triad!=NULL){
  show_message(": > Freeing message sv_Triad.",fnn,MT_INFO,0);
  free(sv_Triad);
 }
 if(sv_Triadarg!=NULL){
  show_message(": > Freeing message sv_Triad arguments.",fnn,MT_INFO,0);
  free(sv_Triadarg);
 }
 if(sv_Blob!=NULL){
  show_message(": > Freeing sv_Blob.",fnn,MT_INFO,0);
  free(sv_Blob);
 }
 if(sv_UserDb != NULL) {
  show_message(": > Freeing sv_UserDb.",fnn,MT_INFO,0);
  free(sv_UserDb);
  sv_UserDb = NULL;
 }
 show_message("\nThe PARDUS server says: Bye!\n","",MT_INFO,0);

 // Terminate logging activities
 close_session_logfile();
 closelog(); 

 return;    
}
#endif

void get_keyboard_input(char *receptstr, size_t receptlen)
// Get input from the keyboard and sanitise.
// If the user aborts with Ctrl+D this will terminate the program with exit(1)
// Otherwise the input string, cleared of any newline or carriage return,
// will be put into receptstr.
{
  char *recp;
  int c;

  // Safety edge case: Guard against invalid or empty buffer requests
  if (receptstr == NULL || receptlen == 0) return;

  fflush(stdout);
  memset((void *)receptstr, 0, receptlen);
  recp = fgets(receptstr, receptlen, stdin);
  
  if(recp == NULL) {
    fprintf(stderr, "\nCRITICAL ERROR: Terminal input read aborted (EOF).\n");
    Verbosity = 0; tidy_up();
    exit(1);
   }

 // -------------------------------------------------------------------------
 // THE STREAM DRAIN HOOK (AUTOMATIC OVERFLOW PROTECTION)
 // -------------------------------------------------------------------------
 // If the captured string does NOT end with a newline, and the string length 
 // reaches (receptlen - 1), it mathematically proves the user's input was 
 // truncated and unread characters are still sitting inside stdin!
 size_t actual_len = strlen(receptstr);
 
 if (actual_len == (receptlen - 1) &&
     receptstr[actual_len - 1] != '\n' &&
     receptstr[actual_len - 1] != '\r') {
     // Drain and discard the remaining keyboard stream characters up to the
     // true newline or EOF
     while ((c = getchar()) != '\n' && c != EOF) {
         // Do absolutely nothing, just clearing the buffer lanes.
     }
 }

 // Cleanly strip out any trailing newlines safely using strcspn
 receptstr[strcspn(receptstr, "\r\n")] = '\0';
 return;
}

int num_words(char *test, const char *sep) 
// Count the number of 'words' in test separated by any character in sep 
// Returns -1 on error.
{
 char *currstrpos,*cpy;  
 int tparts;
 size_t len;
 
 len=strlen(test);
 if(len==0) return 0; else len++;
 
 cpy=calloc(len,sizeof(char));
 if(cpy==NULL){
  show_message("Failed to allocate RAM for word counter.","ERROR: ",MT_ERR,0);
  return -1;
 }
 
 memcpy(cpy,test,len);
 
 tparts=0;
 
 currstrpos=strtok(cpy,sep);
 while(currstrpos!=NULL){
        len=strlen(currstrpos);
        if(len)tparts++;
        currstrpos=strtok(NULL,sep);
      }
      
 free(cpy);
 return tparts;
}

char *graberr_str(int32_t err)
// Return an interpretative string of the image grabber error code
{
 switch(err){
     case GRAB_ERR_NONE:
         return "Success";
         
     case GRAB_ERR_SELECT:
         return "Error selecting a frame from the device stream";
         
     case GRAB_ERR_TIMEOUT:
         return "Timeout reached while waiting for a frame from the stream";
         
     case GRAB_ERR_READIO:
         return "Error reading frame data via standard Read I/O method";
         
     case GRAB_ERR_MMAPD:
         return "Memory-mapped buffer dequeue error (DQBUF)";
         
     case GRAB_ERR_MMAPQ:
         return "Memory-mapped buffer enqueue error (QBUF)";
         
     case GRAB_ERR_USERPD:
         return "User-pointer buffer dequeue error (DQBUF)";
         
     case GRAB_ERR_USERPQ:
         return "User-pointer buffer enqueue error (QBUF)";
         
     case GRAB_ERR_NOSTREAM:
         return "The camera interface is not actively streaming images";
         
     case GRAB_ERR_BUSY:
         return "The image grabber is busy or in use by another function";
         
     case GNS_ETCV:
         return "Error in the tissue content value getting function.";

     case GNS_ECAM:
         return "Failed due to problem running the camera.";
         
     default:
         break;
 }
 return "Unknown error - program bug";
}


char *Pard_err_str(void)
// Translates the meaning of the current value of the global error flag
// Pard_Err into a test message.
//
// Arguments: (none)
// Returns:   char *    : Pointer to character string
// Sets pard_err: (no)
//
{
 char errsource[32];

 if(Pard_Err==PARD_OK){
   snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] All went well.");
   goto end_Pard_err_str;
 }

 snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] Error string unset (program bug).");

 if(abs(Pard_Err)<=63){
   if(Pard_Err<0) snprintf(errsource, sizeof(errsource), "[PARDUS]");
   else  snprintf(errsource, sizeof(errsource), "[PARDUINO]");
   Pard_Err=abs(Pard_Err);
   switch(Pard_Err){
     // First the non-fatal / warning / info messages
     case PARDU_INFO_LIMTRIP:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Motion cut short due to a limit trip.",errsource);
     break;
     case PARDU_INFO_HALTED:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Motion stopped by a Halt signal.",errsource);
     break;
     case PARDU_INFO_STPCNT:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Motion stopped by a step count limit.",errsource);
     break;
     //  Now the errors proper:
     case PARDU_ERR_CHKSUM:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Serial comms checksum error.",errsource);
     break;
     case PARDU_ERR_MSGSEQ:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Message out of seuquence.",errsource);
     break;
     case PARDU_ERR_IMPLEM:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Requested item not (yet) implemented.",errsource);
     break;
     case PARDU_ERR_UNKORDER:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Unknown order.",errsource);
     break;
     case PARDU_ERR_ACTINVAL:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Invalid / non-existent actuator.",errsource);
     break;
     case PARDU_ERR_IDINVAL:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Invalid actuator ID number.",errsource);
     break;
     case PARDU_ERR_IDTAKEN:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Actuator ID number already allocated.",errsource);
     break;
     case PARDU_ERR_NOTERM:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Invalid message (no termination sequence).",errsource);
     break;
     case PARDU_ERR_MSGVER:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Invalid message format version.",errsource);
     break;
     case PARDU_ERR_NMFAIL:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Insufficient RAM to add a new actuator node.",errsource);
     break;
     case PARDU_ERR_NFULL:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Actuator node list full (can't create more).",errsource);
     break;
     case PARDU_ERR_INFINVAL:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Invalid/unknown info item.",errsource);
     break;
     case PARDU_ERR_RDONLY:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Attempt to alter a read only value.",errsource);
     break;
     case PARDU_ERR_PINUNSET:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Attempt to alter the state of an unset pin.",errsource);
     break;
     case PARDU_ERR_PINUSED:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Attempt to use a pin that is already in use.",errsource);
     break;
     case PARDU_ERR_PINRNUM:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Pin number out of range.",errsource);
     break;
     case PARDU_ERR_MUSTDNAV:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Requested microstep denom. not available.",errsource);
     break;
     case PARDU_ERR_BUSY:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Interrupting with non-interruptivum (Busy).",errsource);
     break;
     case PARDU_ERR_DISABLED:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Attempted to activate a disabled actuator.",errsource);
     break;
     case PARDU_ERR_UNHOMED:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Attempted to use an unhomed actuator.",errsource);
     break;
     case PARDU_ERR_HOMERR_0:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Insufficient nhs0 steps to fast-disengage the limit switch.",errsource);
     break;
     case PARDU_ERR_HOMERR_1:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Limit switch triggered after bounce-back nhs1 steps.",errsource);
     break;
     case PARDU_ERR_HOMERR_2:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Insufficient nhs2 steps to fast-engage the limit switch.",errsource);
     break;
     case PARDU_ERR_HOMERR_3:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Failed to fine-engage after nhs0 steps.",errsource);
     break;
     case PARDU_ERR_HOMERR_4:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Failed to fine-disengage after nhs1 steps.",errsource);
     break;
     case PARDU_ERR_NODRIVER:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Motor driver not detected.",errsource);
     break;
     case PARDU_ERR_KILLSWON:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Kill switch is in the active (Kill) state.",errsource);
     break;
     case PARDU_ERR_STALLED:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Stall detected via StallGuard.",errsource);
     break;
     case PARDU_ERR_VCC:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Vcc not detected on Stepper driver board.",errsource);
     break;
     case PARDU_ERR_UART:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s UART comms error after detecting driver.",errsource);
     break;
     default:
       snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"%s Undefined error (program bug).",errsource);
     break;
    }
   goto end_Pard_err_str;
  }

 switch(Pard_Err){
   case PARD_ERR_MSDENOM:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] An invalid microstep denominator value was supplied.");
   break;
   case PARD_ERR_AXIS:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] An invalid motor structure was supplied.");
   break;
   case PARD_ERR_LIMA:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] A limit switch was activated before motion was complete.");
   break;
   case PARD_ERR_LIMU:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] Attempted to use an undefined or nonexistent limit switch type.");
   break;
   case PARD_ERR_LIMSWU:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] Attempted to use an nonexistent limit switch or null homing paramters are not set up.");
   break;
   case PARD_ERR_DIRN:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] A valid direction value was not supplied or could not be set in the motor structure.");
   break;
   case PARD_ERR_SUFBKL:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] Insufficient backlash correction steps could be performed.");
   break;
   case PARD_ERR_NOHOME:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] The motor is not homed (previous direction is undefined).");
   break;
   case PARD_ERR_NEG:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] A negative value was supplied - this is not permitted here.");
   break;
   case PARD_ERR_CALIB:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] Invalid calibration factor (e.g. < 1).");
   break;
   case PARD_ERR_ARGC:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] Wrong number of arguments (possible version incompatibility).");
   break;
   case PARD_ERR_COORD:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] Illegal goto co-ordinate requested for this axis.");
   break;
   case PARD_ERR_IMDEV:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] Could not connect to the imaging device.");
   break;
   case PARD_ERR_MEM:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] A memory allocation task failed.");
   break;
   case PARD_ERR_ACNOM:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] The proposed actuator name is invalid.");
   break;
   case PARD_ERR_ACMEMNOM:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] Failed to get memory to name the new actuator.");
   break;
   case PARD_ERR_ACMEMLIM:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] Failed to get memory for the new actuator limit switch name(s).");
   break;
   case PARD_ERR_ACMEMCAL:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] Failed to get memory for the new actuator unit name.");
   break;
   case PARD_ERR_BUG:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] Failed due to a bug in the source code.");
   break;
   case PARD_ERR_IMMATURE:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] Failed due to the actuator being not sufficiently set up.");
   break;
   case PS_CB_E_BSZ:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Blob too small for correct format requirements.");
   break;
   case PS_CB_E_VER:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Version error.");
   break;
   case PS_CB_E_IID:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Core instruction ID error (invalid CIID).");
   break;
   case PS_CB_E_SID:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Sub-instruction ID error (invalid siid).");
   break;
   case PS_CB_E_SIF:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Invalid image format.");
   break;
   case PS_CB_E_BIN:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Failure to provide a 1 or 0 binary choice value.");
   break;
   case PS_CB_E_MES:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Failure to get memory for the instruction string.");
   break;
   case PS_CB_E_MEB:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Failure to get memory for the instruction blob.");
   break;
   case PS_CB_E_MEE:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Failure to get memory for the encapsulated blob.");
   break;
   case PS_CB_E_PFC:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Preview failure - colour convert failed.");
   break;
   case PS_CB_E_PFZ:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Preview failure - zoom option failed.");
   break;
   case PS_CB_E_DIM:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Wrong image dimensions supplied for intended use.");
   break;
   case PS_CB_E_NOM:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Wrong image name string format for intended use.");
   break;
   case PS_CB_E_SAV:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Failed to save image to local disc.");
   break;
   case PS_CB_E_LDL:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Failed to load image from local disc.");
   break;
   case PS_CB_E_FSR:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Failed to apply a setting due to out of range.");
   break;
   case PS_CB_E_CSE:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Checksum error.");
   break;
   case PS_CB_E_NTC:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] No tissue content.");
   break;
   case PS_CB_E_NBC:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] No background correction.");
   break;
   case PS_CB_E_GRB:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Image grab error (%s).",
                                            graberr_str(ps_Instruction.err[1]));
   break;
   case PS_CB_E_CAM:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Couldn't run or access the camera.");
   break;
   case PS_CB_E_CST:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Couldn't set the control to the value provided.");
   break;
   case PS_CB_E_CGT:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Couldn't get the camera control value.");
   break;
   case PS_CB_E_FSC:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Failed to apply setting due to lack of pre-requisite criteria.");
   break;
   case PS_CB_E_CEM:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Failed to load field correction image for lack of a mask.");
   break;
   case PS_CB_E_CEI:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Failed to load field correction due to failed initialisation.");
   break;
   case PS_CB_E_CMM:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Failed set 'use corrections mask' due to lack of loadable / usable mask.");
   break;
   case PS_CB_E_CID1:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] FAILED to close down the imaging device. Cannot change resolution.");
   break;
   case PS_CB_E_CID2:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] FAILED to change resolution and there may be memory errors - please re-start the server.");
   break;
   case PS_CB_E_CID3:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] FAILED to change resolution (reverted to previous).");
   break;
   case PS_CB_E_CID4:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] FAILED to re-open the imaging device after resize.");
   break;
   case PS_CB_E_LNA:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Selected preview LUT is not available.");
   break;
   case PS_CB_E_SZNA:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Selected preview size is not available.");
   break;
   case PS_CB_E_SZB:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Preview re-size buffer failed.");
   break;
   case PS_CB_E_SZU:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Failed to update preview settings post re-size.");
   break;
   case PS_CB_E_NOS:
     snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDCAP] Error NOS.");
   break;
   default:
   snprintf(Pard_Err_Str, sizeof(Pard_Err_Str),"[PARDUS] Undefined error (program bug).");
   break;
 }

end_Pard_err_str:
 if(Pard_Err_Checksum){
    strcat(Pard_Err_Str,"\nNote: Server comms checksum error has occurred.");
    Pard_Err_Checksum=0;
   }
 return Pard_Err_Str;
}

int str_to_uint32(char *buf,uint32_t *ival)
// Read an unsigned int from the input string buffer buf into ival.
// Return 0 on success, non-zero on failure as per the following:
// 1 = out of allowed range
// 2 = no character was read (possibly a format issue in the input buf)
// 3 = whole input not converted
{
  char *endptr;
  unsigned long int ulin;
  int retval=0;

  errno = 0;
  ulin = strtol(buf, &endptr, 10);
  if (errno == ERANGE) retval = 1;
  else if (endptr == buf) retval = 2;
  else if (*endptr && *endptr != '\n') retval = 3;
  else {
   // We successfully read an integer of type unsigned long int
   // but we want to read a uint32_t so we need to check that the result does not
   // overflow the maximum of that type:
   if(ulin > UINT32_MAX)  retval = 1;
     else *ival = (uint32_t)ulin;
  }

 return retval;
}

int str_to_uchar(char *buf,unsigned char *ival)
// Read an unsigned char from the input string buffer buf into ival.
// Return 0 on success, non-zero on failure as per the following:
// 1 = out of allowed range
// 2 = no character was read (possibly a format issue in the input buf)
// 3 = whole input not converted
{
  char *endptr;
  unsigned long int ulin;
  int retval=0;

  errno = 0;
  ulin = strtol(buf, &endptr, 10);
  if (errno == ERANGE) retval = 1;
  else if (endptr == buf) retval = 2;
  else if (*endptr && *endptr != '\n') retval = 3;
  else {
   // We successfully read an integer of type unsigned long int
   // but we want to read a uint32_t so we need to check that the result does not
   // overflow the maximum of that type:
   if(ulin > 255)  retval = 1;
     else *ival = (unsigned char)ulin;
  }

 return retval;
}

#if IS_WINDOWS_NATIVE
// msleep(): Sleep for the requested number of milliseconds.
// MS Windows version: a high-resolution waitable timer (see win_sleep_100ns)
// replaces nanosleep(). Returns 0, or -1 with errno = EINVAL for a negative
// request, exactly as the Linux version does.
int msleep(long msec)
{
    if (msec < 0)
    {
        errno = EINVAL;
        return -1;
    }

    // 1 millisecond = 10,000 units of 100 nanoseconds
    return win_sleep_100ns((LONGLONG)msec * 10000LL);
}

// musleep(): Sleep for the requested number of microseconds.
// MS Windows version (see msleep above). Note: Windows cannot guarantee a
// wake-up within a few microseconds; with the high-resolution timer the
// typical minimum sleep is of the order of 0.5 to 1 ms. That is still a true
// yield to the OS, which is what the callers (polling loops) require.
int musleep(long musec)
{
    if (musec < 0)
    {
        errno = EINVAL;
        return -1;
    }

    // 1 microsecond = 10 units of 100 nanoseconds
    return win_sleep_100ns((LONGLONG)musec * 10LL);
}
#else
// msleep(): Sleep for the requested number of milliseconds.
// from https://stackoverflow.com/questions/1157209/is-there-an-alternative-
//                                           sleep-function-in-c-to-milliseconds
int msleep(long msec)
{
    struct timespec ts;
    int res;

    if (msec < 0)
    {
        errno = EINVAL;
        return -1;
    }

    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;

    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

    return res;
}

// musleep(): Sleep for the requested number of microseconds.
int musleep(long musec)
{
    struct timespec ts;
    int res;

    if (musec < 0)
    {
        errno = EINVAL;
        return -1;
    }

    ts.tv_sec = musec / 1000000;
    // 1 microsecond = 1,000 nanoseconds
    ts.tv_nsec = (musec % 1000000) * 1000; 

    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

    return res;
}
#endif

int bytechecksum(int benacted, int brequested, char *direction)
// Compares the number (of bytes) actually enacted (i.e. actually sent or
// received with the number you requested to be sent/received.
// direction is a string that will typically be "sent" or "received".
// Returns 0 if the two numbers are the same and 1 otherwise.
{
 if(benacted!=brequested){
    printf("Invalid checksum (%d bytes %s, %d requested).\n",
                                                benacted,direction,brequested);
    Pard_Err_Checksum=1;
    return 1;
  }
 return 0;
}

//---------------------------------------------------------------------------//.
 //                            General utilities                            //.
  //-----------------------------------------------------------------------//.
#endif

  //-----------------------------------------------------------------------\\.
 //                           Actuator Management                           \\.
//---------------------------------------------------------------------------\\.
#ifdef PARDUS

int8_t Act_Stepper_create(uint8_t id,const char *idstr)
// Adds a new stepper actuator node to the list of steppers with its members set
// to nascent defaut values.
// Returns one of the PARD_ERR or negative PARDU_ERR_ constants (which see)
// A negative PARDU_ERR constant has the same meaning as the positive version
// but the negativity indicates that the error originated in the server not the
// Arduino so Pard_err_str() will report it as "[PARDUS] ..." instead of
// "[PARDUINO] ..."
{
 Act_StepperNode *newmem;

 // Ensure the proposed name is at least 1 char long
 if(idstr == NULL || strlen(idstr)<1) return PARD_ERR_ACNOM;
 
 // Is the requested id invalid by being too high?
 if(id>=STEPPERSMAX) return -PARDU_ERR_IDINVAL;

 // Is the requested id already taken?
 if(StepperID[id]!=STEPPER_ID_FREE) return -PARDU_ERR_IDTAKEN;

 // Is the list full?
 if(NSteppers>=STEPPERSMAX) return -PARDU_ERR_NFULL;

 // If we get here then the requested ID number is available and within the
 // valid range of ID numbers and there is room in the list to allocate a new
 // actuator definition node so try to create the new node.

 // See if we can get memory for the new node
 newmem=malloc(sizeof(Act_StepperNode));
 if(newmem==NULL) return -PARDU_ERR_NMFAIL;

 // Memory got OK so update the list pointer, number of elements counter and id.
 Stepperlist[id]=newmem;
 NSteppers++;
 StepperID[id]=id;

 // New node created so set the default values
 Stepperlist[id]->driver_type=SD_UNDEF;
 Stepperlist[id]->homed_to=LIM_UNSET;
 Stepperlist[id]->dirn1=DIRN_UNS;
 Stepperlist[id]->dirn2=DIRN_UNS;
 Stepperlist[id]->currdirn=DIRN_UNS;
 Stepperlist[id]->msdenom=1;
 Stepperlist[id]->fastmsd=1;
 Stepperlist[id]->pulwidth=1;
 Stepperlist[id]->blsteps=0;
 Stepperlist[id]->maxblsteps=0;
 Stepperlist[id]->maxmsdenom=1;
 Stepperlist[id]->tensteps1=0;
 Stepperlist[id]->tensteps2=0;
 Stepperlist[id]->onestep=1;
 Stepperlist[id]->maxcalibri=1.0;
 Stepperlist[id]->stepcount=0;
 Stepperlist[id]->sw_lim1=SW_LIM_OFF;
 Stepperlist[id]->sw_lim2=SW_LIM_OFF;
 Stepperlist[id]->nhs0=0;
 Stepperlist[id]->nhs1=0;
 Stepperlist[id]->nhs2=0;
 Stepperlist[id]->isd_hobl=500;
 Stepperlist[id]->limstate=LIM_UNSET;
 Stepperlist[id]->pin_pul= PIN_IS_UNSET;
 Stepperlist[id]->pin_dir= PIN_IS_UNSET;
 Stepperlist[id]->pin_lim1=PIN_IS_UNSET;
 Stepperlist[id]->pin_lim2=PIN_IS_UNSET;
 Stepperlist[id]->pin_diag=PIN_IS_UNSET;
 Stepperlist[id]->pin_ena= PIN_IS_UNSET;
 Stepperlist[id]->pin_ms1= PIN_IS_UNSET;
 Stepperlist[id]->pin_ms2= PIN_IS_UNSET;
 Stepperlist[id]->pin_ms3= PIN_IS_UNSET;
 Stepperlist[id]->maturity=0;

 // Set the actuator name to the supplied idstr
 snprintf(Stepperlist[id]->Name,32,"%s",idstr);

 // Set the initial limit switch names
 if((Stepperlist[id]->lsname[0] = malloc(32)) == NULL){
   Act_Stepper_freecommon(id);
   return PARD_ERR_ACMEMLIM;
 }
 sprintf(Stepperlist[id]->lsname[0],"Unhomed");
 
 if((Stepperlist[id]->lsname[1] = malloc(32)) == NULL){
   free(Stepperlist[id]->lsname[0]);
   Act_Stepper_freecommon(id);
   return PARD_ERR_ACMEMLIM;
 }
 sprintf(Stepperlist[id]->lsname[1],"Limit 1");
 
 if((Stepperlist[id]->lsname[2] = malloc(32)) == NULL){
   free(Stepperlist[id]->lsname[0]);
   free(Stepperlist[id]->lsname[1]);
   Act_Stepper_freecommon(id);
   return PARD_ERR_ACMEMLIM;
 }
 sprintf(Stepperlist[id]->lsname[2],"Limit 2");
 
 if((Stepperlist[id]->lsname[3] = malloc(32)) == NULL){
   free(Stepperlist[id]->lsname[0]);
   free(Stepperlist[id]->lsname[1]);
   free(Stepperlist[id]->lsname[2]);
   Act_Stepper_freecommon(id);
   return PARD_ERR_ACMEMLIM;
 }
 sprintf(Stepperlist[id]->lsname[3],"Soft  1");
 
 if((Stepperlist[id]->lsname[4] = malloc(32)) == NULL){
   free(Stepperlist[id]->lsname[0]);
   free(Stepperlist[id]->lsname[1]);
   free(Stepperlist[id]->lsname[2]);
   free(Stepperlist[id]->lsname[3]);
   Act_Stepper_freecommon(id);
   return PARD_ERR_ACMEMLIM;
 }
 sprintf(Stepperlist[id]->lsname[4],"Soft  2");

 
 // Set the initial calibration unit name
 if((Stepperlist[id]->unitstr = malloc(32)) == NULL){
   free(Stepperlist[id]->lsname[0]);
   free(Stepperlist[id]->lsname[1]);
   free(Stepperlist[id]->lsname[2]);
   free(Stepperlist[id]->lsname[3]);
   free(Stepperlist[id]->lsname[4]);
   Act_Stepper_freecommon(id);
   return PARD_ERR_ACMEMCAL;
 }
 sprintf(Stepperlist[id]->unitstr,"steps");
 
 // All went well if we got to here so update the global 'StageStatus' flag each
 // time a stage motor is defined
 if(id <= ACT_ID_STAGE_Z){
     if((StepperID[ACT_ID_STAGE_X] != STEPPER_ID_FREE) &&
        (StepperID[ACT_ID_STAGE_Y] != STEPPER_ID_FREE)) {
         if (StepperID[ACT_ID_STAGE_Z] != STEPPER_ID_FREE) {
             StageStatus = ACT_ID_STAGE_XYZ;
         } else {
             StageStatus = ACT_ID_STAGE_XY;
         }
     } else {
         StageStatus = 0;
     }
 }


 return PARD_OK;
}

int8_t Act_Stepper_destroy(uint8_t id)
// Frees the Stepper actuator with the supplied ID number.
// If that ID is not allocated it just returns PARDU_ERR_ACTINVAL.
{
 // There is nothing to do if there are no actuators allocated
 if(NSteppers==0) return -PARDU_ERR_ACTINVAL;
 // There is nothing to do if the requested ID is not valid
 if(id>=STEPPERSMAX) return -PARDU_ERR_IDINVAL;
 // There is nothing to do if the the requested ID is already free
 if(StepperID[id]==STEPPER_ID_FREE) return -PARDU_ERR_ACTINVAL;

 // If we get here then the requested ID exists and is allocated so free it

 // Free string memory
 free(Stepperlist[id]->unitstr);
 free(Stepperlist[id]->lsname[0]);
 free(Stepperlist[id]->lsname[1]);
 free(Stepperlist[id]->lsname[2]);
 free(Stepperlist[id]->lsname[3]);
 free(Stepperlist[id]->lsname[4]);

 // Free the main memory, update the list element counter and list index status
 Act_Stepper_freecommon(id);

 // Update the global 'StageStatus' flag each time a stage motor is destroyed
 if(id <= ACT_ID_STAGE_Z){
     if((StepperID[ACT_ID_STAGE_X] == STEPPER_ID_FREE) || 
        (StepperID[ACT_ID_STAGE_Y] == STEPPER_ID_FREE)) {
         StageStatus = 0; // Missing an essential scanning axis entirely
     } else if(StepperID[ACT_ID_STAGE_Z] == STEPPER_ID_FREE) {
         StageStatus = ACT_ID_STAGE_XY; // Only X and Y remain
     }
 }
 
 return PARD_OK;
}

void Act_Stepper_freecommon(uint8_t id)
// Does dome memory freeing and variable maintenance that occurs multiple times
// in other functions. Saves duplicate code.
{
 if(Stepperlist[id]!=NULL) free(Stepperlist[id]);
 Stepperlist[id]=NULL;
 StepperID[id]=STEPPER_ID_FREE;
 NSteppers--;
 return;
}

int8_t Act_Binary_create(uint8_t id,const char *idstr)
// Adds a new binary actuator node to the list of binaries with its members set
// to nascent defaut values.
// Returns one of the PARD_ERR or negative PARDU_ERR_ constants (which see)
// A negative PARDU_ERR constant has the same meaning as the positive version
// but the negativity indicates that the error originated in the server not the
// Arduino so Pard_err_str() will report it as "[PARDUS] ..." instead of
// "[PARDUINO] ..."
{
 Act_BinaryNode *newmem;

 // Ensure the proposed name is at least 1 char long
 if(idstr == NULL || strlen(idstr)<1) return PARD_ERR_ACNOM;
 
 // Is the requested id invalid by being too high?
 if(id>=BINARIESMAX) return -PARDU_ERR_IDINVAL;

 // Is the requested id already taken?
 if(BinaryID[id]!=BINARY_ID_FREE) return -PARDU_ERR_IDTAKEN;

 // Is the list full?
 if(NBinaries>=BINARIESMAX) return -PARDU_ERR_NFULL;

 // If we get here then the requested ID number is available and within the
 // valid range of ID numbers and there is room in the list to allocate a new
 // actuator definition node so try to create the new node.

 // See if we can get memory for the new node
 newmem=malloc(sizeof(Act_BinaryNode));
 if(newmem==NULL) return -PARDU_ERR_NMFAIL;

 // Memory got OK so update the list pointer, number of elements counter and id.
 Binarylist[id]=newmem;
 NBinaries++;
 BinaryID[id]=id;

 // New node created so set the default values
 Binarylist[id]->on_val  = BIN_UNSET;
 Binarylist[id]->off_val = BIN_UNSET;
 Binarylist[id]->pin_bin = PIN_IS_UNSET;
 Binarylist[id]->maturity=0;

 // Set the actuator name to the supplied idstr
 snprintf(Binarylist[id]->Name,32,"%s",idstr);

 return PARD_OK;
}

int8_t Act_Binary_destroy(uint8_t id)
// Frees the Binary actuator with the supplied ID number.
// If that ID is not allocated it just returns PARDU_ERR_ACTINVAL.
{
 // There is nothing to do if there are no actuators allocated
 if(NBinaries==0) return -PARDU_ERR_ACTINVAL;
 // There is nothing to do if the requested ID is not valid
 if(id>=BINARIESMAX) return -PARDU_ERR_IDINVAL;
 // There is nothing to do if the the requested ID is already free
 if(BinaryID[id]==BINARY_ID_FREE) return -PARDU_ERR_ACTINVAL;

 // If we get here then the requested ID exists and is allocated so free it
 if(Binarylist[id]!=NULL) free(Binarylist[id]);
 Binarylist[id]=NULL;
 BinaryID[id]=BINARY_ID_FREE;
 NBinaries--;
 
 return PARD_OK;
}

int8_t Act_Mlevel_create(uint8_t id,const char *idstr)
// Adds a new M-level actuator node to the list of M-levels with its members set
// to nascent defaut values.
// Returns one of the PARD_ERR or negative PARDU_ERR_ constants (which see)
// A negative PARDU_ERR constant has the same meaning as the positive version
// but the negativity indicates that the error originated in the server not the
// Arduino so Pard_err_str() will report it as "[PARDUS] ..." instead of
// "[PARDUINO] ..."
{
 Act_MlevelNode *newmem;

 // Ensure the proposed name is at least 1 char long
 if(idstr == NULL || strlen(idstr)<1) return PARD_ERR_ACNOM;
 
 // Is the requested id invalid by being too high?
 if(id>=MLEVELSMAX) return -PARDU_ERR_IDINVAL;

 // Is the requested id already taken?
 if(MlevelID[id]!=MLEVEL_ID_FREE) return -PARDU_ERR_IDTAKEN;

 // Is the list full?
 if(NMlevels>=MLEVELSMAX) return -PARDU_ERR_NFULL;

 // If we get here then the requested ID number is available and within the
 // valid range of ID numbers and there is room in the list to allocate a new
 // actuator definition node so try to create the new node.

 // See if we can get memory for the new node
 newmem=malloc(sizeof(Act_MlevelNode));
 if(newmem==NULL) return -PARDU_ERR_NMFAIL;

 // Memory got OK so update the list pointer, number of elements counter and id.
 Mlevellist[id]=newmem;
 NMlevels++;
 MlevelID[id]=id;

 // New node created so set the default values
 Mlevellist[id]->ml_type = MLT_UNSET;
 Mlevellist[id]->ml_tt   = NULL;
 Mlevellist[id]->ntt     = 0;
 Mlevellist[id]->sztt    = 0;

 Mlevellist[id]->ml_pin[0] = PIN_IS_UNSET;
 Mlevellist[id]->ml_pin[1] = PIN_IS_UNSET;
 Mlevellist[id]->ml_pin[2] = PIN_IS_UNSET;
 Mlevellist[id]->ml_pin[3] = PIN_IS_UNSET;
 Mlevellist[id]->ml_pin[4] = PIN_IS_UNSET;
 Mlevellist[id]->ml_pin[5] = PIN_IS_UNSET;
 Mlevellist[id]->ml_pin[6] = PIN_IS_UNSET;
 Mlevellist[id]->ml_pin[7] = PIN_IS_UNSET;

 Mlevellist[id]->last_level= MLEVEL_RESET;
 Mlevellist[id]->maturity  = 0;

 Mlevellist[id]->npins     = 0;
 Mlevellist[id]->pingaps   = 0;
 Mlevellist[id]->max_level = 0;

 Mlevellist[id]->ml_psc    = PWM_PSC_8;

 Mlevellist[id]->last.freq = UINT32_MAX;
 Mlevellist[id]->last.duty = 200;

 // Set the actuator name to the supplied idstr
 snprintf(Mlevellist[id]->Name,32,"%s",idstr);

 return PARD_OK;
}

int8_t Act_Mlevel_destroy(uint8_t id)
// Frees the Mlevel actuator with the supplied ID number.
// If that ID is not allocated it just returns PARDU_ERR_ACTINVAL.
{
 // There is nothing to do if there are no actuators allocated
 if(NMlevels==0) return -PARDU_ERR_ACTINVAL;
 // There is nothing to do if the requested ID is not valid
 if(id>=MLEVELSMAX) return -PARDU_ERR_IDINVAL;
 // There is nothing to do if the the requested ID is already free
 if(MlevelID[id]==MLEVEL_ID_FREE) return -PARDU_ERR_ACTINVAL;

 // If we get here then the requested ID exists and is allocated so free it:
 // First free any translation table:
 if(Mlevellist[id]->ml_tt!=NULL){
   free(Mlevellist[id]->ml_tt);
   Mlevellist[id]->ml_tt = NULL; 
  }
 // Then free the rest:
 if(Mlevellist[id]!=NULL) free(Mlevellist[id]);
 Mlevellist[id]=NULL;
 MlevelID[id]=MLEVEL_ID_FREE;
 NMlevels--;
 
 return PARD_OK;
}

double servo_calibri(uint8_t id)
// Calculates servo angle to microseconds calibration factor to be used thus:
// musec = srv_min + (theta - theta_min)*calibri;
// Return the calibration factor or 0.0 if calculation is not possible.
{
 double calib;
 double deltheta = 0.0;

 deltheta = Servolist[id]->theta_max - Servolist[id]->theta_min; 
 if(deltheta < 1.0e-9) return 0.0;

 calib = (double)(Servolist[id]->srv_max - Servolist[id]->srv_min) / deltheta;

 return calib;
}

uint32_t server_musecs(uint8_t id, double position)
// Convert a user supplied position (must be >=0.0) to servo microseconds.
{
 uint32_t musec = 0;

 // 0. Regardless of units, is this the SERVO_RESET signal? If so pass it
 //    through immediately:
 if(position >= (SERVO_RESET-1)) return SERVO_RESET;

 // 1. If in Unit Mode (= degrees), scale to microseconds.
 if (sv_Unitype[ACT_SERVO]){
     double dsum = (position - Servolist[id]->theta_min) * Servolist[id]->calibri;
     
     if (dsum < 0.0) {
         musec = Servolist[id]->srv_min; // Fall back to absolute minimum pulse
     } else {
         // Correctly round the scaled value and add the base offset
         musec = (uint32_t)lround(dsum) + Servolist[id]->srv_min;
     }
 } 
 // 2. Otherwise (if in raw microseconds mode) just convert directly
 else {
     if (position < 0.0) {
         musec = 0;
     } else {
         musec = (uint32_t)lround(position);
     }
 }

 // 3. Clamp to absolute maximum and minimum to protect hardware
 if (musec > Servolist[id]->srv_max) {
     musec = Servolist[id]->srv_max;
 }
 if (musec < Servolist[id]->srv_min) {
     musec = Servolist[id]->srv_min;
 }

 return musec;
}

int8_t Act_Servo_create(uint8_t id,const char *idstr)
// Adds a new Servo actuator node to the list of Servos with its members set
// to nascent defaut values.
// Returns one of the PARD_ERR or negative PARDU_ERR_ constants (which see)
// A negative PARDU_ERR constant has the same meaning as the positive version
// but the negativity indicates that the error originated in the server not the
// Arduino so Pard_err_str() will report it as "[PARDUS] ..." instead of
// "[PARDUINO] ..."
{
 Act_ServoNode *newmem;

 // Ensure the proposed name is at least 1 char long
 if(idstr == NULL || strlen(idstr)<1) return PARD_ERR_ACNOM;
 
 // Is the requested id invalid by being too high?
 if(id>=SERVOSMAX) return -PARDU_ERR_IDINVAL;

 // Is the requested id already taken?
 if(ServoID[id]!=SERVO_ID_FREE) return -PARDU_ERR_IDTAKEN;

 // Is the list full?
 if(NServos>=SERVOSMAX) return -PARDU_ERR_NFULL;

 // If we get here then the requested ID number is available and within the
 // valid range of ID numbers and there is room in the list to allocate a new
 // actuator definition node so try to create the new node.

 // See if we can get memory for the new node
 newmem=malloc(sizeof(Act_ServoNode));
 if(newmem==NULL) return -PARDU_ERR_NMFAIL;

 // Memory got OK so update the list pointer, number of elements counter and id.
 Servolist[id]=newmem;
 NServos++;
 ServoID[id]=id;

 // New node created so set the default values
 Servolist[id]->pin_srv   = PIN_IS_UNSET;
 Servolist[id]->pin_pos   = PIN_IS_UNSET;
 Servolist[id]->freq      = SRV_FREQ_MID50;
 Servolist[id]->srv_min   = 544;
 Servolist[id]->srv_max   = 2400;
 Servolist[id]->theta_min = 0.0;
 Servolist[id]->theta_max = 180.0;
 Servolist[id]->calibri   = servo_calibri(id);
 Servolist[id]->destn_us  = SERVO_RESET;
 Servolist[id]->isd_min   = 555;  // Generally safe for most 50Hz servos
 Servolist[id]->maturity  = 0;

 // Set the actuator name to the supplied idstr
 snprintf(Servolist[id]->Name,32,"%s",idstr);

 return PARD_OK;
}

int8_t Act_Servo_destroy(uint8_t id)
// Frees the Servo actuator with the supplied ID number.
// If that ID is not allocated it just returns PARDU_ERR_ACTINVAL.
{
 // There is nothing to do if there are no actuators allocated
 if(NServos==0) return -PARDU_ERR_ACTINVAL;
 // There is nothing to do if the requested ID is not valid
 if(id>=SERVOSMAX) return -PARDU_ERR_IDINVAL;
 // There is nothing to do if the the requested ID is already free
 if(ServoID[id]==SERVO_ID_FREE) return -PARDU_ERR_ACTINVAL;

 // If we get here then the requested ID exists and is allocated so free it:
 if(Servolist[id]!=NULL) free(Servolist[id]);
 Servolist[id]=NULL;
 ServoID[id]=SERVO_ID_FREE;
 NServos--;
 
 return PARD_OK;
}

int8_t Act_Sensor_create(uint8_t id,const char *idstr)
// Adds a new Sensor actuator node to the list of Sensors with its members set
// to nascent defaut values.
// Returns one of the PARD_ERR or negative PARDU_ERR_ constants (which see)
// A negative PARDU_ERR constant has the same meaning as the positive version
// but the negativity indicates that the error originated in the server not the
// Arduino so Pard_err_str() will report it as "[PARDUS] ..." instead of
// "[PARDUINO] ..."
{
 Act_SensorNode *newmem;

 // Ensure the proposed name is at least 1 char long
 if(idstr == NULL || strlen(idstr)<1) return PARD_ERR_ACNOM;
 
 // Is the requested id invalid by being too high?
 if(id>=SENSORSMAX) return -PARDU_ERR_IDINVAL;

 // Is the requested id already taken?
 if(SensorID[id]!=SENSOR_ID_FREE) return -PARDU_ERR_IDTAKEN;

 // Is the list full?
 if(NSensors>=SENSORSMAX) return -PARDU_ERR_NFULL;

 // If we get here then the requested ID number is available and within the
 // valid range of ID numbers and there is room in the list to allocate a new
 // actuator definition node so try to create the new node.

 // See if we can get memory for the new node
 newmem=malloc(sizeof(Act_SensorNode));
 if(newmem==NULL) return -PARDU_ERR_NMFAIL;

 // Memory got OK so update the list pointer, number of elements counter and id.
 Sensorlist[id]=newmem;
 NSensors++;
 SensorID[id]=id;

 // New node created so set the default values
 Sensorlist[id]->pin_sense   = PIN_IS_UNSET;
 Sensorlist[id]->type        = SEN_UNSET;
 Sensorlist[id]->maturity    = 0;

 // Set the actuator name to the supplied idstr
 snprintf(Sensorlist[id]->Name,32,"%s",idstr);

 return PARD_OK;
}

int8_t Act_Sensor_destroy(uint8_t id)
// Frees the Sensor actuator with the supplied ID number.
// If that ID is not allocated it just returns PARDU_ERR_ACTINVAL.
{
 // There is nothing to do if there are no actuators allocated
 if(NSensors==0) return -PARDU_ERR_ACTINVAL;
 // There is nothing to do if the requested ID is not valid
 if(id>=SENSORSMAX) return -PARDU_ERR_IDINVAL;
 // There is nothing to do if the the requested ID is already free
 if(SensorID[id]==SENSOR_ID_FREE) return -PARDU_ERR_ACTINVAL;

 // If we get here then the requested ID exists and is allocated so free it:
 if(Sensorlist[id]!=NULL) free(Sensorlist[id]);
 Sensorlist[id]=NULL;
 SensorID[id]=SENSOR_ID_FREE;
 NSensors--;
 
 return PARD_OK;
}

uint8_t actuator_exists(uint8_t actype,uint8_t actid)
// Checks if the specified actuator exists. Retrun 1 if it does, 0 if not..
{
 switch(actype){
   case ACT_STEPPER:
    switch(actid){
     case ACT_ID_STAGE_XY:
      if(StageStatus>=ACT_ID_STAGE_XY) return 1;
     break;
     case ACT_ID_STAGE_XYZ:
      if(StageStatus==ACT_ID_STAGE_XYZ) return 1;
     break;
     default:
      if(actid < STEPPERSMAX && StepperID[actid]!=STEPPER_ID_FREE) return 1;
     break;
    }
   break;
   case ACT_BINARY: 
     if(actid < BINARIESMAX && BinaryID[actid]!=BINARY_ID_FREE) return 1;
   break;
   case ACT_MLEVEL:
     if(actid < MLEVELSMAX && MlevelID[actid]!=MLEVEL_ID_FREE) return 1;
   break;
   case ACT_SERVO:
     if(actid < SERVOSMAX && ServoID[actid]!=SERVO_ID_FREE) return 1;
   break;
   case ACT_SENSOR:
     if(actid < SENSORSMAX && SensorID[actid]!=SENSOR_ID_FREE) return 1;
   break;
   default:
    return 0;
   break;
  }
 return 0;
}

uint8_t act_calc_motion(uint32_t dirsteps,
                        uint32_t duration,
                        double p_acc,
                        uint32_t delay_min,
                        uint32_t delay_max_min)
// Calculate the motion parameters required to make the breakout board perform
// dirsteps steps in duration microseconds with p_acc proportion of those steps
// involved in acceleration up to a top speed with delay_min microseconds of
// inter-step delay but not allowing the slowest speed to have more than
// delay_max_min microseconds of inter-step delay.
// If 'duration' is supplied as 0 then the duration is automatically calculated
// together with suitable accel and decel curves lasting no longer than
// MAX_ACCEL_DURATION microseconds for each phase (accel and decel).
// This function fills in the global motion order form struct Order_Motion with
// the results. This has the following members:
// uint32_t   delay_max;   // The isd for the very first step (in microseconds)
// uint32_t   ash;         // Half the accel/decel steps (at each end)
// uint32_t   delay_diff_diff; // acceleration constant
// uint32_t   xsresid;     // Excess residual steps to be removed from the end
// uint32_t   cvsteps;     // Constant velocity steps
// Return values:
// 0 = all OK, motion will be done with acceleration and deceleration.
// 1 = p_acc effectively 0 - No acceleration requested so constant motion will
//                           be done (this will be with isd = delay_max_min)
// 2 = ash < 2,          default to constant motion
// 3 = <asresid> >= ash, default to constant motion
// 4 = delay_diff_diff (ddd) <= 0.0, default to constant motion
// 5 = delay_max < delay_max_min, default to constant motion

{
 uint32_t  ash, csh, delay_max; 
 uint32_t  asresid;
 uint32_t  ecvstep, ashashp1;
 uint32_t  predicted_steps;
 uint32_t  cs_add, cs_sub;
 double    ddd, dur_2;   
 int retval = 0;   
 // Some values for the auto-duration calculation mode:
 const uint32_t ABSOLUTE_TAIL_CAP = 100000;
 const uint32_t MAX_ACCEL_DURATION = 3000000; // 3 seconds

   //----------------------------------------------------------------------\\.
  //     CHECKS AND CALCS COMMON TO BOTH MANUAL AND AUTO-DURATION MODES     \\.
 //--------------------------------------------------------------------------\\.

 // First look for the special case with p_acc is effectively zero (indicating
 // that the user wants no acceleration i.e. constant velocity motion throughout
 // with isd of delay_max_min. We should never get to this function if dirsteps
 // is 0 but I include the case here for completeness / robustness):
 if (dirsteps == 0 || p_acc < 1.0e-8) { retval = 1; goto constant_motion; }

 // Look for some other incompatible inputs that would effectively rule out
 // acceleration and default to constant motion immediately:
 if (delay_min >= delay_max_min) { retval = 4; goto constant_motion; }
 if (delay_min >= ABSOLUTE_TAIL_CAP) { retval = 4; goto constant_motion; }

 // Calculate ash and csh
 ash = (uint32_t)(0.5+p_acc*(double)dirsteps/4.0);
 csh = (uint32_t)(0.5+(1.0-p_acc)*(double)dirsteps/2.0);

 // Sanity check on the ash parameter
 // Not much point having an acceleration 'curve sigmoid' with fewer than 2
 // points in each limb of the sigma: 
 if(ash < 2) { retval = 2; goto constant_motion; }

 // Calculate the predicted number of steps. Due to float-to-integer round-off
 // error this might be a small number (upto 6 max.) of steps fewer or greater
 // than the originally requrested figure (dirsteps).
 predicted_steps=2*(ash+ash+csh);

 // If this 'predicted' number is lower than dirsteps we will simply add the
 // deficit to the constant velocity steps at the expense of possibly increasing
 // the duration (I prioritise location over speed because there will always be
 // some error in speed needing periodic adjustments but there need not be error
 // in location due to not performing the requested number of steps (there may
 // be error in location for other reasons but we do not need to add to those
 // reasons here).
 cs_add=0;
 if(predicted_steps<dirsteps) cs_add = dirsteps-predicted_steps;

 // If the predicted number of steps is higher than dirsteps then we can 
 // subtract the excess from the constant velocity steps if we have room to do
 // so. If we can't subtract all the excess from the constant velocity steps
 // then we can simply terminate the motion that residual number of steps before
 // reaching the very end of the deceleration phase.
 asresid=0;
 cs_sub=0;
 if(predicted_steps>dirsteps){
    cs_sub = predicted_steps - dirsteps;
    if(cs_sub>(csh+csh)){
        asresid=cs_sub-(csh+csh);
        csh=0;
        cs_sub=0;
      }
   }

 // Check if any correction is possible:
 if(asresid && asresid>=ash){ retval = 3;  goto constant_motion; }
 
 ashashp1 = ash * (ash + 1);

 //--------------------------------------------------------------------------//.
  //                     END OF COMMON MODE CHECKS AND CALCS                //.
   //----------------------------------------------------------------------//.


 if(duration == 0){
   // AUTO-CALCULATE DURATION MODE

   uint32_t baseline_duration = (uint32_t)(0.5 + 
                                        ((double)delay_min * (double)dirsteps) /
                                                                 (0.1 + p_acc));
   dur_2 = (double)baseline_duration / 2.0;
   
   ddd = (dur_2 - 2.0 * (double)(ash * delay_min) - (double)csh * delay_min) /
                                    (double)(ashashp1 + ((ash + 1) * ashashp1));
   
   if(ddd <= 0.0){ retval = 4; goto constant_motion; }
   
   delay_max = delay_min + (uint32_t)(0.5 + (ddd * (double)ashashp1));

   if(delay_max > ABSOLUTE_TAIL_CAP) delay_max = ABSOLUTE_TAIL_CAP;

   uint32_t max_allowed_ash = MAX_ACCEL_DURATION / (delay_max + delay_min);
   
   if(ash > max_allowed_ash){
     ash = max_allowed_ash;
     if (ash < 2) ash = 2; 
     
     csh = (uint32_t)(0.5 + ((double)dirsteps - 4.0 * (double)ash) / 2.0);
     ashashp1 = ash * (ash + 1);

     // Because ash and csh have been modified, we must recalculate 
     // predicted_steps and update the cs_add/cs_sub compensation registers to
     // ensure the caller's choice of dirsteps is strictly adhered to.
     predicted_steps = 2 * (ash + ash + csh);
     cs_add = 0;
     cs_sub = 0;
     asresid = 0;

     if(predicted_steps < dirsteps){
       cs_add = dirsteps - predicted_steps;
      }
     else if(predicted_steps > dirsteps){
       cs_sub = predicted_steps - dirsteps;
       if(cs_sub > (csh + csh)){
           asresid = cs_sub - (csh + csh);
           csh = 0;
           cs_sub = 0;
        }
      }

    }

   ddd = (double)(delay_max - delay_min) / (double)ashashp1;
   
   if((float)ddd <= 0.0f){
     retval = 4;
     goto constant_motion;
    }

   dur_2 = ddd * (double)(ashashp1 + ((ash + 1) * ashashp1)) + 
                              2.0 * (double)(ash * delay_min) + csh * delay_min;
   duration = (uint32_t)(0.5 + (dur_2 * 2.0));

 } else {
   // MANUAL DURATION MODE

   // We are going to be using D/2 in all our calculations so let us calculate
   // that now:
   dur_2 = (double)duration/2.0;
  
  
   // Calculate delay_diff_diff (also called 'z' or 'ddd') and ensure it is >0.0
   ddd = (dur_2 - 2.0*(double)(ash*delay_min)  - csh*delay_min) /
                 (double)(ashashp1 + ((ash+1)*ashashp1));
  
   if(ddd<=0.0){ retval = 4;  goto constant_motion; }
  
   // We can now calculate delay_max:
   delay_max = delay_min + (uint32_t)(0.5+(ddd*(double)ashashp1));
  
 }

 // Check if delay_max is big enough for reliable motion:
 if(delay_max < delay_max_min){ retval = 5; goto constant_motion; }

 // Calculate the constant velocity steps after any corrections
 ecvstep=(csh+csh)+cs_add;
 ecvstep-=cs_sub;

   // --- DYNAMIC BIT-SHIFT CALCULATION (finding B2, corrected 22.10.2026) ---
   // The firmware (parduino_fixed8/fixed9, main(), the four limbs of the
   // ramp) forms, in 32-bit unsigned arithmetic,
   //    dd = j*q  for j = 0 .. ash   (the largest value is ash*q), and
   //    dd + 2^(s-1)                 (the rounding offset; dd <= (ash-1)*q).
   // Both must stay below 2^32 for the q that is actually SENT (rounded).
   // So choose the largest s, from 31 down, for which they do.  The rounding
   // offset 2^(s-1) must be included: without it the sum wraps whenever
   // ash*ddd lies within half a microsecond below 2^(32-s) (for example any
   // ash*ddd from 1.5 to 2 us at s = 31), and the ramp is then wrong.
   uint8_t shift_s = 31;
   uint32_t q_delay_diff_diff;
   for(;;){
     double qd = floor(0.5 + ddd * (double)(1UL << shift_s));   // the q to send
     if( qd <= 4294967295.0
         && (double)ash * qd <= 4294967295.0
         && ((double)ash - 1.0) * qd + (double)(1UL << (shift_s - 1)) <= 4294967295.0 ){
        q_delay_diff_diff = (uint32_t)qd;
        break;
       }
     // No shift can work only if ash*ddd >= 2^31 us, which no uint32_t
     // delay_max allows; constant motion is the safe answer if it ever did,
     // with retval 4, the code this function already uses when the timing
     // leaves no room for a ramp (delay_min >= delay_max_min or >=
     // ABSOLUTE_TAIL_CAP, ddd <= 0).
     if(shift_s == 1){ retval = 4; goto constant_motion; }
     shift_s--;
    }


 // Fill in the motion order form
 Order_Motion.delay_max = delay_max;   
 Order_Motion.ash = ash;               
 Order_Motion.delay_diff_diff = q_delay_diff_diff; // Transmitting 'q' now
 Order_Motion.shift_s = shift_s;                   // Transmitting custom shift scale
 Order_Motion.xsresid = asresid;       
 Order_Motion.cvsteps = ecvstep;       
 return retval;

constant_motion:
 // Fill in the motion order form for constant motion
 Order_Motion.delay_max = delay_max_min;   
 Order_Motion.ash = 0;                     
 Order_Motion.delay_diff_diff = 0;         
 Order_Motion.shift_s = 16;                        // Defaults back to 16 if no acceleration
 Order_Motion.xsresid = 0;                 
 Order_Motion.cvsteps = dirsteps;          
 return retval;
}

#define AIS_LOCAL 1
#define AIS_BOB   2
#define AIS_ERROR 3
 
int act_get_source(uint8_t actype, char *errmsg)
// Check where an infoitem needs to be read from for an actuator of type actype.
// This functions assumes you have filled in the infoitem part of the global
// Order_Info order form.
// Return AIS_LOCAL If it should be read from the local struct
// Return AIS_BOB   If it should be read from the breakout board (e.g. Arduino)
// Return AIS_ERROR If neither apply for whatever reason (that reason will be
//                  printed into errmsg).
{
 int retval=AIS_ERROR;

 switch(actype){
  case ACT_STEPPER:
   switch(Order_Info.infotype){
    case INFO_DIRN1:                       // LG
    case INFO_DIRN2:                       // LG
    case INFO_MSDENOM:                     // LG
    case INFO_FASTMSD:                     // LG
    case INFO_MSDEMAX:     // Server only  // LG
    case INFO_PULWIDTH:                    // LG
    case INFO_BLSTEPS:                     // LG        // Read only
    case INFO_ONESTEP:                     // LG        // Read only
    case INFO_BLSTMAX:     // Server only  // LG
    case INFO_CALIMAX:     // Server only  // LG
    case INFO_SW_LIM1:                     // LG
    case INFO_SW_LIM2:                     // LG
    case INFO_NHS    :                     // LG
    case INFO_ISD_HOBL:                    // LG
    case INFO_TENSTEPS1:                   // LG
    case INFO_TENSTEPS2:                   // LG
    case INFO_PIN_PUL:                     // LG
    case INFO_PIN_DIR:                     // LG
    case INFO_PIN_LIM1:                    // LG
    case INFO_PIN_LIM2:                    // LG
    case INFO_PIN_DIAG:                    // LG
    case INFO_PIN_ENA:                     // LG
    case INFO_PIN_MS1:                     // LG
    case INFO_PIN_MS2:                     // LG
    case INFO_PIN_MS3:                     // LG
    case INFO_SD_TYPE:                     // LG
    case INFO_NAME_LS1:    // Server only  // LG
    case INFO_NAME_LS2:    // Server only  // LG
    case INFO_NAME_SW1:    // Server only  // LG
    case INFO_NAME_SW2:    // Server only  // LG
    case INFO_NAME_UNI:    // Server only  // LG
    case INFO_MATURITY:    // Server only  // LG        // Read only
    case INFO_UNIT_TYPE:   // Server only  // LG        // Read only
    retval=AIS_LOCAL;
    break;
    case INFO_DIRNC:                       // Ask Ardu
    case INFO_ENA_STATE:                   // Ask Ardu
    case INFO_MMODE:       // Ardu only    // Ask Ardu
    case INFO_TURBO_BST:   // Ardu only    // Ask Ardu
    case INFO_SG_BUFFPC:   // Ardu only    // Ask Ardu
    case INFO_SG_THRESH:   // Ardu only    // Ask Ardu
    case INFO_HOMED_TO:                    // Ask Ardu  // Read only
    case INFO_DIRNP    :   // Ardu only    // Ask Ardu  // Read only
    case INFO_TMCSGVAL :   // Ardu only    // Ask Ardu  // Read only
    case INFO_TMCSTATUS:   // Ardu only    // Ask Ardu  // Read only
    case INFO_TMCCHCONF:   // Ardu only    // Ask Ardu  // Read only
    case INFO_TMCMSCNT :   // Ardu only    // Ask Ardu  // Read only
    case INFO_DURALAM  :   // Ardu only    // Ask Ardu  // Read only
    case INFO_STEPLAM  :   // Ardu only    // Ask Ardu  // Read only
    case INFO_STEPCOUNT:                   // Ask Ardu
    case INFO_LIMSTATE :                   // Ask Ardu  // Read only
    case INFO_XYZCOORDS:                   // Ask Ardu
    retval=AIS_BOB;
    break;
    default:
     sprintf(errmsg,"[PARDUS] Info item not applicable to this actuator type.");
    break;
   }
  break;

  case ACT_BINARY:
   switch(Order_Info.infotype){
    case INFO_ON_VAL:                      // LG
    case INFO_OFF_VAL:                     // LG
    case INFO_PIN_BIN:                     // LG
    case INFO_MATURITY:    // Server only  // LG        // Read only
    retval=AIS_LOCAL;
    break;
    case INFO_BIN_STATE:                   // Ask Ardu
    retval=AIS_BOB;
    break;
    default:
     sprintf(errmsg,"[PARDUS] Can't get this info item for this actuator type.");
    break;
   }
  break;

  case ACT_MLEVEL:
   switch(Order_Info.infotype){
    case INFO_ML_TYPE:                     // LG
    case INFO_ML_PSC:                      // LG
    case INFO_PIN_P0:                      // LG
    case INFO_PIN_P1:                      // LG
    case INFO_PIN_P2:                      // LG
    case INFO_PIN_P3:                      // LG
    case INFO_PIN_P4:                      // LG
    case INFO_PIN_P5:                      // LG
    case INFO_PIN_P6:                      // LG
    case INFO_PIN_P7:                      // LG
    case INFO_MATURITY:    // Server only  // LG        // Read only
    case INFO_ML_LEVEL:                    // LG
    case INFO_ML_TTABLE:   // Server only  // LG
    retval=AIS_LOCAL;
    break;
    case INFO_ML_STATE:    // Ardu only    // Ask Ardu  // Read only
    case INFO_ML_TIMER:    // Ardu only    // Ask Ardu  // Read only
    case INFO_ML_FREQ:     // Ardu only    // Ask Ardu  // Read only
    retval=AIS_BOB;
    break;
    default:
     sprintf(errmsg,"[PARDUS] Can't get this info item for this actuator type.");
    break;
   }
  break;

  case ACT_SERVO:
   switch(Order_Info.infotype){
    case INFO_SRV_STATE:   // Ardu only    // Ask Ardu  // Read only
    case INFO_SRV_TIMER:   // Ardu only    // Ask Ardu  // Read only
    case INFO_SRV_AFB:     // Ardu only    // Ask Ardu  // Read only
    case INFO_SRV_SPOS:    // Ardu only    // Ask Ardu
     retval=AIS_BOB;
    break;
    case INFO_SRV_FREQ:                    // LG
    case INFO_SRV_PINS:                    // LG
    case INFO_SRV_PINP:                    // LG
    case INFO_SRV_QPOS:                    // LG
    case INFO_SRV_MIN:                     // LG
    case INFO_SRV_MAX:                     // LG
    case INFO_SRV_ISD:                     // LG
    case INFO_MATURITY:    // Server only  // LG        // Read only
    case INFO_SRV_CALIB:   // Server only  // LG        // Read only
    case INFO_SRV_THMAX:   // Server only  // LG
    case INFO_SRV_THMIN:   // Server only  // LG
    retval=AIS_LOCAL;
    break;
    default: // This would be a program error (bug)
     sprintf(errmsg,"[PARDUS] Info item not applicable to this actuator type.");
    break;
   }
  break;

  case ACT_SENSOR:
   switch(Order_Info.infotype){
    case INFO_SEN_VAL:     // Ardu only    // Ask Ardu  // Read only
     retval=AIS_BOB;
    break;
    case INFO_SEN_TYPE:                    // LG
    case INFO_SEN_PIN:                     // LG
    case INFO_MATURITY:    // Server only  // LG        // Read only
    retval=AIS_LOCAL;
    break;
    default: // This would be a program error (bug)
     sprintf(errmsg,"[PARDUS] Info item not applicable to this actuator type.");
    break;
   }
  break;

  default:
   sprintf(errmsg,"[PARDUS] Actuator not recognised.");
  break;
 }

 return retval;
}

int act_set_source(uint8_t actype, char *errmsg)
// Check where an infoitem needs to be set for an actuator of type actype.
// This functions assumes you have filled in the infoitem part of the global
// Order_Info order form.
// Return AIS_LOCAL If it can ONLY be set on the local actuator struct
// Return AIS_BOB   If it needs be set on the breakout board
// Return AIS_ERROR If neither apply for whatever reason (that reason will be
//                  printed into errmsg).
{
 int retval=AIS_ERROR;

 switch(actype){
  case ACT_STEPPER:
   switch(Order_Info.infotype){
    case INFO_MSDEMAX:      // Server only  // LG
    case INFO_BLSTMAX:      // Server only  // LG
    case INFO_CALIMAX:      // Server only  // LG
    case INFO_TENSTEPS1:                    // LG
    case INFO_TENSTEPS2:                    // LG
    case INFO_NAME_LS1:     // Server only  // LG
    case INFO_NAME_LS2:     // Server only  // LG
    case INFO_NAME_SW1:     // Server only  // LG
    case INFO_NAME_SW2:     // Server only  // LG
    case INFO_NAME_UNI:     // Server only  // LG
    retval=AIS_LOCAL;
    break;
    case INFO_DIRN1:                       // LG
    case INFO_DIRN2:                       // LG
    case INFO_DIRNC:                       // Ask Ardu
    case INFO_MSDENOM:                     // LG
    case INFO_FASTMSD:                     // LG
    case INFO_PULWIDTH:                    // LG
    case INFO_SW_LIM1:                     // LG
    case INFO_SW_LIM2:                     // LG
    case INFO_NHS    :                     // LG
    case INFO_ISD_HOBL:                    // LG
    case INFO_PIN_PUL:                     // LG
    case INFO_PIN_DIR:                     // LG
    case INFO_PIN_LIM1:                    // LG
    case INFO_PIN_LIM2:                    // LG
    case INFO_PIN_DIAG:                    // LG
    case INFO_PIN_ENA:                     // LG
    case INFO_PIN_MS1:                     // LG
    case INFO_PIN_MS2:                     // LG
    case INFO_PIN_MS3:                     // LG
    case INFO_SD_TYPE:                     // LG
    case INFO_ENA_STATE:                   // Ask Ardu
    case INFO_MMODE:       // Ardu only    // Ask Ardu
    case INFO_TURBO_BST:   // Ardu only    // Ask Ardu
    case INFO_SG_BUFFPC:   // Ardu only    // Ask Ardu
    case INFO_SG_THRESH:   // Ardu only    // Ask Ardu
    case INFO_STEPCOUNT:                   // Ask Ardu
    case INFO_XYZCOORDS:                   // Ask Ardu
    retval=AIS_BOB;
    break;
    case INFO_BLSTEPS :                     // LG        // Read only
    case INFO_ONESTEP :                     // LG        // Read only
    case INFO_MATURITY:     // Server only  // LG        // Read only
    case INFO_UNIT_TYPE:    // Server only  // LG        // Read only
    case INFO_HOMED_TO:                     // Ask Ardu  // Read only
    case INFO_DIRNP   :     // Ardu only    // Ask Ardu  // Read only
    case INFO_TMCSGVAL :    // Ardu only    // Ask Ardu  // Read only
    case INFO_TMCSTATUS:    // Ardu only    // Ask Ardu  // Read only
    case INFO_TMCCHCONF:    // Ardu only    // Ask Ardu  // Read only
    case INFO_TMCMSCNT :    // Ardu only    // Ask Ardu  // Read only
    case INFO_DURALAM :     // Ardu only    // Ask Ardu  // Read only
    case INFO_STEPLAM :     // Ardu only    // Ask Ardu  // Read only
    case INFO_LIMSTATE:                     // Ask Ardu  // Read only
     sprintf(errmsg,"[PARDUS] It is forbidden to set a read-only value.");
    break;
    default:
     sprintf(errmsg,"[PARDUS] Info item not applicable to this actuator type.");
    break;
   }
  break;

  case ACT_BINARY:
   switch(Order_Info.infotype){
    case INFO_ON_VAL:                      // LG
    case INFO_OFF_VAL:                     // LG
    case INFO_PIN_BIN:                     // LG
    case INFO_BIN_STATE:                   // Ask Ardu
    retval=AIS_BOB;
    break;
    case INFO_MATURITY:     // Server only  // LG        // Read only
     sprintf(errmsg,"[PARDUS] It is forbidden to set a read-only value.");
    break;
    default:
     sprintf(errmsg,"[PARDUS] Info item not applicable to this actuator type.");
    break;
   }
  break;

  case ACT_MLEVEL:
   switch(Order_Info.infotype){
    case INFO_ML_TYPE:                     // LG
    case INFO_ML_PSC:                      // LG
    case INFO_PIN_P0:                      // LG
    case INFO_PIN_P1:                      // LG
    case INFO_PIN_P2:                      // LG
    case INFO_PIN_P3:                      // LG
    case INFO_PIN_P4:                      // LG
    case INFO_PIN_P5:                      // LG
    case INFO_PIN_P6:                      // LG
    case INFO_PIN_P7:                      // LG
    case INFO_ML_LEVEL:                    // LG
     retval=AIS_BOB;
    break;
    case INFO_ML_STATE:    // Ardu only    // Ask Ardu  // Read only
    case INFO_ML_TIMER:    // Ardu only    // Ask Ardu  // Read only
    case INFO_MATURITY:    // Server only  // LG        // Read only
    case INFO_ML_FREQ:     // Ardu only    // Ask Ardu  // Read only
     sprintf(errmsg,"[PARDUS] It is forbidden to set a read-only value.");
    break;
    case INFO_ML_TTABLE:   // Server only  // LG
    retval=AIS_LOCAL;
    break;
    default: // This would be a program error (bug)
     sprintf(errmsg,"[PARDUS] Info item not applicable to this actuator type.");
    break;
   }
  break;

  case ACT_SERVO:
   switch(Order_Info.infotype){
    case INFO_SRV_FREQ:                    // LG
    case INFO_SRV_PINS:                    // LG
    case INFO_SRV_PINP:                    // LG
    case INFO_SRV_SPOS:    // Ardu only    // Ask Ardu
    case INFO_SRV_QPOS:                    // LG
    case INFO_SRV_MIN:                     // LG
    case INFO_SRV_MAX:                     // LG
    case INFO_SRV_ISD:                     // LG
     retval=AIS_BOB;
    break;
    case INFO_SRV_STATE:   // Ardu only    // Ask Ardu  // Read only
    case INFO_SRV_TIMER:   // Ardu only    // Ask Ardu  // Read only
    case INFO_SRV_AFB:     // Ardu only    // Ask Ardu  // Read only
    case INFO_MATURITY:    // Server only  // LG        // Read only
    case INFO_SRV_CALIB:   // Server only  // LG        // Read only
     sprintf(errmsg,"[PARDUS] It is forbidden to set a read-only value.");
    break;
    case INFO_SRV_THMAX:   // Server only  // LG
    case INFO_SRV_THMIN:   // Server only  // LG
    retval=AIS_LOCAL;
    break;
    default: // This would be a program error (bug)
     sprintf(errmsg,"[PARDUS] Info item not applicable to this actuator type.");
    break;
   }
  break;

  case ACT_SENSOR:
   switch(Order_Info.infotype){
    case INFO_SEN_PIN:                      // LG
    case INFO_SEN_TYPE:                     // LG
     retval=AIS_BOB;
    break;
    case INFO_SEN_VAL:     // Ardu only    // Ask Ardu  // Read only
    case INFO_MATURITY:    // Server only  // LG        // Read only
     sprintf(errmsg,"[PARDUS] It is forbidden to set a read-only value.");
    break;
    default: // This would be a program error (bug)
     sprintf(errmsg,"[PARDUS] Info item not applicable to this actuator type.");
    break;
   }
  break;

  default:
   sprintf(errmsg,"[PARDUS] Actuator not recognised.");
  break;
 }

 return retval;
}

void act_get_from_local(uint8_t actype, uint8_t actidx)
// Fill in the 'value' part of the global Order_Info struct with the required
// data (infotype) from the actuator specified by actype and actidx
// This functions assumes you have filled in the infoitem part of the global
// Order_Info order form struct.
{
 uint32_t idx=0;

 switch(actype){
  case ACT_STEPPER:
   switch(Order_Info.infotype){
    case INFO_DIRN1:                       // LG
      Order_Info.value.ui8val=Stepperlist[actidx]->dirn1;
    break;
    case INFO_DIRN2:                       // LG
      Order_Info.value.ui8val=Stepperlist[actidx]->dirn2;
    break;
    case INFO_PULWIDTH:                    // LG
      Order_Info.value.ui8val=Stepperlist[actidx]->pulwidth;
    break;
    case INFO_MSDENOM:                     // LG
      Order_Info.value.ui32val=Stepperlist[actidx]->msdenom;
    break;
    case INFO_FASTMSD:                     // LG
      Order_Info.value.ui32val=Stepperlist[actidx]->fastmsd;
    break;
    case INFO_MSDEMAX:     // Server only  // LG
      Order_Info.value.ui32val=Stepperlist[actidx]->maxmsdenom;
    break;
    case INFO_BLSTEPS:                     // LG        // Read only
      Order_Info.value.ui32val=Stepperlist[actidx]->blsteps;
    break;
    case INFO_ONESTEP:                     // LG        // Read only
      Order_Info.value.ui32val=Stepperlist[actidx]->onestep;
    break;
    case INFO_BLSTMAX:     // Server only  // LG
      Order_Info.value.ui32val=Stepperlist[actidx]->maxblsteps;
    break;
    case INFO_CALIMAX:     // Server only  // LG
      Order_Info.value.douval=Stepperlist[actidx]->maxcalibri;
    break;
    case INFO_SW_LIM1:                     // LG
      if(Stepperlist[actidx]->sw_lim1 < SW_LIM_OFF)
       Order_Info.value.ui32val=Stepperlist[actidx]->sw_lim1;
      else Order_Info.value.ui32val=0;
    break;
    case INFO_SW_LIM2:                     // LG
      if(Stepperlist[actidx]->sw_lim2 < SW_LIM_OFF)
       Order_Info.value.ui32val=Stepperlist[actidx]->sw_lim2;
      else Order_Info.value.ui32val=0;
    break;
    case INFO_NHS    :                     // LG
      Order_Info.value.triuinval[0]=Stepperlist[actidx]->nhs0;
      Order_Info.value.triuinval[1]=Stepperlist[actidx]->nhs1;
      Order_Info.value.triuinval[2]=Stepperlist[actidx]->nhs2;
    break;
    case INFO_ISD_HOBL:                    // LG
     Order_Info.value.ui32val=Stepperlist[actidx]->isd_hobl;
    break;
    case INFO_TENSTEPS1:                   // LG
     Order_Info.value.ui32val=Stepperlist[actidx]->tensteps1;
    break;
    case INFO_TENSTEPS2:                   // LG
     Order_Info.value.ui32val=Stepperlist[actidx]->tensteps2;
    break;
    case INFO_PIN_PUL:                     // LG
      Order_Info.value.ui8val=Stepperlist[actidx]->pin_pul;
    break;
    case INFO_PIN_DIR:                     // LG
      Order_Info.value.ui8val=Stepperlist[actidx]->pin_dir;
    break;
    case INFO_PIN_LIM1:                    // LG
      Order_Info.value.ui8val=Stepperlist[actidx]->pin_lim1;
    break;
    case INFO_PIN_LIM2:                    // LG
      Order_Info.value.ui8val=Stepperlist[actidx]->pin_lim2;
    break;
    case INFO_PIN_DIAG:                    // LG
      Order_Info.value.ui8val=Stepperlist[actidx]->pin_diag;
    break;
    case INFO_PIN_ENA:                     // LG
      Order_Info.value.ui8val=Stepperlist[actidx]->pin_ena;
    break;
    case INFO_PIN_MS1:                     // LG
      Order_Info.value.ui8val=Stepperlist[actidx]->pin_ms1;
    break;
    case INFO_PIN_MS2:                     // LG
      Order_Info.value.ui8val=Stepperlist[actidx]->pin_ms2;
    break;
    case INFO_PIN_MS3:                     // LG
      Order_Info.value.ui8val=Stepperlist[actidx]->pin_ms3;
    break;
    case INFO_SD_TYPE:                     // LG
      Order_Info.value.ui8val=Stepperlist[actidx]->driver_type;
    break;
    case INFO_NAME_LS1:    // Server only  // LG
      memcpy(Order_Info.value.nomstr,Stepperlist[actidx]->lsname[1],32);    
    break;
    case INFO_NAME_LS2:    // Server only  // LG
      memcpy(Order_Info.value.nomstr,Stepperlist[actidx]->lsname[2],32);    
    break;
    case INFO_NAME_SW1:    // Server only  // LG
      memcpy(Order_Info.value.nomstr,Stepperlist[actidx]->lsname[3],32);    
    break;
    case INFO_NAME_SW2:    // Server only  // LG
      memcpy(Order_Info.value.nomstr,Stepperlist[actidx]->lsname[4],32);    
    break;
    case INFO_NAME_UNI:    // Server only  // LG
      memcpy(Order_Info.value.nomstr,Stepperlist[actidx]->unitstr,32);    
    break;
    case INFO_MATURITY:    // Server only  // LG        // Read only
      Order_Info.value.ui8val=Stepperlist[actidx]->maturity;
    break;
    case INFO_UNIT_TYPE:   // Server only  // LG        // Read only
      Order_Info.value.ui8val=sv_Unitype[actype];
    break;
    default: // If this happens it is a program error (bug)
    break;
   }
  break;

  case ACT_BINARY:
   switch(Order_Info.infotype){
    case INFO_ON_VAL:                      // LG
      Order_Info.value.ui8val=Binarylist[actidx]->on_val;
    break;
    case INFO_OFF_VAL:                     // LG
      Order_Info.value.ui8val=Binarylist[actidx]->off_val;
    break;
    case INFO_PIN_BIN:                     // LG
      Order_Info.value.ui8val=Binarylist[actidx]->pin_bin;
    break;
    case INFO_MATURITY:    // Server only  // LG        // Read only
      Order_Info.value.ui8val=Binarylist[actidx]->maturity;
    break;
    default: // If this happens it is a program error (bug)
    break;
   }
  break;

  case ACT_MLEVEL:
   switch(Order_Info.infotype){
    case INFO_ML_TYPE:                     // LG
      Order_Info.value.ui8val=Mlevellist[actidx]->ml_type;
    break;
    case INFO_ML_PSC:                     // LG
      Order_Info.value.ui8val=Mlevellist[actidx]->ml_psc;
    break;
    case INFO_PIN_P0:                     // LG
      Order_Info.value.ui8val=Mlevellist[actidx]->ml_pin[0];
    break;
    case INFO_PIN_P1:                     // LG
      Order_Info.value.ui8val=Mlevellist[actidx]->ml_pin[1];
    break;
    case INFO_PIN_P2:                     // LG
      Order_Info.value.ui8val=Mlevellist[actidx]->ml_pin[2];
    break;
    case INFO_PIN_P3:                     // LG
      Order_Info.value.ui8val=Mlevellist[actidx]->ml_pin[3];
    break;
    case INFO_PIN_P4:                     // LG
      Order_Info.value.ui8val=Mlevellist[actidx]->ml_pin[4];
    break;
    case INFO_PIN_P5:                     // LG
      Order_Info.value.ui8val=Mlevellist[actidx]->ml_pin[5];
    break;
    case INFO_PIN_P6:                     // LG
      Order_Info.value.ui8val=Mlevellist[actidx]->ml_pin[6];
    break;
    case INFO_PIN_P7:                     // LG
      Order_Info.value.ui8val=Mlevellist[actidx]->ml_pin[7];
    break;
    case INFO_MATURITY:    // Server only  // LG        // Read only
      Order_Info.value.ui8val=Mlevellist[actidx]->maturity;
    break;
    case INFO_ML_LEVEL:                    // LG
      Order_Info.value.ui32val=Mlevellist[actidx]->last_level;
    break;
    case INFO_ML_TTABLE:                    // LG
      idx = Order_Info.value.triuinval[0];
      if(Mlevellist[actidx]->ml_tt == NULL){   // No table
          Order_Info.value.triuinval[0]=idx+1; // This signals the error
          return;
        }
      if(idx >= Mlevellist[actidx]->ntt){      // Index out of bounds
          Order_Info.value.triuinval[0]=idx+2; // This signals the error
          return;
        }
      Order_Info.value.triuinval[1]=Mlevellist[actidx]->ml_tt[idx].freq;
      Order_Info.value.triuinval[2]=(uint32_t)Mlevellist[actidx]->ml_tt[idx].duty;
    break;
    default: // If this happens it is a program error (bug)
    break;
   }
  break;

  case ACT_SERVO:
   switch(Order_Info.infotype){
    case INFO_SRV_FREQ:                    // LG
      Order_Info.value.ui8val=Servolist[actidx]->freq;
    break;
    case INFO_SRV_PINS:                    // LG
      Order_Info.value.ui8val=Servolist[actidx]->pin_srv;
    break;
    case INFO_SRV_PINP:                    // LG
      Order_Info.value.ui8val=Servolist[actidx]->pin_pos;
    break;
    case INFO_MATURITY:    // Server only  // LG        // Read only
      Order_Info.value.ui8val=Servolist[actidx]->maturity;
    break;
    case INFO_SRV_QPOS:                    // LG
      Order_Info.value.ui32val=Servolist[actidx]->destn_us;
    break;
    case INFO_SRV_MIN:                     // LG
      Order_Info.value.ui32val=Servolist[actidx]->srv_min;
    break;
    case INFO_SRV_MAX:                     // LG
      Order_Info.value.ui32val=Servolist[actidx]->srv_max;
    break;
    case INFO_SRV_ISD:                     // LG
      Order_Info.value.ui32val=Servolist[actidx]->isd_min;
    break;
    case INFO_SRV_THMAX:   // Server only  // LG
      Order_Info.value.douval=Servolist[actidx]->theta_max;
    break;
    case INFO_SRV_THMIN:   // Server only  // LG
      Order_Info.value.douval=Servolist[actidx]->theta_min;
    break;
    case INFO_SRV_CALIB:   // Server only  // LG        // Read only
      Order_Info.value.douval=Servolist[actidx]->calibri;
    break;
    default: // If this happens it is a program error (bug)
    break;
   }
  break;

  case ACT_SENSOR:
   switch(Order_Info.infotype){
    case INFO_SEN_PIN:                    // LG
      Order_Info.value.ui8val=Sensorlist[actidx]->pin_sense;
    break;
    case INFO_MATURITY:    // Server only  // LG        // Read only
      Order_Info.value.ui8val=Sensorlist[actidx]->maturity;
    break;
    case INFO_SEN_TYPE:                     // LG
      Order_Info.value.ui8val=Sensorlist[actidx]->type;
    break;
    default: // If this happens it is a program error (bug)
    break;
   }
  break;

  default:
  break;
 } 

 return;
}

size_t act_client_blobsz_from_infotype(uint8_t actype)
// Calculate the size of a server blob (in bytes) to hold a transmission of the
// data from an Order_Info form received from Arduino and destined to be sent
// to the client and store the number of bytes of the transmission to the client
// (this will usually be the same as the size of the Order_Info.value member
// data except for cases where int32_t values need to be converted to doubles
// for the client such as with coordinates and distances).
{
 size_t retval=0;

 // Add 1 byte for the infotype:
 retval++;

 switch(actype){
  case ACT_STEPPER:
   switch(Order_Info.infotype){
    // These convey 1 x uint8_t
    case INFO_ENA_STATE:                   // Ask Ardu
    case INFO_MMODE:       // Ardu only    // Ask Ardu
    case INFO_TURBO_BST:   // Ardu only    // Ask Ardu
    case INFO_SG_BUFFPC:   // Ardu only    // Ask Ardu
    case INFO_SG_THRESH:   // Ardu only    // Ask Ardu
    case INFO_HOMED_TO:                    // Ask Ardu  // Read only
    case INFO_DIRN1:                       // LG
    case INFO_DIRN2:                       // LG
    case INFO_DIRNC:                       // Ask Ardu
    case INFO_DIRNP   :    // Ardu only    // Ask Ardu  // Read only
    case INFO_PULWIDTH:                    // LG
    case INFO_PIN_PUL:                     // LG
    case INFO_PIN_DIR:                     // LG
    case INFO_PIN_LIM1:                    // LG
    case INFO_PIN_LIM2:                    // LG
    case INFO_PIN_DIAG:                    // LG
    case INFO_PIN_ENA:                     // LG
    case INFO_PIN_MS1:                     // LG
    case INFO_PIN_MS2:                     // LG
    case INFO_PIN_MS3:                     // LG
    case INFO_SD_TYPE:                     // LG
    case INFO_MATURITY:    // Server only  // LG        // Read only
    case INFO_UNIT_TYPE:   // Server only  // LG        // Read only
    case INFO_LIMSTATE:                    // Ask Ardu  // Read only
     retval+=1;
     Order_Info.infotype=INVT_UI8;
    break;
    // These convey 1 x uint32_t
    case INFO_ONESTEP:                     // LG        // Read only
    case INFO_MSDENOM:                     // LG
    case INFO_FASTMSD:                     // LG
    case INFO_MSDEMAX:     // Server only  // LG
    case INFO_BLSTEPS:                     // LG        // Read only
    case INFO_TMCSGVAL :   // Ardu only    // Ask Ardu  // Read only
    case INFO_TMCSTATUS:   // Ardu only    // Ask Ardu  // Read only
    case INFO_TMCCHCONF:   // Ardu only    // Ask Ardu  // Read only
    case INFO_TMCMSCNT :   // Ardu only    // Ask Ardu  // Read only
    case INFO_DURALAM:     // Ardu only    // Ask Ardu  // Read only
    case INFO_STEPLAM:     // Ardu only    // Ask Ardu  // Read only
    case INFO_BLSTMAX:     // Server only  // LG
    case INFO_ISD_HOBL:                    // LG
    case INFO_TENSTEPS1:                   // LG
    case INFO_TENSTEPS2:                   // LG
    case INFO_STEPCOUNT:                   // Ask Ardu
    case INFO_SW_LIM1:                     // LG
    case INFO_SW_LIM2:                     // LG
     retval+=4;
     Order_Info.infotype=INVT_UI32;
    break;
   // These convey 3 x uint32_t (3x4=12 bytes)
    case INFO_NHS    :                     // LG
    case INFO_XYZCOORDS:                   // Ask Ardu
     retval+=12;
     Order_Info.infotype=INVT_3UIN;
    break;
   // These convey 3 x double (3x8=24 bytes)
//     retval+=24;
//     Order_Info.infotype=INVT_3DOU;
    break;
   // These convey 1 x double (8 bytes)
    case INFO_CALIMAX:     // Server only  // LG
     retval+=8;
     Order_Info.infotype=INVT_DOU8;
    break;
   // These convey a 32 byte string (includes '\0')
    case INFO_NAME_LS1:    // Server only  // LG
    case INFO_NAME_LS2:    // Server only  // LG
    case INFO_NAME_SW1:    // Server only  // LG
    case INFO_NAME_SW2:    // Server only  // LG
    case INFO_NAME_UNI:    // Server only  // LG
     retval+=32;
     Order_Info.infotype=INVT_STR;
    break;
    default: // This would be a program error (bug)
    break;
   }
  
  break;
  case ACT_BINARY:
   switch(Order_Info.infotype){
    // These convey 1 x uint8_t
    case INFO_BIN_STATE:                   // Ask Ardu
    case INFO_ON_VAL:                      // LG
    case INFO_OFF_VAL:                     // LG
    case INFO_PIN_BIN:                     // LG
    case INFO_MATURITY:    // Server only  // LG        // Read only
     retval+=1;
     Order_Info.infotype=INVT_UI8;
    break;
    default: // This would be a program error (bug)
    break;
   }
  
  break;
  case ACT_MLEVEL:
   switch(Order_Info.infotype){
    // These convey 1 x uint8_t
    case INFO_ML_STATE:    // Ardu only    // Ask Ardu  // Read only
    case INFO_ML_TIMER:    // Ardu only    // Ask Ardu  // Read only
    case INFO_ML_TYPE:                     // LG
    case INFO_ML_PSC:                      // LG
    case INFO_PIN_P0:                      // LG
    case INFO_PIN_P1:                      // LG
    case INFO_PIN_P2:                      // LG
    case INFO_PIN_P3:                      // LG
    case INFO_PIN_P4:                      // LG
    case INFO_PIN_P5:                      // LG
    case INFO_PIN_P6:                      // LG
    case INFO_PIN_P7:                      // LG
    case INFO_MATURITY:    // Server only  // LG        // Read only
     retval+=1;
     Order_Info.infotype=INVT_UI8;
    break;
    // These convey 1 x uint32_t
    case INFO_ML_LEVEL:                    // LG
    case INFO_ML_FREQ:     // Ardu only    // Ask Ardu  // Read only
     retval+=4;
     Order_Info.infotype=INVT_UI32;
    break;
    // These convey 3 x uint32_t
    case INFO_ML_TTABLE:   // Server only  // LG
     retval+=12;
     Order_Info.infotype=INVT_3UIN;
    break;
    default: // This would be a program error (bug)
    break;
   }
  break;

  case ACT_SERVO:
   switch(Order_Info.infotype){
    // These convey 1 x uint8_t
    case INFO_SRV_STATE:   // Ardu only    // Ask Ardu  // Read only
    case INFO_SRV_TIMER:   // Ardu only    // Ask Ardu  // Read only
    case INFO_SRV_FREQ:                    // LG
    case INFO_SRV_PINS:                    // LG
    case INFO_SRV_PINP:                    // LG
     retval+=1;
     Order_Info.infotype=INVT_UI8;
    break;
    // These convey 1 x uint32_t
    case INFO_SRV_AFB:     // Ardu only    // Ask Ardu  // Read only
    case INFO_SRV_SPOS:    // Ardu only    // Ask Ardu
    case INFO_SRV_QPOS:                    // LG
    case INFO_SRV_MIN:                     // LG
    case INFO_SRV_MAX:                     // LG
    case INFO_SRV_ISD:                     // LG
     retval+=4;
     Order_Info.infotype=INVT_UI32;
    break;
   // These convey 1 x double (8 bytes)
    case INFO_SRV_THMAX:   // Server only  // LG
    case INFO_SRV_THMIN:   // Server only  // LG
    case INFO_SRV_CALIB:   // Server only  // LG        // Read only
     retval+=8;
     Order_Info.infotype=INVT_DOU8;
    break;
    default: // This would be a program error (bug)
    break;
   }
  break;

  case ACT_SENSOR:
   switch(Order_Info.infotype){
    // These convey 1 x uint8_t
    case INFO_SEN_TYPE:                    // LG
    case INFO_SEN_PIN:                     // LG
     retval+=1;
     Order_Info.infotype=INVT_UI8;
    break;
    // These convey 1 x uint32_t
    case INFO_SEN_VAL:     // Ardu only    // Ask Ardu  // Read only
     retval+=4;
     Order_Info.infotype=INVT_UI32;
    break;
    default: // This would be a program error (bug)
    break;
   }
  break;

  default:
  break;
 }

 return retval;
}

uint8_t act_set_userinput_datatype(uint8_t actype, uint8_t infotype)
// Returns the type of data conveyed by a PCS script writer as input to set an
// info item. This may not be the same native data type as that info item. For
// example, for coordinates (software limits, stepcount) the user will input a
// double because they may be using calibrated units (e.g. 3.587 microns) but
// the actual motor struct members are in uint32_t. That conversion (from double
// to int) will be done later but for now we need to know what data type to
// expect from the user so we can use the correct sscanf specifier to receive
// it. The data types here are conveyed in the form of one of the INVT_ #defined
// constants.
{
 uint8_t retval=INVT_ERR;

 switch(actype){
  case ACT_STEPPER:
   switch(infotype){
    // The user conveys 1 x uint8_t
    case INFO_ENA_STATE:                   // Ask Ardu
    case INFO_MMODE:       // Ardu only    // Ask Ardu
    case INFO_TURBO_BST:   // Ardu only    // Ask Ardu
    case INFO_SG_BUFFPC:   // Ardu only    // Ask Ardu
    case INFO_SG_THRESH:   // Ardu only    // Ask Ardu
    case INFO_HOMED_TO:                    // Ask Ardu  // Read only
    case INFO_DIRN1:                       // LG
    case INFO_DIRN2:                       // LG
    case INFO_DIRNC:                       // Ask Ardu
    case INFO_DIRNP   :    // Ardu only    // Ask Ardu  // Read only
    case INFO_PULWIDTH:                    // LG
    case INFO_PIN_PUL:                     // LG
    case INFO_PIN_DIR:                     // LG
    case INFO_PIN_LIM1:                    // LG
    case INFO_PIN_LIM2:                    // LG
    case INFO_PIN_DIAG:                    // LG
    case INFO_PIN_ENA:                     // LG
    case INFO_PIN_MS1:                     // LG
    case INFO_PIN_MS2:                     // LG
    case INFO_PIN_MS3:                     // LG
    case INFO_SD_TYPE:                     // LG
    case INFO_MATURITY:    // Server only  // LG        // Read only
    case INFO_UNIT_TYPE:   // Server only  // LG        // Read only
    case INFO_LIMSTATE:                    // Ask Ardu  // Read only
     retval=INVT_UI8;
    break;
    // The user conveys 1 x uint32_t
    case INFO_ONESTEP:                     // LG        // Read only
    case INFO_MSDENOM:                     // LG
    case INFO_FASTMSD:                     // LG
    case INFO_MSDEMAX:     // Server only  // LG
    case INFO_BLSTEPS:                     // LG        // Read only
    case INFO_TMCSGVAL :   // Ardu only    // Ask Ardu  // Read only
    case INFO_TMCSTATUS:   // Ardu only    // Ask Ardu  // Read only
    case INFO_TMCCHCONF:   // Ardu only    // Ask Ardu  // Read only
    case INFO_TMCMSCNT :   // Ardu only    // Ask Ardu  // Read only
    case INFO_DURALAM:     // Ardu only    // Ask Ardu  // Read only
    case INFO_STEPLAM:     // Ardu only    // Ask Ardu  // Read only
    case INFO_BLSTMAX:     // Server only  // LG
    case INFO_ISD_HOBL:                    // LG
    case INFO_TENSTEPS1:                   // LG
    case INFO_TENSTEPS2:                   // LG
     retval=INVT_UI32;
    break;
   // The user conveys 3 x uint32_t
    case INFO_NHS    :                     // LG
     retval=INVT_3UIN;
    break;
   // The user conveys 3 x 8-byte doubles
    case INFO_XYZCOORDS:                   // Ask Ardu
     retval=INVT_3DOU;
    break;
   // The user conveys 1 x double (8 bytes)
    case INFO_STEPCOUNT:                   // Ask Ardu
    case INFO_SW_LIM1:                     // LG
    case INFO_SW_LIM2:                     // LG
    case INFO_CALIMAX:     // Server only  // LG
     retval=INVT_DOU8;
    break;
   // The user conveys a 32 byte string (includes '\0')
    case INFO_NAME_LS1:    // Server only  // LG
    case INFO_NAME_LS2:    // Server only  // LG
    case INFO_NAME_SW1:    // Server only  // LG
    case INFO_NAME_SW2:    // Server only  // LG
    case INFO_NAME_UNI:    // Server only  // LG
     retval=INVT_STR;
    break;
    default: // This would be a program error (bug)
    break;
   }
  
  break;
  case ACT_BINARY:
   switch(infotype){
    // The user conveys 1 x uint8_t
    case INFO_BIN_STATE:                   // Ask Ardu
    case INFO_ON_VAL:                      // LG
    case INFO_OFF_VAL:                     // LG
    case INFO_PIN_BIN:                     // LG
    case INFO_MATURITY:    // Server only  // LG        // Read only
     retval=INVT_UI8;
    break;
    default: // This would be a program error (bug)
    break;
   }
  
  break;
  case ACT_MLEVEL:
   switch(infotype){
    // These convey 1 x uint8_t
    case INFO_ML_STATE:    // Ardu only    // Ask Ardu  // Read only
    case INFO_ML_TIMER:    // Ardu only    // Ask Ardu  // Read only
    case INFO_ML_TYPE:                     // LG
    case INFO_ML_PSC:                      // LG
    case INFO_PIN_P0:                      // LG
    case INFO_PIN_P1:                      // LG
    case INFO_PIN_P2:                      // LG
    case INFO_PIN_P3:                      // LG
    case INFO_PIN_P4:                      // LG
    case INFO_PIN_P5:                      // LG
    case INFO_PIN_P6:                      // LG
    case INFO_PIN_P7:                      // LG
    case INFO_MATURITY:    // Server only  // LG        // Read only
     retval=INVT_UI8;
    break;
    // These convey 1 x uint32_t
    case INFO_ML_LEVEL:                    // LG
    case INFO_ML_FREQ:     // Ardu only    // Ask Ardu  // Read only
     retval=INVT_UI32;
    break;
    // These convey 3 x uint32_t
    case INFO_ML_TTABLE:   // Server only  // LG
     retval=INVT_3UIN;
    break;
    default: // This would be a program error (bug)
    break;
   }
  break;

  case ACT_SERVO:
   switch(infotype){
    // These convey 1 x uint8_t
    case INFO_SRV_STATE:   // Ardu only    // Ask Ardu  // Read only
    case INFO_SRV_TIMER:   // Ardu only    // Ask Ardu  // Read only
    case INFO_SRV_FREQ:                    // LG
    case INFO_SRV_PINS:                    // LG
    case INFO_SRV_PINP:                    // LG
     retval=INVT_UI8;
    break;
    // These convey 1 x uint32_t
    case INFO_SRV_AFB:     // Ardu only    // Ask Ardu  // Read only
    case INFO_SRV_MIN:                     // LG
    case INFO_SRV_MAX:                     // LG
    case INFO_SRV_ISD:                     // LG
     retval=INVT_UI32;
    break;
   // These convey 1 x double (8 bytes)
    case INFO_SRV_SPOS:    // Ardu only    // Ask Ardu
    case INFO_SRV_QPOS:                    // LG
    case INFO_SRV_THMAX:   // Server only  // LG
    case INFO_SRV_THMIN:   // Server only  // LG
    case INFO_SRV_CALIB:   // Server only  // LG        // Read only
     retval=INVT_DOU8;
    break;
    default: // This would be a program error (bug)
    break;
   }
  break;

  case ACT_SENSOR:
   switch(infotype){
    // These convey 1 x uint8_t
    case INFO_SEN_TYPE:                    // LG
    case INFO_SEN_PIN:                     // LG
     retval=INVT_UI8;
    break;
    // These convey 1 x uint32_t
    case INFO_SEN_VAL:     // Ardu only    // Ask Ardu  // Read only
     retval=INVT_UI32;
    break;
    default: // This would be a program error (bug)
    break;
   }
  break;

  default:
  break;
 }

 return retval;

}

uint8_t tt_new_row(uint8_t mlidx, uint32_t freq, uint16_t duty,char *errmsg)
// Adds a new translation table node (row) to the ml_tt of an M-Level actuator.
// Returns 0 on success
// Returns 1 on failure with a message printed into errmsg
{
 MLevel_TTable *newmem;
 size_t newsize;
 uint32_t idx; 

 // Try to get memory for the new node
 newsize=Mlevellist[mlidx]->sztt+sizeof(MLevel_TTable);
 newmem=realloc(Mlevellist[mlidx]->ml_tt,newsize);
 if(newmem==NULL){
    snprintf(errmsg,ERRMSG_LEN,"Failed to get memory for new translation table row.");
    return 1;
  }
 Mlevellist[mlidx]->ml_tt=newmem;
 Mlevellist[mlidx]->sztt=newsize;
 idx=Mlevellist[mlidx]->ntt;
 Mlevellist[mlidx]->ntt++;
 
 // Now set the values assigned to the node
 Mlevellist[mlidx]->ml_tt[idx].freq=freq;
 Mlevellist[mlidx]->ml_tt[idx].duty=duty;
 return 0;
}


static int ml_duty_is_valid(uint32_t d)
// ROBOVOX iteration 2: is d a valid PWM M-Level duty?
// 0..ML_DUTY_MAX: a plain duty in per mille;
// ML_DUTY_MAX+1..ML_DUTY_CODE_MAX: an effect code 1000*E + P,
// refused when P = 0 for E = 1, 2, 3.
{
  uint32_t e, p;
  if (d <= ML_DUTY_MAX)      return 1;  // plain duty 0..1000
  if (d  > ML_DUTY_CODE_MAX) return 0;  // above 4999
  e = d  / ML_FX_BASE;                  // 1..4
  p = d  % ML_FX_BASE;                  // 0..999
  if (e == ML_FX_GLIDE)      return 1;  // 4000..4999, 4000 = off
  return p != 0;                        // 2000 and 3000 refused
}

uint8_t act_set_to_local(uint8_t actype, uint8_t actidx, char *errmsg)
// Replace the current value of the local actuator struct with the 'value' part
// of the global Order_Info struct. This function is for those members of the
// actuator struct that do not exist on the Arduino. See act_sync_to_local for
// setting those members that are common to both Arduino and the local struct.
// Returns 0 on success
// Returns 1 on failure with the reason printed into errmsg (which is of size
//           ERRMSG_LEN).
{
 uint8_t retval = 0;
 uint16_t duty;
 uint32_t idx,freq;

 switch(actype){
  case ACT_STEPPER:
   switch(Order_Info.infotype){
    case INFO_MSDEMAX:     // Server only  // LG
      Stepperlist[actidx]->maxmsdenom=Order_Info.value.ui32val;
    break;
    case INFO_BLSTMAX:     // Server only  // LG
      Stepperlist[actidx]->maxblsteps=Order_Info.value.ui32val;
    break;
    case INFO_CALIMAX:     // Server only  // LG
      Stepperlist[actidx]->maxcalibri=Order_Info.value.douval;
    break;
    case INFO_TENSTEPS1:                   // LG
      Stepperlist[actidx]->tensteps1=Order_Info.value.ui32val;
    break;
    case INFO_TENSTEPS2:                   // LG
      Stepperlist[actidx]->tensteps2=Order_Info.value.ui32val;
    break;
    case INFO_NAME_LS1:    // Server only  // LG
      sprintf(Stepperlist[actidx]->lsname[1],"%.31s",Order_Info.value.nomstr);
    break;
    case INFO_NAME_LS2:    // Server only  // LG
      sprintf(Stepperlist[actidx]->lsname[2],"%.31s",Order_Info.value.nomstr);
    break;
    case INFO_NAME_SW1:    // Server only  // LG
      sprintf(Stepperlist[actidx]->lsname[3],"%.31s",Order_Info.value.nomstr);
    break;
    case INFO_NAME_SW2:    // Server only  // LG
      sprintf(Stepperlist[actidx]->lsname[4],"%.31s",Order_Info.value.nomstr);
    break;
    case INFO_NAME_UNI:    // Server only  // LG
      sprintf(Stepperlist[actidx]->unitstr,"%.31s",Order_Info.value.nomstr);
    break;
    default:
    break;
   }
  break;

  case ACT_BINARY:
   // Only actuator name and maturity are not stored on the Arduino for Binary
   // actuators and those are not dealt with in this function so there is
   // nothing to do.
  break;

  case ACT_MLEVEL:
   // Actuator name and maturity are dealt with elsewhere. ml_type, ml_pin and
   // last_level will be synced post BOB command success. So all that is left
   // for purely local management is building the translation table:
   switch(Order_Info.infotype){
    case INFO_ML_TTABLE:     // Server only  // LG
      idx = Order_Info.value.triuinval[0]; // Target row

      // Test the limits of the proposed values:
      if(Order_Info.value.triuinval[1]>8000000){
         retval = 1;
         snprintf(errmsg,ERRMSG_LEN,"The frequency value must be <= 8000000.");
         break;
        } else freq = Order_Info.value.triuinval[1];
      if(!ml_duty_is_valid(Order_Info.value.triuinval[2])){
         retval = 1;
         snprintf(errmsg,ERRMSG_LEN,"The duty cycle value must be <= 4999 and exclude 2000 and 3000.");
         break;
        } else duty = (uint16_t)Order_Info.value.triuinval[2];

      // User can't 'jump' or delete rows - they can either edit an existing row 
      // or append a new entry immediately after the last extant row. So check
      // for these two cases and reject as an error anything else:
      if(idx<Mlevellist[actidx]->ntt){
      // 1. The row exists - it is a replace request:
        Mlevellist[actidx]->ml_tt[idx].freq = freq;
        Mlevellist[actidx]->ml_tt[idx].duty = duty;

       } else if(idx==Mlevellist[actidx]->ntt){
      // 2. This is an append request:
       if( tt_new_row(actidx, freq, duty, errmsg) ) retval = 1;
       } else {
      // 3. Anything else is fobridden:
        retval = 1;
        snprintf(errmsg,ERRMSG_LEN,"For a translation table you may only append"
                                    " a new entry or modify an existing entry.");
       }
    break;
    default:
    break;
   }  
  break; 

  case ACT_SERVO:
   switch(Order_Info.infotype){
    case INFO_SRV_THMAX:   // Server only  // LG
      Servolist[actidx]->theta_max=Order_Info.value.douval;
      // Update calibri:
      Servolist[actidx]->calibri=servo_calibri(actidx);
    break;
    case INFO_SRV_THMIN:   // Server only  // LG
      Servolist[actidx]->theta_min=Order_Info.value.douval;
      // Update calibri:
      Servolist[actidx]->calibri=servo_calibri(actidx);
    break;
    default:
    break;
   }
  break;

  case ACT_SENSOR:
   // Actuator name and maturity are dealt with elsewhere. type and pin will
   // will be synced post BOB command success. So there is nothing to do here.
  break;

  default:
  break;
 } 

 return retval;
}

void act_sync_to_local(uint8_t actype, uint8_t actidx, uint8_t fromset)
// Replace the current value of the local actuator struct with the 'value' part
// of the global Order_Info struct for those members of the actuator struct that
// exist on both Arduino and the server. See act_set_to_local for setting those
// members that are exclusively held on the server.
// This function is called as part of the act_set and act_get operations. If
// called as part of an act_set then use fromset==1, else make it 0.
{
 uint8_t pinsupdated = 0;

 switch(actype){
  case ACT_STEPPER:
   // All entries labelled 'LG' up to and including INFO_SD_TYPE might have
   // been altered on the Arduino by an act_set command so need to be updated
   // on the local copy. However, an act_get command could not have changed them
   // so only do them if fromset==1.
   switch(Order_Info.infotype){
    case INFO_DIRN1:                       // LG
      if(fromset) Stepperlist[actidx]->dirn1=Order_Info.value.ui8val;
    break;
    case INFO_DIRN2:                       // LG
      if(fromset) Stepperlist[actidx]->dirn2=Order_Info.value.ui8val;
    break;
    case INFO_DIRNC:                       // Ask Ardu
      Stepperlist[actidx]->currdirn=Order_Info.value.ui8val;
    break;
    case INFO_MSDENOM:                     // LG
      if(fromset) Stepperlist[actidx]->msdenom=Order_Info.value.ui32val;
    break;
    case INFO_FASTMSD:                     // LG
      if(fromset) Stepperlist[actidx]->fastmsd=Order_Info.value.ui32val;
    break;
    case INFO_PULWIDTH:                    // LG
      if(fromset) Stepperlist[actidx]->pulwidth=Order_Info.value.ui8val;
    break;
    case INFO_SW_LIM1:                     // LG
      if(fromset) Stepperlist[actidx]->sw_lim1=Order_Info.value.ui32val;
    break;
    case INFO_SW_LIM2:                     // LG
      if(fromset) Stepperlist[actidx]->sw_lim2=Order_Info.value.ui32val;
    break;
    case INFO_NHS    :                     // LG
      if(fromset){
        Stepperlist[actidx]->nhs0=Order_Info.value.triuinval[0];
        Stepperlist[actidx]->nhs1=Order_Info.value.triuinval[1];
        Stepperlist[actidx]->nhs2=Order_Info.value.triuinval[2];
       }
    break;
    case INFO_ISD_HOBL:                    // LG
      if(fromset) Stepperlist[actidx]->isd_hobl=Order_Info.value.ui32val;
    break;
    case INFO_PIN_PUL:                     // LG
      if(fromset) Stepperlist[actidx]->pin_pul=Order_Info.value.ui8val;
    break;
    case INFO_PIN_DIR:                     // LG
      if(fromset) Stepperlist[actidx]->pin_dir=Order_Info.value.ui8val;
    break;
    case INFO_PIN_LIM1:                    // LG
      if(fromset) Stepperlist[actidx]->pin_lim1=Order_Info.value.ui8val;
    break;
    case INFO_PIN_LIM2:                    // LG
      if(fromset) Stepperlist[actidx]->pin_lim2=Order_Info.value.ui8val;
    break;
    case INFO_PIN_DIAG:                    // LG
      if(fromset) Stepperlist[actidx]->pin_diag=Order_Info.value.ui8val;
    break;
    case INFO_PIN_ENA:                     // LG
      if(fromset) Stepperlist[actidx]->pin_ena=Order_Info.value.ui8val;
    break;
    case INFO_PIN_MS1:                     // LG
      if(fromset) Stepperlist[actidx]->pin_ms1=Order_Info.value.ui8val;
    break;
    case INFO_PIN_MS2:                     // LG
      if(fromset) Stepperlist[actidx]->pin_ms2=Order_Info.value.ui8val;
    break;
    case INFO_PIN_MS3:                     // LG
      if(fromset) Stepperlist[actidx]->pin_ms3=Order_Info.value.ui8val;
    break;
    case INFO_SD_TYPE:                     // LG
      if(fromset) Stepperlist[actidx]->driver_type=Order_Info.value.ui8val;
    break;
    // The next two may have been altered by motion regardless of whether we
    // just did an act_set or act_get and also might have been directly changed
    // on the BOB by an act_set. So sync them regardless of whose calling:
    case INFO_STEPCOUNT:                   // Ask Ardu
      Stepperlist[actidx]->stepcount=Order_Info.value.ui32val;
    break;
    case INFO_XYZCOORDS:                   // Ask Ardu
      Stepperlist[ACT_ID_STAGE_X]->stepcount=Order_Info.value.triuinval[0];
      Stepperlist[ACT_ID_STAGE_Y]->stepcount=Order_Info.value.triuinval[1];
      Stepperlist[ACT_ID_STAGE_Z]->stepcount=Order_Info.value.triuinval[2];
    break;
    case INFO_MSDEMAX:      // Server only  // LG
    case INFO_BLSTMAX:      // Server only  // LG
    case INFO_CALIMAX:      // Server only  // LG
    case INFO_NAME_LS1:     // Server only  // LG
    case INFO_NAME_LS2:     // Server only  // LG
    case INFO_NAME_SW1:     // Server only  // LG
    case INFO_NAME_SW2:     // Server only  // LG
    case INFO_NAME_UNI:     // Server only  // LG
     // If calling from act_get, the above would not result in a call to Arduino
     // so they could not have changed - just skip them.
     // If calling from act_set, the above would have been set already and there
     // would be no call to Arduino. For the 'MAX' items we will recalculate the
     // dependent value (blsteps) later and update it on the Arduino in separate
     // code. So here we just move on. Likewise the TENSTEPS items will be dealt
     // with by separate code that converts the values to steps in the current
     // msdenom before sending to Arduino.
    break;
    case INFO_BLSTEPS :                     // LG        // Read only
    case INFO_ONESTEP :                     // LG        // Read only
    case INFO_TENSTEPS1:                    // LG
    case INFO_TENSTEPS2:                    // LG
     // With act_get these could not have changed.
     // With act_set these might be recalculated if a new MSDENOM, MSDEMAX or
     // (for BLSTEPS) BLSTMAX was set but we do the calcs and updates in
     // separate code - so just skip for now.
    break;
    case INFO_MATURITY:     // Server only  // LG        // Read only
     // Maturity might get re-calculated after an act_set (not act_get) but then
     // it will be updated elsewhere (where the re-calculation is done) - skip
    break;
    case INFO_UNIT_TYPE:    // Server only  // LG        // Read only
     // This will not change with an act_set (only the unit_type command can
     // alter it). Thus there is nothing to sync after act_set.
    break;
    case INFO_HOMED_TO:                     // Ask Ardu  // Read only
     // If an act_set alters limit switch parameters then the homed_to value
     // should be invalidated but that is not the role of this value sync 
     // function. An INFO_HOMED_TO case in this switch means that an act_get
     // referred to this parameter. act_set cannot refer to this because it is
     // read only. However, act_get would have got this from the Arduino so it
     // needs updating locally:
     if(!fromset) Stepperlist[actidx]->homed_to=Order_Info.value.ui8val;
    break;
    case INFO_LIMSTATE:                     // Ask Ardu  // Read only
     // As for homed_to, update it if the call is from act_get:
      if(!fromset) Stepperlist[actidx]->limstate=Order_Info.value.ui8val;
    break;
    case INFO_DIRNP   :    // Ardu only      // Ask Ardu  // Read only
    case INFO_TMCSGVAL :   // Ardu only      // Ask Ardu  // Read only
    case INFO_TMCSTATUS:   // Ardu only      // Ask Ardu  // Read only
    case INFO_TMCCHCONF:   // Ardu only      // Ask Ardu  // Read only
    case INFO_TMCMSCNT :   // Ardu only      // Ask Ardu  // Read only
    case INFO_DURALAM :    // Ardu only      // Ask Ardu  // Read only
    case INFO_STEPLAM :    // Ardu only      // Ask Ardu  // Read only
    case INFO_ENA_STATE:   // Ardu only      // Ask Ardu
    case INFO_MMODE:       // Ardu only      // Ask Ardu
    case INFO_TURBO_BST:   // Ardu only      // Ask Ardu
    case INFO_SG_BUFFPC:   // Ardu only      // Ask Ardu
    case INFO_SG_THRESH:   // Ardu only      // Ask Ardu
     // There is no local copy of the above items.
     // These are only on the Arduino struct. No copy of the TMC driver
     // values are stored anywhere (they are polled from the driver when needed)
     // So just skip these.
    break;
    default:
    break;
   }
  break;

  case ACT_BINARY:
   // All entries labelled 'LG' up to and including INFO_PIN_BIN might have
   // been altered on the Arduino by an act_set command so need to be updated
   // on the local copy. However, an act_get command could not have changed them
   // so only do them if fromset==1.
   switch(Order_Info.infotype){
    case INFO_ON_VAL:                       // LG
      if(fromset) Binarylist[actidx]->on_val=Order_Info.value.ui8val;
    break;
    case INFO_OFF_VAL:                      // LG
      if(fromset) Binarylist[actidx]->off_val=Order_Info.value.ui8val;
    break;
    case INFO_PIN_BIN:                      // LG
      if(fromset) Binarylist[actidx]->pin_bin=Order_Info.value.ui8val;
    break;
    case INFO_MATURITY:     // Server only  // LG        // Read only
     // Maturity might get re-calculated after an act_set (not act_get) but then
     // it will be updated elsewhere (where the re-calculation is done) - skip
    break;
    case INFO_BIN_STATE:   // Ardu only     // Ask Ardu
     // There is no local copy of bin_state, so just skip it.
    break;
    default:
    break;
   }

  break;
  case ACT_MLEVEL:
   switch(Order_Info.infotype){
    case INFO_ML_TYPE:                     // LG
      if(fromset) Mlevellist[actidx]->ml_type=Order_Info.value.ui8val;
    break;
    case INFO_ML_PSC:                      // LG
      if(fromset) Mlevellist[actidx]->ml_psc=Order_Info.value.ui8val;
    break;
    case INFO_PIN_P0:                      // LG
      if(fromset){
        Mlevellist[actidx]->ml_pin[0]=Order_Info.value.ui8val;
        pinsupdated = 1;
       }
    break;
    case INFO_PIN_P1:                      // LG
      if(fromset){
         Mlevellist[actidx]->ml_pin[1]=Order_Info.value.ui8val;
        pinsupdated = 1;
       }
    break;
    case INFO_PIN_P2:                      // LG
      if(fromset){
         Mlevellist[actidx]->ml_pin[2]=Order_Info.value.ui8val;
        pinsupdated = 1;
       }
    break;
    case INFO_PIN_P3:                      // LG
      if(fromset){
         Mlevellist[actidx]->ml_pin[3]=Order_Info.value.ui8val;
        pinsupdated = 1;
       }
    break;
    case INFO_PIN_P4:                      // LG
      if(fromset){
         Mlevellist[actidx]->ml_pin[4]=Order_Info.value.ui8val;
        pinsupdated = 1;
       }
    break;
    case INFO_PIN_P5:                      // LG
      if(fromset){
         Mlevellist[actidx]->ml_pin[5]=Order_Info.value.ui8val;
        pinsupdated = 1;
       }
    break;
    case INFO_PIN_P6:                      // LG
      if(fromset){
         Mlevellist[actidx]->ml_pin[6]=Order_Info.value.ui8val;
        pinsupdated = 1;
       }
    break;
    case INFO_PIN_P7:                      // LG
      if(fromset){
         Mlevellist[actidx]->ml_pin[7]=Order_Info.value.ui8val;
        pinsupdated = 1;
       }
    break;
    case INFO_ML_LEVEL:                    // LG
      if(fromset){
         Mlevellist[actidx]->last_level= LK_mlevel;
         Mlevellist[actidx]->last.freq = LK_freq;
         Mlevellist[actidx]->last.duty = LK_duty;
        }
    break;
    case INFO_ML_STATE:    // Ardu only    // Ask Ardu  // Read only
    case INFO_ML_TIMER:    // Ardu only    // Ask Ardu  // Read only
    case INFO_ML_TTABLE:   // Server only  // LG
    case INFO_ML_FREQ:     // Ardu only    // Ask Ardu  // Read only
     // The translation table is 'synced' already because it is created here.
     // ML_STATE and ML_TIMER are also not stored on the server so no syncing
     // is needed - skip. There is no local copy of ML_FREQ so skip.
    break;
    case INFO_MATURITY:     // Server only  // LG        // Read only
     // Maturity might get re-calculated after an act_set (not act_get) but then
     // it will be updated elsewhere (where the re-calculation is done) - skip
    break;
    default: // This would be a program error (bug)
    break;
   }
  if(pinsupdated){
     if (Mlevellist[actidx]->ml_type == MLT_PARALLEL) {
         uint8_t count = 0;
         for (uint8_t i = 0; i < 8; i++) {
             if (Mlevellist[actidx]->ml_pin[i] != PIN_IS_UNSET) {
                 count++;
             }
         }
         Mlevellist[actidx]->npins     = count;
         // Fast bit-shift calculates the limit: (1 << 2) - 1 = 3
         Mlevellist[actidx]->max_level = (1UL << count) - 1UL;

        Mlevellist[actidx]->pingaps = mlevel_pins_have_gaps(actidx);
     } 
    }
  break;

  case ACT_SERVO:
   switch(Order_Info.infotype){
    case INFO_SRV_MIN:                       // LG
      if(fromset){
         Servolist[actidx]->srv_min=Order_Info.value.ui32val;
         // Update calibri:
         Servolist[actidx]->calibri=servo_calibri(actidx);
       }
    break;
    case INFO_SRV_MAX:                       // LG
      if(fromset){
         Servolist[actidx]->srv_max=Order_Info.value.ui32val;
         // Update calibri:
         Servolist[actidx]->calibri=servo_calibri(actidx);
       }
    break;
    case INFO_SRV_ISD:                       // LG
      if(fromset){
         Servolist[actidx]->isd_min=Order_Info.value.ui32val;
       }
    break;
    case INFO_SRV_QPOS:                      // LG
      if(fromset) Servolist[actidx]->destn_us=Order_Info.value.ui32val;
    break;
    case INFO_SRV_FREQ:                      // LG
      if(fromset) Servolist[actidx]->freq=Order_Info.value.ui8val;
    break;
    case INFO_SRV_PINS:                      // LG
      if(fromset) Servolist[actidx]->pin_srv=Order_Info.value.ui8val;
    break;
    case INFO_SRV_PINP:                      // LG
      if(fromset) Servolist[actidx]->pin_pos=Order_Info.value.ui8val;
    break;
    case INFO_MATURITY:     // Server only  // LG        // Read only
     // Maturity might get re-calculated after an act_set (not act_get) but then
     // it will be updated elsewhere (where the re-calculation is done) - skip
    break;
    case INFO_SRV_SPOS:    // Ardu only    // Ask Ardu
    case INFO_SRV_STATE:   // Ardu only    // Ask Ardu  // Read only
    case INFO_SRV_TIMER:   // Ardu only    // Ask Ardu  // Read only
    case INFO_SRV_AFB:     // Ardu only    // Ask Ardu  // Read only
     // There is no local copy of of these members, so just skip them.
    break;
    case INFO_SRV_THMAX:   // Server only  // LG
    case INFO_SRV_THMIN:   // Server only  // LG
    case INFO_SRV_CALIB:   // Server only  // LG        // Read only
     // Theta max and min are dealt with under local set and calib is read only
     // so nothing to do here for these. There is no struct member for AFB
     // because this must be read afresh from the Arduino on demand.
    break;
    default:
    break;
   }
  break;

  case ACT_SENSOR:
   switch(Order_Info.infotype){
    case INFO_SEN_TYPE:                     // LG
      if(fromset) Sensorlist[actidx]->type=Order_Info.value.ui8val;
    break;
    case INFO_SEN_PIN:                      // LG
      if(fromset) Sensorlist[actidx]->pin_sense=Order_Info.value.ui8val;
    break;
    case INFO_MATURITY:     // Server only  // LG        // Read only
     // Maturity might get re-calculated after an act_set (not act_get) but then
     // it will be updated elsewhere (where the re-calculation is done) - skip
    break;
    case INFO_SEN_VAL:     // Ardu only    // Ask Ardu  // Read only
     // There is no local copy of of this member, so just skip it.
    break;
    default:
    break;
   }
  break;

  default:
  break;
 }

 return;
}

uint8_t adult_mlevel(uint8_t ml_type)
// Return the maturity threshold for an Mlevel of type ml_type
{
 switch(ml_type){
   case MLT_PARALLEL: return ADULT_MLEVEL_PAR;
   case MLT_PWM:      return ADULT_MLEVEL_PWM;
   default: break;
  }
 return 255;
}


uint8_t adult_stepper(uint8_t sd_type)
// Return the maturity threshold for a stepper of type sd_type
{
 switch(sd_type){
   case SD_TMC2209:  return ADULT_STEPPER_TMC;
   case SD_A4988:    return ADULT_STEPPER_DRV;
   case SD_DRV8825:  return ADULT_STEPPER_DRV;
   default: break;
  }
 return 255;
}

uint8_t mlevel_pins_have_gaps(uint8_t id)
// Return non-zero if there are gaps in the definition of the pin sequence
{
 uint8_t hole = 0;

 // Pin 0 must be set regardless of type so check that first
 if(Mlevellist[id]->ml_pin[0] == PIN_IS_UNSET) return 1;

 switch(Mlevellist[id]->ml_type){
   case MLT_PARALLEL:
    if(Mlevellist[id]->ml_pin[1] == PIN_IS_UNSET) hole++;
    else if(hole) return 1;
    if(Mlevellist[id]->ml_pin[2] == PIN_IS_UNSET) hole++;
    else if(hole) return 1;
    if(Mlevellist[id]->ml_pin[3] == PIN_IS_UNSET) hole++;
    else if(hole) return 1;
    if(Mlevellist[id]->ml_pin[4] == PIN_IS_UNSET) hole++;
    else if(hole) return 1;
    if(Mlevellist[id]->ml_pin[5] == PIN_IS_UNSET) hole++;
    else if(hole) return 1;
    if(Mlevellist[id]->ml_pin[6] == PIN_IS_UNSET) hole++;
    else if(hole) return 1;
    if(Mlevellist[id]->ml_pin[7] == PIN_IS_UNSET) hole++;
    else if(hole) return 1;
   break;    
   case MLT_PWM: // If we get here Pin 0 must be set, so just break.
   break;
   default:
   return 1;
  }
 return 0;
}

uint8_t act_nhs_invalid(uint8_t actype, uint8_t actidx)
// Checks if the null homing members of an actuator are sufficient for doing
// null homing or not.
// Returns 1 if they are not sufficient
// Returns 0 if they are sufficient
{
 uint8_t retval = 1;

 switch(actype){
  case ACT_STEPPER:
   if(Stepperlist[actidx]->nhs1){
     if((Stepperlist[actidx]->nhs0) || (Stepperlist[actidx]->nhs2)) retval=0;
    }
  break;
  case ACT_BINARY:
  break;
  case ACT_MLEVEL:
  break;
  case ACT_SERVO:
  break;
  case ACT_SENSOR:
  break;
  default: break;
 }
 return retval;
} 

void act_update_maturity(uint8_t actype, uint8_t actidx)
// Update the maturity value of an actuator by tallying how many of its 
// 'essential members' are set or differ from their default nascent value.
{
 uint8_t maturity=0,matopt1=0;

 switch(actype){
  case ACT_STEPPER:
   if(Stepperlist[actidx]->driver_type  != SD_UNDEF)   maturity++;
   // Nothing else may be set if the driver_type is unset - so only bother if it is:
   if(maturity){
   if(Stepperlist[actidx]->pin_pul != PIN_IS_UNSET) maturity++;
   if(Stepperlist[actidx]->pin_dir != PIN_IS_UNSET) maturity++;
   if(Stepperlist[actidx]->pin_ena != PIN_IS_UNSET) maturity++;
   if(Stepperlist[actidx]->dirn1 != DIRN_UNS) maturity++;
   if(Stepperlist[actidx]->dirn2 != DIRN_UNS) maturity++;
   if(Stepperlist[actidx]->nhs1){
     if((Stepperlist[actidx]->nhs0) || (Stepperlist[actidx]->nhs2)) matopt1++;
    }
   if( (Stepperlist[actidx]->pin_lim1 != PIN_IS_UNSET) ||
       (Stepperlist[actidx]->pin_lim2 != PIN_IS_UNSET) ) matopt1++; 
   if(matopt1) maturity++;
     // These are only required for non-TMC drivers.
     if(Stepperlist[actidx]->pin_ms1 != PIN_IS_UNSET) maturity++;
     if(Stepperlist[actidx]->pin_ms2 != PIN_IS_UNSET) maturity++;
     if(Stepperlist[actidx]->pin_ms3 != PIN_IS_UNSET) maturity++;
    }
   Stepperlist[actidx]->maturity=maturity;
  break;
  case ACT_BINARY:
   if(Binarylist[actidx]->pin_bin != PIN_IS_UNSET) maturity++;
   if(Binarylist[actidx]->on_val  != BIN_UNSET)    maturity++;
   if(Binarylist[actidx]->off_val != BIN_UNSET)    maturity++;
   Binarylist[actidx]->maturity=maturity;
  break;
  case ACT_MLEVEL:
   if(Mlevellist[actidx]->ml_type  != MLT_UNSET)   maturity++;
   // Nothing else may be set if the ml_type is unset - so only bother if it is:
   if(maturity){
     // Regardless of type, the first pin must be allocated:
     if(Mlevellist[actidx]->ml_pin[0] != PIN_IS_UNSET) maturity++;

     switch(Mlevellist[actidx]->ml_type){
       case MLT_PARALLEL:
        // Nothing more need be set for a PARALLEL (mature if >= 2)
       break;
       case MLT_PWM:
        // For a PWM the translation table must have at least one entry:
        if(Mlevellist[actidx]->ml_tt != NULL) maturity++; 
       break;
       default: break; // This should be impossible.
      }
    }
    Mlevellist[actidx]->maturity = maturity;   
  break;
  case ACT_SERVO:
   if(Servolist[actidx]->pin_srv != PIN_IS_UNSET) maturity++;
   Servolist[actidx]->maturity=maturity;
  break;
  case ACT_SENSOR:
   if(Sensorlist[actidx]->type  != SEN_UNSET)   maturity++;
   // Nothing else may be set if the type is unset - so only bother if it is:
   if(maturity){
     // Regardless of type, the sensor pin must be allocated:
     if(Sensorlist[actidx]->pin_sense != PIN_IS_UNSET) maturity++;
    }
    Sensorlist[actidx]->maturity = maturity;   
  break;

  default: break;
 }
 return;
} 

int act_fillforms_home(uint8_t actype,uint8_t actid,uint8_t hometype)
// Assesses the correctness of the parameters for a homing request for an
// actuator of type actype and, if correct, will fill in the required order
// forms to allow us to send that order to the Arduino. It is assumed a check
// for the existence of the actuator was made before calling this funtion and
// that the actuator does exist.
// Returns PARD_OK if all went well
// Returns the appropriate error value is something went wrong.
{
 int retval=PARD_OK;

 // Fill in the ID order form except otype (that will be done in the switch)
 Order_ID.actype = actype;
 Order_ID.actid  = actid;

 switch(actype){
   case ACT_STEPPER:
    // Request for an ORDER_STEPPER_HOME. First test if it can be done.
    // 1. Is the stepper mature enough for this?
    if(Stepperlist[actid]->maturity < adult_stepper(Stepperlist[actid]->driver_type)){
       retval = PARD_ERR_IMMATURE;
       goto endswitch;
      }
    // 2. Does the stepper have the required home limits set up?
    switch(hometype){
      case LIM_SWTC1:
       if(Stepperlist[actid]->pin_lim1==PIN_IS_UNSET){
         retval = PARD_ERR_LIMSWU;
         goto endswitch;
        }
      break;
      case LIM_SWTC2:
       if(Stepperlist[actid]->pin_lim2==PIN_IS_UNSET){
         retval = PARD_ERR_LIMSWU;
         goto endswitch;
        }
      break;
      case LIM_NULL1:
      case LIM_NULL2:
       if(act_nhs_invalid(actype,actid)){
         retval = PARD_ERR_LIMSWU;
         goto endswitch;
        }      
      break;
      default:
       retval = PARD_ERR_BUG; goto endswitch;
      break;
     }
    // If we got to here then all is OK to fill in the rest of the order forms
    // 1. Finish off the ID form
    Order_ID.otype = ORDER_STEPPER_HOME;
    // 2. Fill in the Order_Home form (only z is used for a single stepper)
    Order_Home.dirn_z = hometype;
   break;
   case ACT_BINARY:
    retval = -PARDU_ERR_IMPLEM;
   break;
   case ACT_MLEVEL:
    retval = -PARDU_ERR_IMPLEM;
   break;
   case ACT_SERVO:
    retval = -PARDU_ERR_IMPLEM;
   break;
   case ACT_SENSOR:
    retval = -PARDU_ERR_IMPLEM;
   break;
   default:
    retval = PARD_ERR_BUG;
   break;
  }
endswitch:
 if(retval!=PARD_OK) goto end_of_act_fillforms_home;
 

end_of_act_fillforms_home: 
 return retval;
}

//---------------------------------------------------------------------------//.
 //                           Actuator Management                           //.
  //-----------------------------------------------------------------------//.
#endif

  //-----------------------------------------------------------------------\\.
 //                         PARDCapture server comms                        \\.
//---------------------------------------------------------------------------\\.
#ifdef PARDUS

int ps_fmtchoice(char *argstr)
// Image 'save as' format checker. Given the input string, check that this
// corresponds to one of - yuv, pgm, bm8, ppm, bmp, png, int, jpg and return
// the corresponding SAF_ value or -1 if there is no match.
{
    if(!strcmp(argstr, "yuv"))      return SAF_YUYV;
    else if(!strcmp(argstr, "pgm")) return SAF_YP5;
    else if(!strcmp(argstr, "bm8")) return SAF_BM8;
    else if(!strcmp(argstr, "int")) return SAF_INT;
    else if(!strcmp(argstr, "ppm")) return SAF_RGB;
    else if(!strcmp(argstr, "bmp")) return SAF_BMP;
    else if(!strcmp(argstr, "png")) return SAF_PNG;
    else if(!strcmp(argstr, "jpg")) return SAF_JPG;

    return -1;
}

int32_t ps_alloc_insblob(void)
// Allocate ps_Instruction.blen bytes of memory to the instruction blob.
// Return PS_CB_OK if succeeded or PS_CB_E_MEB if failed
{
 void *new_blob;

 // Check that ps_Instruction.blen > 0, otherwise this is just a request to free
 if(ps_Instruction.blen <= 0) {
    if(ps_Instruction.blob != NULL){
       free(ps_Instruction.blob);
       ps_Instruction.blob = NULL;
    }
    ps_Instruction.blen = 0;        
    ps_Instruction.used.blob = 0; // Set the used flag to unused
    return PS_CB_OK;              
 }

 // Using an isolated temporary pointer and forcing a strict 1-byte unit size
 new_blob = calloc(ps_Instruction.blen, 1);
 if(new_blob == NULL){
   // Transaction Failure: Leave the original ps_Instruction.blob and its blen
   // completely untouched!
   return PS_CB_E_MEB;
 }

 // TRANSACTION SUCCESS HANDSHAKE: Commit changes safely
 if(ps_Instruction.blob != NULL){
     free(ps_Instruction.blob);
 }
 ps_Instruction.blob = new_blob;
 ps_Instruction.used.blob = 1;  // Set the used flag to used

 return PS_CB_OK;
}

int32_t ps_alloc_instr(void)
// Allocate ps_Instruction.slen bytes of memory to the instruction string.
// Return PS_CB_OK if succeeded or PS_CB_E_MES if failed
{
 // 1. Free any pre-existing memory cleanly
 if(ps_Instruction.str != NULL){
   free(ps_Instruction.str);
   ps_Instruction.str = NULL;
 }

 // 2. If slen <= 0, this is just a request to free the buffer
 if(ps_Instruction.slen <= 0) {     
    ps_Instruction.slen = 0;        
    ps_Instruction.used.str = 0;  // Set the used flag to unused
    return PS_CB_OK;              
 }

 // 3. Allocation uses exactly 1 byte per character element
 ps_Instruction.str = calloc(ps_Instruction.slen + 1, 1);
 if(ps_Instruction.str == NULL) {
     ps_Instruction.slen = 0; 
     return PS_CB_E_MES;
 }
 
 // 4. Set final sentinel null terminator and toggle tracking flag
 ps_Instruction.str[ps_Instruction.slen] = '\0'; 
 ps_Instruction.used.str = 1;  

 return PS_CB_OK;
}

int32_t   ps_ins_set(int imember,void *ivalptr,void *ibuff)
// Set the specified ps_Instruction structure member to the specified value.
// The ps_Instruction members should not be set directly but via this function.
// This is because this function performs housekeeping by setting the 'used'
// flags for any members that are used and it provides a layer of abstraction
// that can allow the details of the ps_Instruction struct to change without
// requiring the programmer to change all the code that sets its member.
// When using ibuff, this function assumes that ibuff points to a location with
// sufficient bytes alloced to copy over to the relevant ps_Instruction buffer
// according to the value pointed to by ivalptr - if not, undefined behaviour
// may occur.
// imember = the ps_Instruction struct member to work on. This will be one of
//           the PS_INS_... #defined constants
// ivalptr = pointer to a variable of appropriate type that has the value to set
//           that member to. In the cases of imember = PS_INS_STR or PS_INS_BLOB
//           this ivalue will pertain to the corresponding slen and blen. This
//           is why there are no PS_INS_.. constants for blen or slen.
// ibuff  =  pointer to the data to put into the str or blob
// Returns PS_CB_OK if all goes well
// Returns PS_CB_E_MEB if memory alloc fails for str or blob
{
 int32_t rv = PS_CB_OK;

 switch(imember) {
     case PS_INS_CIID:
         memcpy(&ps_Instruction.ciid, ivalptr, sizeof(uint32_t));
         break;

     case PS_INS_SIID:
         memcpy(&ps_Instruction.siid, ivalptr, sizeof(uint8_t));
         ps_Instruction.used.siid = 1;
         break;

     case PS_INS_SAFMT:
         memcpy(&ps_Instruction.safmt, ivalptr, sizeof(uint8_t));
         ps_Instruction.used.safmt = 1;
         break;

     case PS_INS_IVAL1:
         memcpy(&ps_Instruction.ival1, ivalptr, sizeof(int32_t));
         ps_Instruction.used.ival1 = 1;
         break;

     case PS_INS_IVAL2:
         memcpy(&ps_Instruction.ival2, ivalptr, sizeof(int32_t));
         ps_Instruction.used.ival2 = 1;
         break;

     case PS_INS_IVAL3:
         memcpy(&ps_Instruction.ival3, ivalptr, sizeof(int32_t));
         ps_Instruction.used.ival3 = 1;
         break;

     case PS_INS_DVAL1:
         memcpy(&ps_Instruction.dval1, ivalptr, sizeof(double));
         ps_Instruction.used.dval1 = 1;
         break;

     case PS_INS_DVAL2:
         memcpy(&ps_Instruction.dval2, ivalptr, sizeof(double));
         ps_Instruction.used.dval2 = 1;
         break;

     case PS_INS_DVAL3:
         memcpy(&ps_Instruction.dval3, ivalptr, sizeof(double));
         ps_Instruction.used.dval3 = 1;
         break;

     case PS_INS_ERR0:
         memcpy(&ps_Instruction.err[0], ivalptr, sizeof(int32_t));
         ps_Instruction.used.err = 1;
         break;

     case PS_INS_ERR1:
         memcpy(&ps_Instruction.err[1], ivalptr, sizeof(int32_t));
         ps_Instruction.used.err = 1;
         break;

     case PS_INS_ERR2:
         memcpy(&ps_Instruction.err[2], ivalptr, sizeof(int32_t));
         ps_Instruction.used.err = 1;
         break;

     case PS_INS_ERR3:
         memcpy(&ps_Instruction.err[3], ivalptr, sizeof(int32_t));
         ps_Instruction.used.err = 1;
         break;

     case PS_INS_STR:
         memcpy(&ps_Instruction.slen, ivalptr, sizeof(int32_t));
         rv = ps_alloc_instr();
         if(rv == PS_CB_OK) memcpy(ps_Instruction.str, ibuff, ps_Instruction.slen);
         break;

     case PS_INS_BLOB:
         memcpy(&ps_Instruction.blen, ivalptr, sizeof(int32_t));
         rv = ps_alloc_insblob();
         if(rv == PS_CB_OK) {
           if (ibuff != NULL) {
               memcpy(ps_Instruction.blob, ibuff, ps_Instruction.blen);
           } else rv = PS_CB_E_MEB;
         }
         break;

     default:
         break;
 }

 return rv;
}

void ps_ins_reset(void)
// Reset the ps_Instruction structure.
{
 ps_Instruction.ciid=0;    // Core instruction ID
 ps_Instruction.siid=0;    // Sub instruction ID
 ps_Instruction.safmt=0;   // Image format when dealing with image data
 ps_Instruction.ival1=0;   // Int 1
 ps_Instruction.ival2=0;   // Int 2
 ps_Instruction.ival3=0;   // Int 3
 ps_Instruction.dval1=0.0; // float 1
 ps_Instruction.dval2=0.0; // float 2
 ps_Instruction.dval3=0.0; // float 3
 ps_Instruction.slen=0;    // Number of bytes in the following string
 ps_alloc_instr();
 ps_Instruction.blen=0;    // Number of bytes in the following blob)
 ps_alloc_insblob();
 ps_Instruction.err[0]=0;  // Error value (0 = no error)
 ps_Instruction.err[1]=0;  // Error value (0 = no error)
 ps_Instruction.err[2]=0;  // Error value (0 = no error)
 ps_Instruction.err[3]=0;  // Error value (0 = no error)

 // The 'used' bit field
 ps_Instruction.used.siid  = 0;
 ps_Instruction.used.safmt = 0;
 ps_Instruction.used.ival1 = 0;
 ps_Instruction.used.ival2 = 0;
 ps_Instruction.used.ival3 = 0;
 ps_Instruction.used.dval1 = 0;
 ps_Instruction.used.dval2 = 0;
 ps_Instruction.used.dval3 = 0;
 ps_Instruction.used.err   = 0;
 ps_Instruction.used.str   = 0;
 ps_Instruction.used.blob  = 0;

 return;
}

void ps_initialise_instruction(void)
// Initialise the members of the ps_Instruction structure.
// This is called once at the start of the program and not again.
// Use ps_ins_reset() to reset the ps_Instruction structure subsequently.
{
 static uint8_t previously_called=0;

 if(previously_called) return;

 ps_Instruction.ciid=0;    // Core instruction ID
 ps_Instruction.siid=0;    // Sub instruction ID
 ps_Instruction.safmt=0;   // Image format when dealing with image data
 ps_Instruction.ival1=0;   // Int 1
 ps_Instruction.ival2=0;   // Int 2
 ps_Instruction.ival3=0;   // Int 3
 ps_Instruction.dval1=0.0; // Float 1
 ps_Instruction.dval2=0.0; // Float 2
 ps_Instruction.dval3=0.0; // Float 3
 ps_Instruction.slen=0;    // Number of bytes in the following string
 ps_Instruction.str=NULL;  // String of chars 
 ps_Instruction.blen=0;    // Number of bytes in the following blob)
 ps_Instruction.blob=NULL; // Blob for image or other general binary transmision
 ps_Instruction.err[0]=0;  // Error value (0 = no error)
 ps_Instruction.err[1]=0;  // Error value (0 = no error)
 ps_Instruction.err[2]=0;  // Error value (0 = no error)
 ps_Instruction.err[3]=0;  // Error value (0 = no error)

 // The 'used' bit field
 ps_Instruction.used.siid  = 0;
 ps_Instruction.used.safmt = 0;
 ps_Instruction.used.ival1 = 0;
 ps_Instruction.used.ival2 = 0;
 ps_Instruction.used.ival3 = 0;
 ps_Instruction.used.dval1 = 0;
 ps_Instruction.used.dval2 = 0;
 ps_Instruction.used.dval3 = 0;
 ps_Instruction.used.err   = 0;
 ps_Instruction.used.str   = 0;
 ps_Instruction.used.blob  = 0;

 previously_called=1;

 return;
}

int32_t ps_uc_to_int32(unsigned char *ucbuffer)
// Converts a buffer containing 4 unsigned char values into a single int32_t
// type. This function assumes the memory pointed to by ucbuffer is large
// enough (i.e. has at least 4 bytes alloced). Undefined behaviour (including a
// crash) may result if it does not.
{
 int32_t val;
 memcpy(&val, ucbuffer, sizeof(int32_t));
 return val;
}

uint32_t ps_uc_to_uint32(unsigned char *ucbuffer)
// Converts a buffer containing 4 unsigned char values into a single uint32_t
// type. This function assumes the memory pointed to by ucbuffer is large
// enough (i.e. has at least 4 bytes alloced). Undefined behaviour (including a
// crash) may result if it does not.
{
 uint32_t val;
 memcpy(&val, ucbuffer, sizeof(uint32_t));
 return val;
}

double ps_uc_to_dou(unsigned char *ucbuffer)
// Converts a buffer containing sizeof(double) unsigned char values into a
// single double type. This function assumes the memory pointed to by ucbuffer 
// is large enough (i.e. has at least 8 bytes alloced). Undefined behaviour 
// (including a crash) may result if it does not. 
{
  double result;
  memcpy(&result, ucbuffer, sizeof(double));
  return result;
}

void *ps_get_in_addr(struct sockaddr *sa)
// get sockaddr, IPv4 or IPv6:
{
	if (sa->sa_family == AF_INET) {
		return &(((struct sockaddr_in*)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

#if IS_WINDOWS_NATIVE
// ------------------------------------------------------------------------- //
// MS Windows versions of the PARDcapture socket functions (Winsock 2).       //
// The int connection descriptors used by the shared code are converted to    //
// and from Winsock SOCKET handles with ps_win_socket() / ps_win_fd(); socket  //
// errors are taken from WSAGetLastError() instead of errno; poll() becomes    //
// WSAPoll() and close() becomes closesocket(). Winsock is started once at     //
// program start by win_platform_init() (called from main).                    //
// ------------------------------------------------------------------------- //

int ps_send_dc(int socketfd, char *databuff, int *datalen)
// Send a capsule of data and ensure all of it is sent.
// Return 0 on success, -1 on error, and *datalen ends up as the number of bytes
// actually sent in any case.
// MS Windows version: on failure the Winsock error code is saved in
// ps_win_LastSockErr (send() returns SOCKET_ERROR, which equals -1).
{
int total = 0; // how many bytes we've sent so far
int bytesleft = *datalen; // how many we have left to send
int n = 0;
SOCKET sock = ps_win_socket(socketfd);

 while(total < *datalen) {
   n = send(sock, databuff+total, bytesleft, 0);
   if (n == SOCKET_ERROR) { ps_win_LastSockErr = WSAGetLastError(); break; }
   total += n;
   bytesleft -= n;
  }
 *datalen = total; // return number actually sent here

 return n==SOCKET_ERROR?-1:0; // return -1 on failure, 0 on success
}

int ps_recv_dc(int socketfd, unsigned char *databuff, int *datalen, int timeout_ms)
// Receive a capsule of data. *datalen is the maximum number of bytes the
// databuff can accommodate.
// Return 0 on success and *datalen ends up as the number of bytes actually sent
// otherwise the value of *datalen is left unchanged and the program returns one
// of the following non-zero values:
// 1 = the poll() function failed
// 2 = timeout occurred after the specified number of ms
// 3 = the recv() function failed
// 4 = poll error: An error on the socket
// 5 = poll error: Hang up (also returned when recv() returns 0 bytes, i.e.
//     when the peer has closed the connection in an orderly way: 17.11.26)
// 6 = poll error: Invalid socket
// 7 = poll error: Error condition NOS
// MS Windows version: WSAPoll() (the Winsock equivalent of poll(), with the same
// POLLIN/POLLERR/POLLHUP/POLLNVAL flags) replaces poll(). The Winsock error
// code of a failed call is saved in ps_win_LastSockErr. Note that WSAPoll may
// report a peer's orderly shutdown as POLLHUP alone (return value 5) or as
// POLLIN followed by a zero-byte recv(); since 17.11.26 both give 5, as on
// Linux and macOS (the Prof's choice N1).
{
 WSAPOLLFD ufds[1];
 int rv,numbytes=0;
 int rstatus=0;

  rstatus=0;
  // Check if there is anything on the read buffer or we could hang forever
  // waiting for recv to read something.
  ufds[0].fd = ps_win_socket(socketfd);
  ufds[0].events = POLLIN;
  ufds[0].revents = 0;
  // wait for events on the sockets with a timeout_ms timeout
  rv = WSAPoll(ufds, 1, timeout_ms);
  if (rv == SOCKET_ERROR) {
    ps_win_LastSockErr = WSAGetLastError();
    rstatus=1; goto end_recv_dc;
   } else if (rv == 0) {
    rstatus=2; goto end_recv_dc;
   } else {
    // check if there is anything to read:
    if (ufds[0].revents & POLLIN) { // if so, read it
      if ((numbytes = recv(ufds[0].fd, (char *)databuff, *datalen, 0))
                                                              == SOCKET_ERROR) {
        ps_win_LastSockErr = WSAGetLastError();
        rstatus=3; goto end_recv_dc;
       }
      // (17.11.26) 0 bytes from a stream socket asked for at least one byte
      // means that the peer has closed the connection: report a hang-up, as
      // the POLLHUP case below does (the Prof's choice N1; the same rule as
      // in pardcap since 16.11.26).
      if (numbytes == 0 && *datalen > 0) {
        rstatus=5; goto end_recv_dc;
       }
     } else if (ufds[0].revents & POLLERR) { // An error on the socket
      rstatus=4; goto end_recv_dc;
     } else if (ufds[0].revents & POLLHUP) { // Hang up
      rstatus=5; goto end_recv_dc;
     } else if (ufds[0].revents & POLLNVAL) { // Invalid socket
      rstatus=6; goto end_recv_dc;
     } else { // Some other error condition
      rstatus=7; goto end_recv_dc;
     }
   }

  if(rstatus==0){ // Something was read set *datalen to the number of bytes read
     *datalen=numbytes;
    }

end_recv_dc:
 return rstatus;
}

void ps_print_recv_error(int rv, char *infostr)
// MS Windows version: the poll and recv errors are described with the Winsock
// error code saved (in ps_win_LastSockErr) when the failure occurred.
{
 switch(rv){
   case 1:
     sprintf(infostr,": recv error: WSAPoll: %s",
                                     win_errstr((DWORD)ps_win_LastSockErr));
   break;
   case 2:
     sprintf(infostr,": recv error: Timeout occurred.");
   break;
   case 3:
     sprintf(infostr,": recv error: %s.",win_errstr((DWORD)ps_win_LastSockErr));
   break;
   case 4:
     sprintf(infostr,": recv error: Input socket.");
   break;
   case 5:
     sprintf(infostr,": recv error: Connected entity hung up.");
   break;
   case 6:
     sprintf(infostr,": recv error: The requested socket is invalid.");
   break;
   case 7:
     sprintf(infostr,": recv error: An unspecified error in the inbound socket.");
   break;
   default:
     sprintf(infostr,": recv error: Programming error.");
   break;
  }
 return;
}

int ps_connect(char *ps_ipaddress, char *ps_port)
// Attempt to add a new connection to a PARD capture server to the PSlist.
// Return CNE_SUCCESS on success.
// Return one of the EAI_.. error codes if getaddrinfo fails to find a server
// Returns CNE_CONFAIL if failed to connect with a found server
// Returns one of the ps_recv_dc error codes if fails to receive data from a
//         connected server
// Returns 1 for other comms errors
// A message is printed to MT_ERR to help define the error in all cases
// MS Windows version: Winsock SOCKET handles (checked against INVALID_SOCKET
// and SOCKET_ERROR), closesocket() and Winsock error texts. The connected
// SOCKET is stored in the (int) PSlist[].fd via ps_win_fd().
{
	unsigned char recvbuf[PS_MAXDATASIZE];
	struct addrinfo hints, *servinfo, *p;
	int rv,ps_comms_fd;
	SOCKET ps_comms_sock = INVALID_SOCKET;
	char s[INET6_ADDRSTRLEN];
 int msglen;
 int maxbuffsz;
 uint8_t dupscore;
 char infostr[320];
 ConnectNode* newcnmem;
 size_t newcnsize;
 unsigned int idx;

 rv=CNE_SUCCESS;

 // Ensure the proferred ip address and port combo is not already taken by an
 // existing connected node but if the node exists and is unconnected then try
 // to connect it but set a marker (dupscore==2) so we don't try to create the
 // node anew later).
 for(idx=0,dupscore=0; idx < NPSs; idx++){
     if(!strcmp(ps_ipaddress, PSlist[idx].ipaddr)) dupscore++;
     if(!strcmp(ps_port, PSlist[idx].port)) dupscore++;
     if(dupscore==2){
       if(PSlist[idx].fd<0) break; else return CNE_DUPNAME;
      } else dupscore=0;
   }

 // Set up structs for a connection search
	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	sprintf(infostr,": PARDclient: Looking for server %s:%s",ps_ipaddress,ps_port);
 show_message(infostr,fnn,MT_ERR,0);

	if ((rv = getaddrinfo(ps_ipaddress, ps_port, &hints, &servinfo)) != 0) {
		sprintf(infostr,": PARDclient: getaddrinfo: %s", gai_strerrorA(rv));
  show_message(infostr,fnn,MT_ERR,0);
		goto end_pardcap_connect;
	}

	// loop through all the results and connect to the first we can
	for(p = servinfo; p != NULL; p = p->ai_next) {
		if ((ps_comms_sock = socket(p->ai_family, p->ai_socktype,
				p->ai_protocol)) == INVALID_SOCKET) {
				sprintf(infostr,": PARDclient: socket: %s",
                                     win_errstr((DWORD)WSAGetLastError()));
    show_message(infostr,fnn,MT_ERR,0);
			continue;
		}

		if (connect(ps_comms_sock, p->ai_addr, (int)p->ai_addrlen) == SOCKET_ERROR) {
   sprintf(infostr,": PARDclient: connect: %s",
                                     win_errstr((DWORD)WSAGetLastError()));
   show_message(infostr,fnn,MT_ERR,0);
			closesocket(ps_comms_sock);
			ps_comms_sock = INVALID_SOCKET;
			continue;
		}

		break;
	}

	if (p == NULL) {
  show_message(": PARDclient: failed to connect",fnn,MT_ERR,0);
  freeaddrinfo(servinfo); // Avoid memory leaks
		return CNE_CONFAIL;
	}

 // The shared code keeps connections in int descriptors (see ps_win_fd), so
 // make certain that this SOCKET value is representable as a positive int:
 if((uintptr_t)ps_comms_sock > (uintptr_t)INT_MAX){
   show_message(": PARDclient: Winsock returned a socket handle that cannot be"
                " stored (handle > INT_MAX).",fnn,MT_ERR,0);
   closesocket(ps_comms_sock);
   freeaddrinfo(servinfo);
   return 1;
  }
 ps_comms_fd = ps_win_fd(ps_comms_sock);

	inet_ntop(p->ai_family, ps_get_in_addr((struct sockaddr *)p->ai_addr),
			                                                                s, sizeof s);
	sprintf(infostr,": PARDclient: connecting to %s", s);
 show_message(infostr,fnn,MT_INFO,0);

	freeaddrinfo(servinfo); // all done with this structure

 show_message(": PARDclient: Waiting for response ...",fnn,MT_INFO,0);
 recvbuf[0]='\0';
 maxbuffsz = PS_MAXDATASIZE-1;
 if ((rv = ps_recv_dc(ps_comms_fd, recvbuf, &maxbuffsz, 3500))) {
    ps_print_recv_error(rv,infostr);
    show_message(infostr,fnn,MT_ERR,0);
    closesocket(ps_comms_sock);
		  	goto end_pardcap_connect;
   } else {

   // Test if the respone is compatible with an acceptable server:
   if(maxbuffsz!=PS_MSGLEN){
      sprintf(infostr,": PARDclient: received %d bytes but was expecting %d",
                                                          maxbuffsz,PS_MSGLEN);
      show_message(infostr,fnn,MT_ERR,0);
      show_message(": PARDclient: This is not an acceptable server.",
                                                       fnn,MT_ERR,0);
      rv=1;
      closesocket(ps_comms_sock);
    		goto end_pardcap_connect;
     } else {
      recvbuf[maxbuffsz] = '\0';
      sprintf(infostr,": PARDclient: received '%s'",recvbuf);
      show_message(infostr,fnn,MT_ERR,0);
      if(strncmp("PARD_Capture",(const char *)recvbuf,PS_MSGLEN)){
        printf("PARDclient: This is not an acceptable server.\n");
        rv=1;
        closesocket(ps_comms_sock);
    	  	goto end_pardcap_connect;
       }
     }
  }

 show_message(": PARDclient: sending 'PARDucom'",fnn,MT_INFO,0);
 msglen=(int)strlen("PARDucom");
 if (ps_send_dc(ps_comms_fd, "PARDucom", &msglen) == -1){
    sprintf(infostr,": PARDclient: ps_send_dc: %s",
                                         win_errstr((DWORD)ps_win_LastSockErr));
    show_message(infostr,fnn,MT_ERR,0);
    sprintf(infostr,": We only sent %d bytes because of the error.", msglen);
    show_message(infostr,fnn,MT_INFO,0);
    rv=1;
    closesocket(ps_comms_sock);
    goto end_pardcap_connect;
  }

 if(dupscore<2){ // Only create a new node if it doesn't already exist
   // Try to get memory for the new connection node
   newcnsize = SizePSs + sizeof(ConnectNode);
   newcnmem = realloc(PSlist, newcnsize);
   if(newcnmem == NULL){
     show_message(": Failed to get memory for the new connection.\n",
                                                              fnn, MT_ERR, 1);
     closesocket(ps_comms_sock);
     return CNE_MEMFAIL;
    }
   PSlist = newcnmem;
   SizePSs = newcnsize;
   idx = NPSs;
   NPSs++;
  }

 sprintf(infostr,": PARDclient: Success. Connected to PARD Capture server %u.",
                                                          NPSs);
 show_message(infostr,fnn,MT_INFO,0);

 // Allocate the members of the new node
 sprintf(PSlist[idx].ipaddr,"%s",ps_ipaddress);
 sprintf(PSlist[idx].port,"%s",ps_port);
 PSlist[idx].fd = ps_comms_fd;

end_pardcap_connect:

	return rv;
}

void ps_hangup(unsigned int idx)
// Hang up on connection idx (if it is connected and if it exists in the PSlist
// of connections - otherwise do nothing.
// MS Windows version: closesocket() replaces close().
{
 char infostr[320];

 if(idx>=NPSs) return; // Can't hang up on a non-existent connection
 if(PSlist[idx].fd < 0) return; // Node is not connected - nothing to do.

 sprintf(infostr,": Hanging up on PARD capture server %u",idx+1);
 show_message(infostr,fnn,MT_INFO,0);
 closesocket(ps_win_socket(PSlist[idx].fd));
 PSlist[idx].fd = -1;
	return;
}
#else
int ps_send_dc(int socketfd, char *databuff, int *datalen)
// Send a capsule of data and ensure all of it is sent.
// Return 0 on success, -1 on error, and *datalen ends up as the number of bytes
// actually sent in any case.
{
int total = 0; // how many bytes we've sent so far
int bytesleft = *datalen; // how many we have left to send
int n = 0;

 while(total < *datalen) {
   n = send(socketfd, databuff+total, bytesleft, 0);
   if (n == -1) { break; }
   total += n;
   bytesleft -= n;
  }
 *datalen = total; // return number actually sent here

 return n==-1?-1:0; // return -1 on failure, 0 on success
}

int ps_recv_dc(int socketfd, unsigned char *databuff, int *datalen, int timeout_ms)
// Receive a capsule of data. *datalen is the maximum number of bytes the
// databuff can accommodate.
// Return 0 on success and *datalen ends up as the number of bytes actually sent
// otherwise the value of *datalen is left unchanged and the program returns one
// of the following non-zero values:
// 1 = the poll() function failed
// 2 = timeout occurred after the specified number of ms
// 3 = the recv() function failed
// 4 = poll error: An error on the socket
// 5 = poll error: Hang up (also returned when recv() returns 0 bytes, i.e.
//     when the peer has closed the connection in an orderly way: 17.11.26)
// 6 = poll error: Invalid socket
// 7 = poll error: Error condition NOS
{
 struct pollfd ufds[1];
 int rv,numbytes;
 int rstatus=0;

  rstatus=0;
  // Check if there is anything on the read buffer or we could hang forever
  // waiting for recv to read something.
  ufds[0].fd = socketfd;
  ufds[0].events = POLLIN;
  // wait for events on the sockets with a timeout_ms timeout
  rv = poll(ufds, 1, timeout_ms);
  if (rv == -1) {
    rstatus=1; goto end_recv_dc;
   } else if (rv == 0) {
    rstatus=2; goto end_recv_dc;
   } else {
    // check if there is anything to read:
    if (ufds[0].revents & POLLIN) { // if so, read it
      if ((numbytes = recv(socketfd, databuff, *datalen, 0)) == -1) {
        rstatus=3; goto end_recv_dc;
       }
      // (17.11.26) 0 bytes from a stream socket asked for at least one byte
      // means that the peer has closed the connection: report a hang-up, as
      // Windows does (the Prof's choice N1; the same rule as in pardcap since
      // 16.11.26). Before, ps_receive_blob() counted 250 empty reads before
      // giving up with "Received 0 bytes ... Giving up." (rv = -4).
      if (numbytes == 0 && *datalen > 0) {
        rstatus=5; goto end_recv_dc;
       }
     } else if (ufds[0].revents & POLLERR) { // An error on the socket
      rstatus=4; goto end_recv_dc;
     } else if (ufds[0].revents & POLLHUP) { // Hang up
      rstatus=5; goto end_recv_dc;
     } else if (ufds[0].revents & POLLNVAL) { // Invalid socket
      rstatus=6; goto end_recv_dc;
     } else { // Some other error condition
      rstatus=7; goto end_recv_dc;
     }
   }

  if(rstatus==0){ // Something was read set *datalen to the number of bytes read
     *datalen=numbytes;
    }

end_recv_dc:
 return rstatus;
}

void ps_print_recv_error(int rv, char *infostr)
{
 switch(rv){
   case 1:
     sprintf(infostr,": recv error: poll: %s",strerror(errno));
   break;
   case 2:
     sprintf(infostr,": recv error: Timeout occurred.");
   break;
   case 3:
     sprintf(infostr,": recv error: %s.",strerror(errno));
   break;
   case 4:
     sprintf(infostr,": recv error: Input socket.");
   break;
   case 5:
     sprintf(infostr,": recv error: Connected entity hung up.");
   break;
   case 6:
     sprintf(infostr,": recv error: The requested socket is invalid.");
   break;
   case 7:
     sprintf(infostr,": recv error: An unspecified error in the inbound socket.");
   break;
   default:
     sprintf(infostr,": recv error: Programming error.");
   break;
  }
 return;
}

int ps_connect(char *ps_ipaddress, char *ps_port)
// Attempt to add a new connection to a PARD capture server to the PSlist.
// Return CNE_SUCCESS on success.
// Return one of the EAI_.. error codes if getaddrinfo fails to find a server
// Returns CNE_CONFAIL if failed to connect with a found server
// Returns one of the ps_recv_dc error codes if fails to receive data from a
//         connected server
// Returns 1 for other comms errors
// A message is printed to MT_ERR to help define the error in all cases
{
	unsigned char recvbuf[PS_MAXDATASIZE];
	struct addrinfo hints, *servinfo, *p;
	int rv,ps_comms_fd;
	char s[INET6_ADDRSTRLEN];
 int msglen;
 int maxbuffsz;
 uint8_t dupscore;
 char infostr[320];
 ConnectNode* newcnmem;
 size_t newcnsize;
 unsigned int idx;

 rv=CNE_SUCCESS;

 // Ensure the proferred ip address and port combo is not already taken by an
 // existing connected node but if the node exists and is unconnected then try
 // to connect it but set a marker (dupscore==2) so we don't try to create the
 // node anew later).
 for(idx=0,dupscore=0; idx < NPSs; idx++){
     if(!strcmp(ps_ipaddress, PSlist[idx].ipaddr)) dupscore++;
     if(!strcmp(ps_port, PSlist[idx].port)) dupscore++;
     if(dupscore==2){
       if(PSlist[idx].fd<0) break; else return CNE_DUPNAME;
      } else dupscore=0;
   }

 // Set up structs for a connection search
	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	sprintf(infostr,": PARDclient: Looking for server %s:%s",ps_ipaddress,ps_port);
 show_message(infostr,fnn,MT_ERR,0);

	if ((rv = getaddrinfo(ps_ipaddress, ps_port, &hints, &servinfo)) != 0) {
		sprintf(infostr,": PARDclient: getaddrinfo: %s", gai_strerror(rv));
  show_message(infostr,fnn,MT_ERR,0);
		goto end_pardcap_connect;
	}

	// loop through all the results and connect to the first we can
	for(p = servinfo; p != NULL; p = p->ai_next) {
		if ((ps_comms_fd = socket(p->ai_family, p->ai_socktype,
				p->ai_protocol)) == -1) {
				sprintf(infostr,": PARDclient: socket: %s",strerror(errno));
    show_message(infostr,fnn,MT_ERR,0);
			continue;
		}

		if (connect(ps_comms_fd, p->ai_addr, p->ai_addrlen) == -1) {
   sprintf(infostr,": PARDclient: connect: %s",strerror(errno));
   show_message(infostr,fnn,MT_ERR,0);
			close(ps_comms_fd);
			continue;
		}

		break;
	}

	if (p == NULL) {
  show_message(": PARDclient: failed to connect",fnn,MT_ERR,0);
  freeaddrinfo(servinfo); // Avoid memory leaks
		return CNE_CONFAIL;
	}

	inet_ntop(p->ai_family, ps_get_in_addr((struct sockaddr *)p->ai_addr),
			                                                                s, sizeof s);
	sprintf(infostr,": PARDclient: connecting to %s", s);
 show_message(infostr,fnn,MT_INFO,0);

	freeaddrinfo(servinfo); // all done with this structure

 show_message(": PARDclient: Waiting for response ...",fnn,MT_INFO,0);
 recvbuf[0]='\0';
 maxbuffsz = PS_MAXDATASIZE-1;
 if ((rv = ps_recv_dc(ps_comms_fd, recvbuf, &maxbuffsz, 3500))) {
    ps_print_recv_error(rv,infostr);
    show_message(infostr,fnn,MT_ERR,0);
    close(ps_comms_fd);
	  	goto end_pardcap_connect;
   } else {

   // Test if the respone is compatible with an acceptable server:
   if(maxbuffsz!=PS_MSGLEN){
      sprintf(infostr,": PARDclient: received %d bytes but was expecting %d",
                                                          maxbuffsz,PS_MSGLEN);
      show_message(infostr,fnn,MT_ERR,0);
      show_message(": PARDclient: This is not an acceptable server.",
                                                       fnn,MT_ERR,0);
      rv=1;
      close(ps_comms_fd);
    		goto end_pardcap_connect;
     } else {
      recvbuf[maxbuffsz] = '\0';
      sprintf(infostr,": PARDclient: received '%s'",recvbuf);
      show_message(infostr,fnn,MT_ERR,0);
      if(strncmp("PARD_Capture",(const char *)recvbuf,PS_MSGLEN)){
        printf("PARDclient: This is not an acceptable server.\n");
        rv=1;
        close(ps_comms_fd);
    	  	goto end_pardcap_connect;
       }
     }
  }

 show_message(": PARDclient: sending 'PARDucom'",fnn,MT_INFO,0);
 msglen=(int)strlen("PARDucom");
 if (ps_send_dc(ps_comms_fd, "PARDucom", &msglen) == -1){
    sprintf(infostr,": PARDclient: ps_send_dc: %s",strerror(errno));
    show_message(infostr,fnn,MT_ERR,0);
    sprintf(infostr,": We only sent %d bytes because of the error.", msglen);
    show_message(infostr,fnn,MT_INFO,0);
    rv=1;
    close(ps_comms_fd);
    goto end_pardcap_connect;
  }

 if(dupscore<2){ // Only create a new node if it doesn't already exist
   // Try to get memory for the new connection node
   newcnsize = SizePSs + sizeof(ConnectNode);
   newcnmem = realloc(PSlist, newcnsize);
   if(newcnmem == NULL){
     show_message(": Failed to get memory for the new connection.\n",
                                                              fnn, MT_ERR, 1);
     close(ps_comms_fd);
     return CNE_MEMFAIL;
    }
   PSlist = newcnmem;
   SizePSs = newcnsize;
   idx = NPSs;
   NPSs++;
  }

 sprintf(infostr,": PARDclient: Success. Connected to PARD Capture server %u.",
                                                          NPSs);
 show_message(infostr,fnn,MT_INFO,0);

 // Allocate the members of the new node
 sprintf(PSlist[idx].ipaddr,"%s",ps_ipaddress);
 sprintf(PSlist[idx].port,"%s",ps_port);
 PSlist[idx].fd = ps_comms_fd;

end_pardcap_connect:

	return rv;
}

void ps_hangup(unsigned int idx)
// Hang up on connection idx (if it is connected and if it exists in the PSlist
// of connections - otherwise do nothing.
{
 char infostr[320];

 if(idx>=NPSs) return; // Can't hang up on a non-existent connection
 if(PSlist[idx].fd < 0) return; // Node is not connected - nothing to do.

 sprintf(infostr,": Hanging up on PARD capture server %u",idx+1);
 show_message(infostr,fnn,MT_INFO,0);
 close(PSlist[idx].fd);
 PSlist[idx].fd = -1;
	return;
}
#endif

int ps_resize_blob(int32_t bsz)
// Change the size of the ps_Blob with housekeeping. Return 1 on failure.
{
 int rv=0;
 void *new_blob;

 // If ps_BlobSize<2 then we can't do anything - some corruption has occurred
 if(ps_BlobSize < 2){
   show_message("Could not re-size the ps_Blob. This is a serious memory error."
                " Please save your work and exit ASAP because the program has "
                "become unstable and may crash at any time without notice.",
                "Memory Error: ", MT_ERR, 0);
     rv = 1; 
#ifdef PARDCAP_VERN
     ps_Server_allowed = 0;
#endif
     goto end_ps_resize_blob;
  }

 if(bsz == ps_BlobSize){
     memset(ps_Blob, 0, ps_BlobSize);
     return 0;
  }

 new_blob = calloc(bsz,sizeof(*ps_Blob));

 if(new_blob == NULL){
     show_message("Could not re-initialise the ps_Blob. This is a "
                   "serious memory error. Please save your work and exit ASAP "
                   "because the program has become unstable and may crash at "
                   "any time without notice.", "Memory Error: ", MT_ERR, 0);
#ifdef PARDCAP_VERN
     ps_Server_allowed = 0;
#endif
     rv = 1;
     goto end_ps_resize_blob;
  }

 // Commit changes to the hardware registers safely
 free(ps_Blob);
 ps_Blob = new_blob;
 ps_BlobSize = bsz;

end_ps_resize_blob:
 return rv;
}

int ps_initialise_blob(void)
// Allocate memory for ps_Blob - this is a one-off initial allocation
// because I always free(ps_Blob); before doing a new allocation.
// This returns 1 on success and 0 on failure (so, opposite to most function).
{
 int rv=1;
 static uint8_t previously_called=0;

 if(previously_called) return 1;

 ps_BlobSize=2;
 ps_Blob=calloc(ps_BlobSize,sizeof(*ps_Blob));
 if(ps_Blob==NULL){
  show_message("Could not allocate RAM for the ps_Blob. Server functionality "
               "will be disabled.","Error: ",MT_ERR,0);
  rv=0;  goto end_ps_initialise_blob;
 }
end_ps_initialise_blob:
 previously_called=1;

 return rv;
}

int ps_receive_blob(int timeout_ms)
// poll a connected client for a maximum of timeout_ms and receive a properly
// formatted instruction message from a connected client. If this succeeds then
// the header integer is stripped off the incomming encapsulated blob to leave
// the infoblob (ready for checking and actioning) in ps_Blob.
// Returns -4 if blob reception or processing is already in progress or if the
//            client comms socket is not currently set.
// Returns -3 if the first integer value received is not positive
// Returns -2 if calloc failed to get memory for the receive buffer
// Returns -1 if no client is connected
// Returns 0 if success and all expected data is read
// Returns 1 if the poll() function returned an error
// Returns 2 if a timeout occured
// Returns 3 if the recv() function returned an error
// Returns 4 to 7 if there is a loss of connection with the client for some
// reason (e.g. bad socket, client hung up, etc.) and so the user should check
// for these error values and call ps_stop_server() if they are detected.
// Then, optionally, ps_start_server() to allow for a re-connect.
{
 int rv,recattempts,maxattempts,recdx,currpos;
 #ifdef PARDCAP_VERN
 int ssrv,hup,start_pollbar=1;
 #endif
 int32_t maxbuffsz,reqbuffsz,maxbuffcpy;
 char infomsg[256];
 unsigned char *recvbuf;

 if(ps_Comms_fd<=0)    return -4;
#ifdef PARDCAP_VERN
 if(ps_Receiving_blob) return -4;
 if(ps_Actioning_blob) return -4;
 ps_Receiving_blob=1;
#endif

#ifdef PARDCAP_VERN
 hup=0;
 ssrv=0;
#endif
 rv=-2;
 reqbuffsz=maxbuffsz=maxbuffcpy = 4;

 if(ps_resize_blob(reqbuffsz)){
   show_message(": Calloc failed on blob.",fnn,MT_ERR,0);
#ifdef PARDCAP_VERN
   hup=1;
   if(ps_BlobSize<2) ssrv=1;
#endif
   goto end_of_ps_receive_blob;
  } else recvbuf=ps_Blob;

 rv=-1;

 if(!ps_CliSrv_connected) goto end_of_ps_receive_blob;

 recdx = 0;       // Current round of complete data acquisition
 recattempts=0;   // Number of attempts at getting all expected data
 maxattempts=250; // Max. attempts before giving up. The count resets whenever
                  // new data is received in a given attempt so this is actually
                  // the max. number of attempts at getting all expected data 
                  // with no new data received in any of those sequential
                  // attempts.
 currpos=0;  // current position in a partially received message


ps_receive_more:

#ifdef PARDCAP_VERN
   if(start_pollbar)
   ps_pollbar(PS_PB_INITIATE);
#endif

   if ((rv = ps_recv_dc(ps_Comms_fd, recvbuf+currpos, &maxbuffsz, timeout_ms)))
    {
#ifndef PARDCAP_VERN
     ps_print_recv_error(rv,infomsg);
     show_message(infomsg,fnn,MT_ERR,0);
#endif
#ifdef PARDCAP_VERN
     if(rv>1) ps_pollbar(PS_PB_PROGRESS);
#endif 
     if(rv==2) goto end_of_ps_receive_blob;
     if(rv==6){
         sprintf(infomsg,": Invalid socket value = %d.",ps_Comms_fd);
         show_message(infomsg,fnn,MT_ERR,0);
      }
#ifdef PARDCAP_VERN
     if(rv>2) hup=1; // Hang up and go back to server listening mode on exit.
#endif
     goto end_of_ps_receive_blob;
    } else {

    // The ps_recv_dc function returned without error so now see if we got all
    // the bytes we requested. Retry 5 times if not (before giving up).
#ifdef PARDCAP_VERN
    ps_pollbar(PS_PB_TERMINATE);
    start_pollbar=0;
#endif
    if(maxbuffsz<reqbuffsz){
       if(maxbuffsz==maxbuffcpy || maxbuffsz==0) recattempts++;
        else recattempts=0;
       if(recattempts>=maxattempts){ 
         sprintf(infomsg,": Received %"PRId32
            " bytes but was expecting %"PRId32" after many tries. Giving up.",
                                                           maxbuffsz,reqbuffsz);
        show_message(infomsg,fnn,MT_ERR,0);
        rv=-4; 
#ifdef PARDCAP_VERN
        hup=1;
#endif
        goto end_of_ps_receive_blob;
       } else {
         sprintf(infomsg,": Received %"PRId32
                             " bytes (out of %"PRId32"). Trying for more ...",
                                                           maxbuffsz,reqbuffsz);
         show_message(infomsg,fnn,MT_ERR,0);
         currpos+=maxbuffsz;
         reqbuffsz-=maxbuffsz;
         maxbuffsz=maxbuffcpy=reqbuffsz;
         goto ps_receive_more;
       }
     }
   }
 // If we get to here then we got all the data we were expecting (in terms of
 // number of bytes) without errors. So analyse the values and act on them.

 // The PARD Capture blob transfer protocol:
 // For the very first reception we only request 4 bytes, the value of which is
 // interperted as 4 byte signed integer (int32_t) and tells us how many bytes
 // in total to expect next if it is non-negative (from 0 to 2147483647).
 // This protocol allows for sending / receiving a 24 bpp image of dimensions
 // 26754 x 26754 pixels (or 715 megapixels) which should be sufficient for
 // any single capture a PARDUS camera could ever make.
 // If the first int has a negative value then this indicates an error and I
 // will need to work out details of how to respond to this later.
 // This protocol is just for sending and receiving blobs. Interpretation of the
 // contents of the blob involves a separate protocol.
 reqbuffsz=0;
 if(recdx==0){
   // The very first aquisition - just 4 bytes - get the value as an integer:
   reqbuffsz=ps_uc_to_int32(recvbuf);
   sprintf(infomsg,
        ": Blob expected to be %"PRId32" bytes long.",reqbuffsz);
        show_message(infomsg,fnn,MT_INFO,0);
   if(reqbuffsz<=0){ // If this is 0 or negative, there is something wrong
     // Error management and return
       show_message(": The first integer did not have a positive value.",fnn,
                                                                      MT_ERR,0);
       rv=-3; // The 'first integer is not positive' return code.
#ifdef PARDCAP_VERN
       hup=1;
#endif
       goto end_of_ps_receive_blob;
    } else { // Otherwise get set to receive the blob
     // First attempt to realloc the receiver buffer to the required size
     maxbuffsz=reqbuffsz;

     if(ps_resize_blob(reqbuffsz)){
       show_message(": Calloc failed on blob buffer.\n",fnn,MT_ERR,0);
       rv=-2; // The 'insufficient memory / malloc failed' return code.
#ifdef PARDCAP_VERN
       hup=1; // The hangup code
       if(ps_BlobSize<2) ssrv=1; // The 'server not available' code
#endif
       goto end_of_ps_receive_blob;
      } else {
       recvbuf=ps_Blob;
       memset(recvbuf,0,ps_BlobSize);
      }
     // If we reach here then we have sufficient storage for the blob so get set
     // to receive it:
     recdx++; // Update the 'Current round of complete data reception' index
     recattempts=0; // Reset the 'No. of attempts at getting all expected data'
     currpos=0; // Reset the current position in a partially received message
     goto ps_receive_more;
    }
  } else {
   // All the expected blob has arrived
   sprintf(infomsg,": Received OK (%d bytes).",ps_BlobSize);
   show_message(infomsg,fnn,MT_INFO,0);
  }
#ifdef PARDCAP_VERN
 ps_Actioning_blob=1; // The received blob needs actioning
#endif

end_of_ps_receive_blob:
#ifdef PARDCAP_VERN
 ps_Receiving_blob=0;
 if(hup) ps_hang_up();
 if(ssrv) ps_stop_server(1); // Serious memory error has occured - server ops no
                             // longer possible.
#endif
 return rv;
}

int32_t ps_increment_blobindex(int32_t *idx,int32_t incr)
// Increment the blob index by incr (bytes) with housekeeping.
// Return PS_CB_OK if the incremented index is still within the ps_BlobSize
// Return PS_CB_E_BSZ otherwise
{
 (*idx)+=incr;
 if(*idx>=ps_BlobSize) return PS_CB_E_BSZ;
 return PS_CB_OK;
}

int32_t ps_check_cuss_type(int8_t)
// Test is the supplied int8_t is a valid custom frame grabber setting type.
// Return PS_CB_OK if it is, PS_CB_E_SID otherwise.
{
 int rv=PS_CB_OK;

 switch(ps_Instruction.siid){
  case PS_CUSS_FTOSK: // Frames-to-skip 
  case PS_CUSS_SKFRM: // Skip frames 
  case PS_CUSS_GAIYU: // YUYV Gain 
  case PS_CUSS_BIAYU: // YUYV Bias 
  case PS_CUSS_FRMNO: // Frame number 
  case PS_CUSS_FTOAV: // Frames to average 
  case PS_CUSS_SAFMT: // Save-as format
  case PS_CUSS_FFCOR: // flat field correction on/off
  case PS_CUSS_DFCOR: // dark field correction on/off 
  case PS_CUSS_MACOR: // mask for corrections on/off
  case PS_CUSS_PCDFC: // preview colour dark field corr. on/off 
  case PS_CUSS_PMAST: // preview mask for stats. on/off 
  case PS_CUSS_PMASH: // preview mask show on/off 
  case PS_CUSS_PHSSH: // preview histogram show on/off 
  case PS_CUSS_PHSCR: // preview histogram cumulative Red/Y on/off
  case PS_CUSS_PHSCG: // preview histogram cumulative Green on/off
  case PS_CUSS_PHSCB: // preview histogram cumulative blue on/off
  case PS_CUSS_PHSDF: // preview histogram don't auto-fit on/off
  case PS_CUSS_PHSVM: // preview histogram restrict var/mn on/off
  case PS_CUSS_PHSSF: // preview histogram scale factors
  case PS_CUSS_PHSSL: // preview histogram saturation lower limits
  case PS_CUSS_PHSSU: // preview histogram saturation upper limits
  case PS_CUSS_PFMAL: // Focus assist - mean abs. Laplacian on/off
  case PS_CUSS_PFVAR: // Focus assist - var x other(s) on/off
  case PS_CUSS_PMONO: // Preview in monochrome on/off
  case PS_CUSS_PINVT: // Preview invert colours on/off
  case PS_CUSS_PFLPH: // Preview flip horizontal on/off
  case PS_CUSS_PFLPV: // Preview flip vertical on/off
  case PS_CUSS_PLAPL: // Preview display Laplacian on/off
  case PS_CUSS_PVLUT: // Preview LUT
  case PS_CUSS_PHSXY: // Preview histogram position XY
  case PS_CUSS_PFCPY: // Preview focus assist position Y
  case PS_CUSS_PMCEJ: // Preview mono correction images eject
  case PS_CUSS_PMINT: // Preview mono integral number
  case PS_CUSS_PMBIA: // Preview mono bias number
  case PS_CUSS_AVSCA: // Scale each image to first (averaging)on/off
  case PS_CUSS_GBTMO: // Grabber timeout in seconds
  case PS_CUSS_GBRTY: // Grabber retries before failure
  case PS_CUSS_GBDLY: // Grabber pre-delay for first image (s)
  case PS_CUSS_SADOU: // Save as raw doubles raw/fits/off
  case PS_CUSS_PFCSH: // Preview focus assist show mode
  case PS_CUSS_GPVST: // Get prieview stats off/on
  case PS_CUSS_GPFOC: // Get prieview focus value
  case PS_CUSS_TCTHR: // Set tissue content thresholds
  case PS_CUSS_GTCVL: // Get tissue content value off/on
  case PS_CUSS_PFDCT: // Focus parameter DCT on/off
  case PS_CUSS_PREMDN:  // Preview pre-Laplacian median on/off
  case PS_CUSS_POSTMDN: // Preview post-Laplacian median on/off
  case PS_CUSS_DCTAL:   // Calc. DCT after LUT on/off
  case PS_CUSS_PVSZ:    // Preview size
  break;
  default:
   rv=PS_CB_E_SID; // Currently no other PS_CUSS_ subtype is supported
  break;
 }
 return rv;
}

int32_t ps_check_img_type(int8_t)
// Test is the supplied int8_t is a valid image type.
// Return PS_CB_OK if it is, PS_CB_E_SID otherwise.
{
 int rv=PS_CB_OK;

    switch(ps_Instruction.siid){
     case PS_IMGT_MAINF: // Main image full size (i.e. capture an image/series) 
     case PS_IMGT_FFIMF: // Flat field image full size 
     case PS_IMGT_FFIMP: // Flat field image for mono preview
     case PS_IMGT_DFIMF: // Darkfield image full size
     case PS_IMGT_DFIMP: // Darkfield image for mono preview
     case PS_IMGT_DFCDP: // Darkfield image for colour preview
     case PS_IMGT_MASKF: // Corrections mask full size
     case PS_IMGT_MASKP: // Mask for preview image
     case PS_IMGT_PREVW: // Preview image at 640x480
     break;
     default:
      rv=PS_CB_E_SID; // Currently no other IMGR subtype is supported
     break;
    }
 return rv;
}

int32_t ps_check_img_fmt(int8_t fmt)
// Test is the supplied int8_t is a valid image format type.
// Return PS_CB_OK if it is, PS_CB_E_SIF otherwise.
{
 int rv=PS_CB_OK;

    switch(fmt){
     case SAF_YUYV: // Raw YUYV 16 bpp array
     case SAF_YP5 : // Y-only (greyscale) 8bpp pgm p5 format
     case SAF_BM8 : // Y-only (greyscale) 8bpp bmp format
     case SAF_INT : // Insensity from RGB/YUYV raw doubles monochrome
     case SAF_RGB : // Full colour RGB array
     case SAF_BMP : // Full colour BGR array
     case SAF_PNG : // Full colour PNG file
     case SAF_JPG : // Full colour JPEG file
     case SAF_DOU : // Raw doubles mono
     case SAF_DO3 : // Raw doubles colour
     break;
     default:
      rv=PS_CB_E_SIF; // Currently no other image format is supported
     break;
    }
 return rv;
}

int32_t ps_check_blob_from_server(void)
// Read the current report info blob received from a PARDcap server and check
// for format errors. As we do this, populate the ps_Instruction structure with
// the information read so we don't need to do it again if the blob checks out
// OK and we go on to actioning it.
// Return 1 if an error is found.
// Return 0 on success.
{
 int32_t idx,rv,npixels,si32;
 unsigned char *cptr;
 uint32_t ui32;
 uint8_t ui8; 

 // Set error markers to the 'no error' value
 rv=PS_CB_OK;
 // Reset the ps_Instruction struct to the unused state
 ps_ins_reset();

 // Check if we have sufficient blob to cover the basic intro data
 if(ps_BlobSize<5) rv=PS_CB_E_BSZ;
 if(rv) goto end_ps_check_blob_from_server;

 // Check if it is a compatible blob format version
 idx=0;
 switch(ps_Blob[idx]){
   case 0: break; // v.0 is supported
   default:
   rv=PS_CB_E_VER; // Currently no other version is supported
   break;
  }
 if(rv) goto end_ps_check_blob_from_server;

 // Check instruction type
 if((rv=ps_increment_blobindex(&idx,1))) goto end_ps_check_blob_from_server;
 cptr=ps_Blob+idx;
 ui32=ps_uc_to_uint32(cptr); ps_ins_set(PS_INS_CIID,&ui32,NULL);
 switch(ps_Instruction.ciid){
   case PS_CIID_IMGR: // Retreive an image
    // If a report has this CIID it means either an image is present in the blob
    // or, in the case of siid=PS_IMGT_MAINF, maybe not. That would not be an
    // error but a consequence of a deliberate decision not to save the image
    // due to its tissue content factor being below the currently set threshold.
    // In all cases - whether an image blob is present or not - the report will
    // contain data items for image format, height and width, so read them in:

    // Check validity of the sub-instruction (instruction sub-type) for an IMGR
    if((rv=ps_increment_blobindex(&idx,4))) break;
    ui8=ps_Blob[idx]; ps_ins_set(PS_INS_SIID,&ui8,NULL);
    if((rv=ps_check_img_type(ps_Instruction.siid)!=PS_CB_OK)) break; 
    switch(ps_Instruction.siid){
     case PS_IMGT_MAINF: // Main image full size (i.e. capture an image/series)
      // Read the header data items:
      //Image format
      if((rv=ps_increment_blobindex(&idx,1))) break;
      ui8=ps_Blob[idx]; ps_ins_set(PS_INS_SAFMT,&ui8,NULL);
      if((rv=ps_check_img_fmt(ps_Instruction.safmt))) break;    
      // Image height
      if((rv=ps_increment_blobindex(&idx,1))) break;
      cptr=ps_Blob+idx;
      si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_IVAL1,&si32,NULL);
      // Image width
      if((rv=ps_increment_blobindex(&idx,4))) break;
      cptr=ps_Blob+idx;
      si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_IVAL2,&si32,NULL);
      // dval1 is never used here so now read the error code err[0]
      if((rv=ps_increment_blobindex(&idx,4))) break;
      cptr=ps_Blob+idx;
      si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_ERR0,&si32,NULL);
      // The only time a string and blob are sent is if an image was actually
      // saved and this will be if ps_Instruction.err[0]==PS_CB_OK or
      // PS_CB_E_NBC, so check it first.
      if(ps_Instruction.err[0]==PS_CB_OK || ps_Instruction.err[0]==PS_CB_E_NBC){
        // Image name string and its length (in bytes)
        if((rv=ps_increment_blobindex(&idx,16))) break;
        cptr=ps_Blob+idx;
        si32=ps_uc_to_int32(cptr); // Put string length into si32
        // Read string if length is >4. Smaller string names indicate a file
        // name format error because you need at least 5 chars for a file name
        // plus a 3 letter extension and that is what the program requires for
        // images that are to be set.
        if(si32>4){
           // Set pointer to first char in string
           if((rv=ps_increment_blobindex(&idx,4))) break;
           cptr=ps_Blob+idx;
           // Check there is enough blob to contain string
           if((rv=ps_increment_blobindex(&idx,si32))) break;
           // Read string blob into instruction string
           if((rv=ps_ins_set(PS_INS_STR,&si32,cptr))) break;
         } else { rv=PS_CB_E_NOM; break;}
         // Image blob length (in bytes)
         cptr=ps_Blob+idx;
         si32=ps_uc_to_int32(cptr); // Put blob length into si32
         // There must be a non-zero blob here:
         if(si32<=0){rv=PS_CB_E_BSZ; break;}
         // Check the supplied blob length is compatible with the image save as
         // format.
         npixels=ps_Instruction.ival1*ps_Instruction.ival2;
         switch(ps_Instruction.safmt){
           case SAF_YUYV: // YUYV-raw format
             if(si32 != (2*npixels)) rv=PS_CB_E_BSZ;
           break;
           case SAF_BM8: // Y-only (greyscale) 8bpp bmp format
           case SAF_YP5: // Y-only (greyscale) 8bpp pgm p5 format
             if(si32 != npixels) rv=PS_CB_E_BSZ;
           break;
           case SAF_BMP: // Full colour RGB array saved in 24bpp BMP format
           case SAF_RGB: // Full colour RGB array saved in PPM p6 format
             if(si32 != (3*npixels) ) rv=PS_CB_E_BSZ;
           break;
           case SAF_INT: // Insensity from col. raw doubles (1 channel) '_i.dou'
           case SAF_DOU: // Raw doubles array (1 channel) '_y.dou'
             if(si32 != (sizeof(double)*npixels) ) rv=PS_CB_E_BSZ;
           break;
           case SAF_DO3: // Raw doubles arrays (3 channels) '_r/g/b.dou'
             if(si32 != (sizeof(double)*3*npixels) ) rv=PS_CB_E_BSZ;
           break;
           case SAF_JPG: // JPEG - can't check, just pass it
           break;
           default:
            rv=PS_CB_E_NOM;
           break;
          }
         if(rv!=PS_CB_OK) break;
         if((rv=ps_increment_blobindex(&idx,4))) break;
         // Read in the image blob
         // Set pointer to first char in image blob
         cptr=ps_Blob+idx;
         // Check there is enough blob to contain image
         if((rv=ps_increment_blobindex(&idx,si32-1))) break;
         // Read the image blob into instruction blob
         if((rv=ps_ins_set(PS_INS_BLOB,&si32,cptr))) break;
       }    

     break;
     case PS_IMGT_PREVW:
      // Read the header data items:
      //Image format (should be SAF_RGB for a preview image0
      if((rv=ps_increment_blobindex(&idx,1))) break;
      ui8=ps_Blob[idx]; ps_ins_set(PS_INS_SAFMT,&ui8,NULL);
      if(ps_Instruction.safmt!=SAF_RGB){rv=PS_CB_E_SIF; break;}    
      // Image height
      if((rv=ps_increment_blobindex(&idx,1))) break;
      cptr=ps_Blob+idx;
      si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_IVAL1,&si32,NULL);
      PreviewHt=si32;
      // Image width
      if((rv=ps_increment_blobindex(&idx,4))) break;
      cptr=ps_Blob+idx;
      si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_IVAL2,&si32,NULL);
      PreviewWd=si32;
      // dval1 is never used here so now read the error code err[0]
      // Read err[0] - must be PS_CB_OK here else there is a bug:
      if((rv=ps_increment_blobindex(&idx,4))) break;
      cptr=ps_Blob+idx;
      si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_ERR0,&si32,NULL);
      if(ps_Instruction.err[0]!=PS_CB_OK){rv=PS_CB_E_NOS; break;}
      // There is no str for a file name so go straight to image blob:
      // Image blob length (in bytes)
      if((rv=ps_increment_blobindex(&idx,16))) break;
      cptr=ps_Blob+idx;
      si32=ps_uc_to_int32(cptr); // Put blob length into si32
      // There must be a non-zero blob here:
      if(si32<=0){rv=PS_CB_E_BSZ; break;}
      // Check the supplied blob length is compatible with the image save as
      // format (SAF_RGB).
      npixels=ps_Instruction.ival1*ps_Instruction.ival2;
      if(si32 != (3*npixels) ){ rv=PS_CB_E_BSZ; break;}
      if((rv=ps_increment_blobindex(&idx,4))) break;
      // Read in the image blob
      // Set pointer to first char in image blob
      cptr=ps_Blob+idx;
      // Check there is enough blob to contain image
      if((rv=ps_increment_blobindex(&idx,si32-1))) break;
      // Read the image blob into instruction blob
      if((rv=ps_ins_set(PS_INS_BLOB,&si32,cptr))) break;
     break;
     default: // This shouldn't happen.
         rv=PS_CB_E_SID;
     break;
    }

   break;
   case PS_CIID_IMGS: // Set an image
    // The report will be a duplicate of the instruction sent but with the image
    // blob removed. In a way therefore there is no need to read it beyond the
    // CIID but we can check for comms corruption by reading and checking the
    // other items.
    // Check validity of the sub-instruction for the IMGS CIID
    if((rv=ps_increment_blobindex(&idx,4))) break;
    ui8=ps_Blob[idx]; ps_ins_set(PS_INS_SIID,&ui8,NULL);
/*
    if((rv=ps_check_img_type(ps_Instruction.siid))) break; 
    //Image format
    if((rv=ps_increment_blobindex(&idx,1))) break;
    ui8=ps_Blob[idx]; ps_ins_set(PS_INS_SAFMT,&ui8,NULL);
    if((rv=ps_check_img_fmt(ps_Instruction.safmt))) break;    
    // Image height
    if((rv=ps_increment_blobindex(&idx,1))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_IVAL1,&si32,NULL);
    // Image width
    if((rv=ps_increment_blobindex(&idx,4))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_IVAL2,&si32,NULL);
    // Read err[0] - must be PS_CB_OK here else there is a bug:
    if((rv=ps_increment_blobindex(&idx,4))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_ERR0,&si32,NULL);
    if((rv=ps_increment_blobindex(&idx,16))) break;
    // Image name string and its length (in bytes)
    if(ps_Instruction.err[0]!=PS_CB_OK){rv=PS_CB_E_NOS; break;}
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); // Put string length into si32
    // Read string if length is >4. Smaller string names indicate a file name
    // format error because you need at least 5 chars for a file name plus a 3
    // letter extension and that is what the program requires for images that
    // are to be set.
    if(si32>4){
       // Set pointer to first char in string
       if((rv=ps_increment_blobindex(&idx,4))) break;
       cptr=ps_Blob+idx;
       // Check there is enough blob to contain string
       if((rv=ps_increment_blobindex(&idx,si32))) break;
       // Read string blob into instruction string
       if((rv=ps_ins_set(PS_INS_STR,&si32,cptr))) break;
     } else { rv=PS_CB_E_NOM; break;}
*/
   break;
   case PS_CIID_CAMS: // Set a camera setting
   case PS_CIID_SRES: // Set the resolution for main image capture
    // Camera setting ID / Height
    if((rv=ps_increment_blobindex(&idx,4))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_IVAL1,&si32,NULL);
    // Value to set it to / Width
    if((rv=ps_increment_blobindex(&idx,4))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_IVAL2,&si32,NULL);
    // Read err[0] - must be PS_CB_OK here else there is a bug:
    if((rv=ps_increment_blobindex(&idx,4))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_ERR0,&si32,NULL);
    if(ps_Instruction.err[0]!=PS_CB_OK){rv=PS_CB_E_NOS; break;}
   break;
   case PS_CIID_CAMG: // Get a camera setting
    // Camera setting ID
    if((rv=ps_increment_blobindex(&idx,4))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_IVAL1,&si32,NULL);
    // Read err[0] - must be PS_CB_OK here else there is a bug:
    if((rv=ps_increment_blobindex(&idx,4))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_ERR0,&si32,NULL);
    if(ps_Instruction.err[0]!=PS_CB_OK){rv=PS_CB_E_NOS; break;}
      // Read the setting blob
    // Get past the error array
    if((rv=ps_increment_blobindex(&idx,16))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); // Put blob length into si32
    // Check we have the correct blob length:
    if(si32!=CAMGETBYTES){rv=PS_CB_E_BSZ; break;}
    if((rv=ps_increment_blobindex(&idx,4))) break;
    // Read in the setting value blob
    // Set pointer to first char in the blob
    cptr=ps_Blob+idx;
    // Check there is enough blob to contain the value
    if((rv=ps_increment_blobindex(&idx,si32-1))) break;
    // Read the setting blob into the instruction blob
    if((rv=ps_ins_set(PS_INS_BLOB,&si32,cptr))) break;
   break;
   case PS_CIID_CUSS: // Set a custom (non-driver) frame grabber setting
    // Check validity of the sub-instruction for the CUSS CIID
    if((rv=ps_increment_blobindex(&idx,4))) break;
    ui8=ps_Blob[idx]; ps_ins_set(PS_INS_SIID,&ui8,NULL);
    if((rv=ps_check_cuss_type(ps_Instruction.siid))) break; 
    // Read err[0] - must be PS_CB_OK here else there is a bug:
    if((rv=ps_increment_blobindex(&idx,1))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_ERR0,&si32,NULL);
    if(ps_Instruction.err[0]!=PS_CB_OK){rv=PS_CB_E_NOS; break;}
    // Now any additional data as per SIID
    switch(ui8){
      case PS_CUSS_GPVST: // Get prieview stats off/on
        // Read the stats blob
      // Get past the error array
      if((rv=ps_increment_blobindex(&idx,16))) break;
      cptr=ps_Blob+idx;
      si32=ps_uc_to_int32(cptr); // Put blob length into si32
      // Check we have the correct blob length:
      if(si32!=STATSBYTES){rv=PS_CB_E_BSZ; break;}
      if((rv=ps_increment_blobindex(&idx,4))) break;
      // Read in the stats blob
      // Set pointer to first char in the blob
      cptr=ps_Blob+idx;
      // Check there is enough blob to contain the stats
      if((rv=ps_increment_blobindex(&idx,si32-1))) break;
      // Read the stats blob into the instruction blob
      if((rv=ps_ins_set(PS_INS_BLOB,&si32,cptr))) break;

      break;
      case PS_CUSS_GPFOC: // Get prieview focus value
        // Read the focus blob
      // Get past the error array
      if((rv=ps_increment_blobindex(&idx,16))) break;
      cptr=ps_Blob+idx;
      si32=ps_uc_to_int32(cptr); // Put blob length into si32
      // Check we have the correct blob length:
      if(si32!=FOCUSBYTES){rv=PS_CB_E_BSZ; break;}
      if((rv=ps_increment_blobindex(&idx,4))) break;
      // Read in the focus blob
      // Set pointer to first char in the blob
      cptr=ps_Blob+idx;
      // Check there is enough blob to contain the focus value
      if((rv=ps_increment_blobindex(&idx,si32-1))) break;
      // Read the focus blob into the instruction blob
      if((rv=ps_ins_set(PS_INS_BLOB,&si32,cptr))) break;

      break;
      case PS_CUSS_GTCVL: // Get tissue content off/on
        // Read the tc blob
      // Get past the error array
      if((rv=ps_increment_blobindex(&idx,16))) break;
      cptr=ps_Blob+idx;
      si32=ps_uc_to_int32(cptr); // Put blob length into si32
      // Check we have the correct blob length:
      if(si32!=TCBYTES){rv=PS_CB_E_BSZ; break;}
      if((rv=ps_increment_blobindex(&idx,4))) break;
      // Read in the tc blob
      // Set pointer to first char in the blob
      cptr=ps_Blob+idx;
      // Check there is enough blob to contain the tc value
      if((rv=ps_increment_blobindex(&idx,si32-1))) break;
      // Read the tc blob into the instruction blob
      if((rv=ps_ins_set(PS_INS_BLOB,&si32,cptr))) break;

      break;
      default:
      break;
     }  
   break;
   case PS_CIID_LRES: // List resolutions (and frame rates) camera can do
    // Read err[0] - must be PS_CB_OK here else there is a bug:
    if((rv=ps_increment_blobindex(&idx,4))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_ERR0,&si32,NULL);
    if(ps_Instruction.err[0]!=PS_CB_OK){rv=PS_CB_E_NOS; break;}
    // List string and its length (in bytes)
    if((rv=ps_increment_blobindex(&idx,16))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); // Put string length into si32
    // Read string if length is >42. A smaller string indicate an error.
    if(si32>42){
       // Set pointer to first char in string
       if((rv=ps_increment_blobindex(&idx,4))) break;
       cptr=ps_Blob+idx;
       // Check there is enough blob to contain string
       if((rv=ps_increment_blobindex(&idx,si32-1))) break;
       // Read string blob into instruction string
       if((rv=ps_ins_set(PS_INS_STR,&si32,cptr))) break;
     } else {rv=PS_CB_E_NOM; break;}

   break;
   case PS_CIID_LSET: // List available camera settings
    // Read err[0] - must be PS_CB_OK here else there was an error getting the
    // settings:
    if((rv=ps_increment_blobindex(&idx,4))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_ERR0,&si32,NULL);
    if(ps_Instruction.err[0]!=PS_CB_OK){rv=PS_CB_E_NOS; break;}
    // List string and its length (in bytes)
    if((rv=ps_increment_blobindex(&idx,16))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); // Put string length into si32
    // Read string if length is >5. A smaller string indicates an error.
    if(si32>5){
       // Set pointer to first char in string
       if((rv=ps_increment_blobindex(&idx,4))) break;
       cptr=ps_Blob+idx;
       // Check there is enough blob to contain string
       if((rv=ps_increment_blobindex(&idx,si32-1))) break;
       // Read string blob into instruction string
       if((rv=ps_ins_set(PS_INS_STR,&si32,cptr))) break;
     } else {rv=PS_CB_E_NOM; break;}
   break; // No more info needed for the above CIIDs
   case PS_CIID_ERRC: // The instruction could not be done due to an error
   // An error-code-conveying CIID contains only 4 used fields after the CIID
   // and those are the err[0]-err[3]. If any of them is negative it means it
   // does not contain any error information (a negative value is just a filler)
    // Read err[0]:
    if((rv=ps_increment_blobindex(&idx,4))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_ERR0,&si32,NULL);
    // Read err[1]:
    if((rv=ps_increment_blobindex(&idx,4))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_ERR1,&si32,NULL);
    // Read err[2]:
    if((rv=ps_increment_blobindex(&idx,4))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_ERR2,&si32,NULL);
    // Read err[3]:
    if((rv=ps_increment_blobindex(&idx,4))) break;
    cptr=ps_Blob+idx;
    si32=ps_uc_to_int32(cptr); ps_ins_set(PS_INS_ERR3,&si32,NULL);
   break;
   default:
    rv=PS_CB_E_IID; // Currently no other activity is supported
   break;
  }

end_ps_check_blob_from_server:
 // Whether correctly formatted or not, we have finished with the ps_Blob
 // storage at this point, so free it:
 if(ps_resize_blob(2)){ // This is unlikely to fail but we check anyway.
  show_message(": Calloc failed on blob.",fnn,MT_ERR,0);
  } 
 // If there was an error, release any memory allocated to the instruction buffs
 if(rv!=PS_CB_OK){
   ps_Instruction.slen=0; ps_alloc_instr();   // This also sets the used flag=0
   ps_Instruction.blen=0; ps_alloc_insblob(); // This also sets the used flag=0 
  }

 return rv;
}

void ps_binval_to_uc(unsigned char *buf,void *bvptr, int binvalen)
// Copies the binvalen bytes of a binary value (pointed to by bvptr) into buf.
// The binary value is binvalen bytes long and buf is assumed to point to
// a memory location that has at least binvalen bytes of allocated memory.
// An example use could be to copy the 4 bytes of an int32_t integer into an
// unsigned char array but this function is data type agnostic - it could just
// as well be a signed int, a float a double, whatever.
{
int idx;
unsigned char byte;

for (idx = 0; idx < binvalen; idx++) {
  byte = *((unsigned char *)bvptr + idx);
  buf[idx]=byte;
}

 return;
}

int ps_ins_to_blob(unsigned char **blobuff, int *bloblen)
// Encapsulate the current ps_Instruction struct into a blob suitable for
// sending with the PARD capture protocol. Put the blob in blobuff after
// allocing it sufficient space.
// Return 0 on success and non-zero on failure (memory alloc failure).
{
 int32_t blen,idx,rv;
 char infostr[320];

 rv=PS_CB_OK;

 // Calculate the length (number of bytes) of the required encapsulated blob.
 // Start with 9 because this is the size of the int32_t ('number of bytes to
 // follow') header int (4 bytes) of the capsule, plus the 1 uint8_t byte used
 // for the version number of the info blob inside the capsule plus the 4 bytes
 // of the mandatory ciid field (uint32_t) of the ps_Instruction:
 blen = 9;

 // Now increment it by the appropriate number of bytes for each used member of
 // the ps_Instruction struct
 if(ps_Instruction.used.siid)  blen++;  // (uint8_t)
 if(ps_Instruction.used.safmt) blen++;  // (uint8_t)
 if(ps_Instruction.used.ival1) blen+=4; // (int32_t)
 if(ps_Instruction.used.ival2) blen+=4; // (int32_t)
 if(ps_Instruction.used.ival3) blen+=4; // (int32_t)
 if(ps_Instruction.used.dval1) blen+=sizeof(double); // (double)
 if(ps_Instruction.used.dval2) blen+=sizeof(double); // (double)
 if(ps_Instruction.used.dval3) blen+=sizeof(double); // (double)
 if(ps_Instruction.used.err) blen+=16; // (int32_t[4])
 // If the string or blob are used, make sure to add the 4 bytes for the
 // corresponding slen or blen members as well as the bytes in the string/blob
 if(ps_Instruction.used.str) blen+=(4+ps_Instruction.slen);
 if(ps_Instruction.used.blob) blen+=(4+ps_Instruction.blen);

 // Now allocate memory to the transport (encapsulated) blob
 *blobuff=calloc(blen,sizeof(**blobuff));
 if(*blobuff==NULL){
   sprintf(infostr,": Calloc failed on send buffer for %"PRId32" bytes.",blen);
   show_message(infostr,fnn,MT_ERR,0);
   rv=PS_CB_E_MEE; return 1;
  } 

 // Put the datastream 'to follow' size in the buffer first (datastream 'to 
 // follow' size = number of bytes to follow after the first 4 bytes of the
 // whole encapsulated blob (of size blen) are read. Hence we do blen-=4 first:
 idx=0;blen-=4;
 ps_binval_to_uc((*blobuff)+idx,&blen,4);
 blen+=4; // Restore blen so we can use it for a checksum at the end
 idx+=4;  // Increment the current position in blobuff by the number of bytes
          // we just wrote.

 // Now add the blob format version byte (currently this has value 0)
 (*blobuff)[idx]=0;
 idx++; // Incr. the curr. pos. in blobuff by the No. of bytes we just wrote

 // Now the mandatory uint32_t ciid value:
 ps_binval_to_uc((*blobuff)+idx,&(ps_Instruction.ciid),4);
 idx+=4;// Incr. the curr. pos. in blobuff by the No. of bytes we just wrote

 // Now add the optional values of the ps_Instruction struct that are used:
 if(ps_Instruction.used.siid){
   (*blobuff)[idx]=ps_Instruction.siid;
   idx++;  // (uint8_t)
  }
 if(ps_Instruction.used.safmt){
   (*blobuff)[idx]=ps_Instruction.safmt;
   idx++;  // (uint8_t)
  }
 if(ps_Instruction.used.ival1){
   ps_binval_to_uc((*blobuff)+idx,&(ps_Instruction.ival1),4);
   idx+=4; // (int32_t)
  }
 if(ps_Instruction.used.ival2){
   ps_binval_to_uc((*blobuff)+idx,&(ps_Instruction.ival2),4);
   idx+=4; // (int32_t)
  }
 if(ps_Instruction.used.ival3){
   ps_binval_to_uc((*blobuff)+idx,&(ps_Instruction.ival3),4);
   idx+=4; // (int32_t)
  }
 if(ps_Instruction.used.dval1){
   ps_binval_to_uc((*blobuff)+idx,&(ps_Instruction.dval1),sizeof(double));
   idx+=sizeof(double); // (double)
  }
 if(ps_Instruction.used.dval2){
   ps_binval_to_uc((*blobuff)+idx,&(ps_Instruction.dval2),sizeof(double));
   idx+=sizeof(double); // (double)
  }
 if(ps_Instruction.used.dval3){
   ps_binval_to_uc((*blobuff)+idx,&(ps_Instruction.dval3),sizeof(double));
   idx+=sizeof(double); // (double)
  }
 if(ps_Instruction.used.err){
   ps_binval_to_uc((*blobuff)+idx,(ps_Instruction.err),16);
   idx+=16; // (int32_t[4])
  }
 // If the string or blob are used, make sure to add the 4 bytes for the
 // corresponding slen or blen members as well as the bytes in the string/blob
 if(ps_Instruction.used.str){
   ps_binval_to_uc((*blobuff)+idx,&(ps_Instruction.slen),4);
   idx+=4;
   ps_binval_to_uc((*blobuff)+idx,(ps_Instruction.str),ps_Instruction.slen);
   idx+=ps_Instruction.slen;
  }
 if(ps_Instruction.used.blob){
   ps_binval_to_uc((*blobuff)+idx,&(ps_Instruction.blen),4);
   idx+=4;
   ps_binval_to_uc((*blobuff)+idx,(ps_Instruction.blob),ps_Instruction.blen);
   idx+=ps_Instruction.blen;
  }

 // Do a quick checksum in case we messed up
 if(idx != blen){
   sprintf(infostr,": Checksum error: %"PRId32" vs %"PRId32"\n",idx,blen);
   show_message(infostr,fnn,MT_ERR,0);
   rv=PS_CB_E_CSE; goto end_err_ps_ins_to_blob;
  }

 // Set the blob length:
 *bloblen=(int)blen;

 return rv;

end_err_ps_ins_to_blob:
 free(*blobuff); *blobuff=NULL;
 return rv;
}

int ps_ins_send(void)
// Encapsulate the ps_Instruction into a blob and send it to the PARDcap server.
// Return 0 on success
// Return 1 on error
{
 int bloblen,rv;
 unsigned char *blobuff;

 rv=0;

 blobuff=NULL;bloblen=0;
 if(ps_ins_to_blob(&blobuff,&bloblen)){rv=1; goto end_ps_ins_send;}
 if (ps_send_dc(ps_Comms_fd,(char *)blobuff, &bloblen) == -1) rv=1;

end_ps_ins_send:
 if(blobuff!=NULL) free(blobuff);
 return rv;
}

//---------------------------------------------------------------------------//.
 //                         PARDCapture server comms                        //.
  //-----------------------------------------------------------------------//.
#endif

  //-----------------------------------------------------------------------\\.
 //                           Arduino serial comms                          \\.
//---------------------------------------------------------------------------\\.
#ifdef PARDUS

#if IS_WINDOWS_NATIVE
int as_win_try_port(const char *port)
// Opens and configures one COM port and performs the PARD handshake on it,
// with the same single retry after a 3-second boot wait as the Linux code.
// Returns 0 on success (as_hCom then holds the open port) and 1 on failure,
// in which case the port has been closed again (a COM port is held
// exclusively, so a port that is not the Arduino must be released at once).
// MS Windows details (see as_open_comms_with_Arduino for the overview):
//  * The port is opened with CreateFileA() on its Win32 device path
//    ("COM5" -> "\\.\COM5") for exclusive use, as Windows requires.
//  * The termios settings (B500000 and cfmakeraw) become a DCB: 500000 baud,
//    8 data bits, no parity, 1 stop bit, binary, no flow control, no special
//    characters and NUL bytes passed through. O_NONBLOCK becomes COMMTIMEOUTS
//    which make ReadFile() return at once with whatever bytes have arrived.
//  * The Linux ASYNC_LOW_LATENCY request has no Win32 counterpart (for FTDI
//    adapters the latency timer is a driver setting - see the manual).
//  * On the retry after a failed handshake the port, already open and
//    configured, is re-used: Windows would refuse a second open of a COM port,
//    and keeping it open keeps DTR asserted so that the Arduino is not reset a
//    second time.
{
	DCB dcb;                       // Device control block (cf. struct termios)
	COMMTIMEOUTS timeouts;         // Read/write time-outs (cf. O_NONBLOCK)
	unsigned char bytes_received[128];
	char bytes_sent[32];
 int retval=0,receptval;
 char infostr[320];
 char devpath[300];             // Win32 device path, e.g. "\\.\COM12"
 int bytecount;
 uint8_t attempt_number = 0;

 as_hCom = INVALID_HANDLE_VALUE;
 as_win_device_path(port, devpath, sizeof(devpath));
 as_hCom = CreateFileA(devpath, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                                                         OPEN_EXISTING, 0, NULL);
 if(as_hCom == INVALID_HANDLE_VALUE){
   sprintf(infostr,": Failed to open %.40s: %s",port,win_errstr(GetLastError()));
   show_message(infostr,fnn,MT_ERR,0);
   retval=1;
   goto end_as_win_try_port;
  }

 // Get the current serial port configuration into the DCB structure (the
 // counterpart of tcgetattr). Unlike tcgetattr on Linux this can fail if the
 // name given was not that of a serial port, so check it.
 memset(&dcb, 0, sizeof(dcb));
 dcb.DCBlength = sizeof(dcb);
 if(!GetCommState(as_hCom, &dcb)){
    sprintf(infostr,": Failed to read serial port attributes: %s",
                                                     win_errstr(GetLastError()));
    show_message(infostr,fnn,MT_ERR,0);
    retval=1;
    goto end_as_win_try_port;
   }

 // Set baud rate for input and output (cf. cfsetispeed/cfsetospeed B500000)
 dcb.BaudRate = 500000;

 // Set 'raw' mode (cf. cfmakeraw): binary transfers, 8 data bits, no parity,
 // one stop bit, no software (XON/XOFF) or hardware (RTS/CTS, DTR/DSR) flow
 // control, no error-character replacement and NUL bytes are NOT discarded
 // (the PARD serial protocol is binary).
 dcb.fBinary           = TRUE;
 dcb.fParity           = FALSE;
 dcb.Parity            = NOPARITY;
 dcb.ByteSize          = 8;
 dcb.StopBits          = ONESTOPBIT;
 dcb.fOutxCtsFlow      = FALSE;
 dcb.fOutxDsrFlow      = FALSE;
 dcb.fDsrSensitivity   = FALSE;
 dcb.fTXContinueOnXoff = TRUE;
 dcb.fOutX             = FALSE;
 dcb.fInX              = FALSE;
 dcb.fErrorChar        = FALSE;
 dcb.fNull             = FALSE;
 dcb.fAbortOnError     = FALSE;
 // DTR and RTS are asserted, as Linux does when it opens a tty. (Asserting DTR
 // resets the Arduino through its auto-reset circuit, just as on Linux.)
 dcb.fDtrControl       = DTR_CONTROL_ENABLE;
 dcb.fRtsControl       = RTS_CONTROL_ENABLE;

 // Driver buffer sizes (advisory), then apply the customised configuration
 // after discarding any data in the input and output buffers (cf. TCSAFLUSH):
 SetupComm(as_hCom, 4096, 4096);
 PurgeComm(as_hCom, PURGE_RXCLEAR | PURGE_TXCLEAR);
	if (!SetCommState(as_hCom, &dcb)){
    show_message(": Failed to implement serial port attributes.",
                                        fnn,MT_ERR,0);
    retval=1;
    goto end_as_win_try_port;
   };

 // Non-blocking reads (cf. O_NONBLOCK): ReadFile() returns immediately with
 // the bytes already received, even if there are none. A write gives up after
 // as_portTimeout ms (then fewer bytes are reported, which the byte-count
 // checks detect, as for a short non-blocking write() on Linux).
 timeouts.ReadIntervalTimeout         = MAXDWORD;
 timeouts.ReadTotalTimeoutMultiplier  = 0;
 timeouts.ReadTotalTimeoutConstant    = 0;
 timeouts.WriteTotalTimeoutMultiplier = 0;
 timeouts.WriteTotalTimeoutConstant   = (DWORD)as_portTimeout;
 if(!SetCommTimeouts(as_hCom, &timeouts)){
    show_message(": Failed to implement serial port time-outs.",fnn,MT_ERR,0);
    retval=1;
    goto end_as_win_try_port;
   }

 handshake:
 // If we failed once, it may be that the Arduino reset is blocking us so try
 // once more with a 3 second delay ...
 if(attempt_number){
   // Loop to handle the 3-second animated countdown
   for (int seconds_left = 3; seconds_left > 0; seconds_left--) {
       // Construct the message text string manually
       char countdown_str[128];
       snprintf(countdown_str, sizeof(countdown_str),
                ": Waiting %d seconds for Arduino to boot... ", seconds_left);

       show_message(countdown_str, fnn, MT_INFO, 0);

       // Force the terminal console buffer to flush the text stream immediately
       fflush(stdout);

       // Wait exactly 1 second before looping to the next number
       Sleep(1000);
   }

   // Clean up the terminal line context right before proceeding to try again
   char clear_str[128];
   snprintf(clear_str, sizeof(clear_str),
                            ": Arduino boot wait completed.                  \n");
   show_message(clear_str, fnn, MT_INFO, 0);
 }


	// Send acknowledge command to Arduino

 sprintf(bytes_sent,"PARDack");
 sprintf(infostr,": Sending: %s",bytes_sent);
 show_message(infostr,fnn,MT_INFO,0);
	// Clear the input buffer to ensure we only see the reply to THIS command
 PurgeComm(as_hCom, PURGE_RXCLEAR);                   // cf. tcflush(TCIFLUSH)
	bytecount=as_win_write(bytes_sent, 7);
 if(bytecount!=7){
   show_message(": Failed to send PARDack initiation signal. Quitting.",
                                        fnn,MT_ERR,0);
   retval=1;
   goto end_as_win_try_port;
  }
 FlushFileBuffers(as_hCom); // Wait until all output has been sent (cf. tcdrain)
 msleep(20); // Safe 20ms delay for Arduino to process and reply

 // Listen for the expected reply

 receptval=as_read_sigstr((char *)bytes_received, "PARD_OK");
 if(receptval==0) show_message(": Comms established.\n",fnn,MT_INFO,0);
   else {
         sprintf(infostr,": Received wrong number (%s) of bytes. Quitting.\n",
                                                                bytes_received);
         show_message(infostr,fnn,MT_ERR,0);
         retval = 1;
        }

 if(retval && attempt_number == 0){
    show_message(": Trying once more but with a 3 second delay ...\n",
                                                                 fnn,MT_INFO,0);
    attempt_number = 1;
    retval=0;
    goto handshake; // the port stays open and configured (see above)
  }

end_as_win_try_port:
 // A COM port is held exclusively, so release a port that could not be brought
 // into service.
 if(retval && as_hCom != INVALID_HANDLE_VALUE){
   CloseHandle(as_hCom);
   as_hCom = INVALID_HANDLE_VALUE;
  }
 return retval;
}

int as_open_comms_with_Arduino(char *arduinopath)
// Open and configure the serial port and send a handshake to the Arduino if it
// can be successfully found on arduinopath. 'nobob' opens the NUL device (the
// counterpart of /dev/null). The 'auto' mode (arduinopath==NULL) searches the
// present COM ports for the Arduino.
// MS Windows version (revised 14.11.26):
//  * A named port (COM4, COM12, \\.\COM12 ...) is opened by as_win_try_port().
//  * 'auto': the Linux code tries /dev/ttyACM0, then /dev/ttyUSB0. Windows
//    numbers its COM ports freely, so the search lists every present COM port
//    of a USB device (as_win_list_candidates: SetupAPI, with the registry list
//    SERIALCOMM as a fall-back), in this order: boards with an Arduino USB
//    vendor ID, other USB CDC-ACM ports (cf. /dev/ttyACM*), USB-serial bridges
//    (CH34x, FTDI, CP210x, PL2303; cf. /dev/ttyUSB*), other USB serial ports -
//    each group by increasing COM number - and it performs the PARD handshake
//    on each in turn until one answers. Motherboard, Bluetooth, modem and
//    virtual ports are never tried. The list is printed, so that a failure
//    shows which ports were seen.
//    (The first version took only the lowest-numbered port of the classes
//    found in the registry list SERIALCOMM by driver name, and did not try a
//    second port when the first one did not answer. A board whose driver lists
//    its port under another device name, or a second USB serial device with a
//    lower COM number, therefore defeated it.)
{
 AsWinCandidate cand[AS_WIN_MAX_CANDIDATES];
 static const char *const rankname[] = {"Arduino", "USB CDC-ACM", "USB-serial bridge",
                                        "other USB serial"};
 char infostr[400];
 int ncand, idx;

	sprintf(infostr,": Attempting serial USB port communication with Arduino: %s",
                                         (arduinopath==NULL)?"...":arduinopath);
 show_message(infostr,fnn,MT_INFO,0);

 // Check for the 'no breakout board' option first
 if (arduinopath != NULL && strcmp(arduinopath, "nobob") == 0) {
     show_message(": ⚠️ Bypassing hardware breakout board.", fnn, MT_INFO, 0);

     // Open the null device (NUL) so as_hCom holds a valid handle for cleanup
     as_hCom = CreateFileA("NUL", GENERIC_READ | GENERIC_WRITE,
                           FILE_SHARE_READ | FILE_SHARE_WRITE, NULL,
                           OPEN_EXISTING, 0, NULL);
     if (as_hCom == INVALID_HANDLE_VALUE) {
         show_message(": Failed to open NUL device context.", fnn, MT_ERR, 0);
         return 1;
     }

     // Set connection status flags so the server knows it is initialized
     as_connected = 1;

     show_message(": Comms established (Virtual Loopback).\n", fnn, MT_INFO, 0);
     return 0; // Return success immediately and skip the rest of the function!
 }

 // A named port
 if(arduinopath != NULL) return as_win_try_port(arduinopath);

 // 'auto': list the candidate ports and try them in turn
 ncand = as_win_list_candidates(cand, AS_WIN_MAX_CANDIDATES);
 if(ncand == 0){
   show_message(": Failed: no COM port of a USB serial device is present"
                " (is the Arduino plugged in and its driver installed?).\n",
                                                                 fnn,MT_ERR,0);
   return 1;
  }
 snprintf(infostr,sizeof(infostr),": %d candidate port(s) found:",ncand);
 show_message(infostr,fnn,MT_INFO,0);
 for(idx = 0; idx < ncand; idx++){
    snprintf(infostr,sizeof(infostr),":   %-6s %-17s %s",cand[idx].port,
                                    rankname[cand[idx].rank],cand[idx].desc);
    show_message(infostr,fnn,MT_INFO,0);
   }
 for(idx = 0; idx < ncand; idx++){
    snprintf(infostr,sizeof(infostr),": Attempting coms on %s ...",cand[idx].port);
    show_message(infostr,fnn,MT_INFO,0);
    if(as_win_try_port(cand[idx].port) == 0){
      snprintf(infostr,sizeof(infostr),": Arduino found on %s.",cand[idx].port);
      show_message(infostr,fnn,MT_INFO,0);
      return 0;
     }
   }
 show_message(": Failed: none of the candidate ports answered the PARD handshake."
              " Name the port explicitly in the configuration file (e.g. COM4)"
              " if the Arduino is on one of them.\n",fnn,MT_ERR,0);
 return 1;
}
#elif defined(__APPLE__)
int as_mac_try_port(const char *devpath)
// Opens and configures one serial device and performs the PARD handshake on
// it, with the same single retry after a 3-second boot wait as the Linux code.
// Returns 0 on success (as_portName then holds the open port) and 1 on
// failure, in which case the device has been closed again.
// macOS details (see as_open_comms_with_Arduino for the overview):
//  * macOS termios has no B500000. The speed is set with the IOKit ioctl
//    IOSSIOSPEED (Apple's documented method for non-standard rates) after
//    tcsetattr(); should that fail, the numeric rate is passed to
//    cfsetispeed()/cfsetospeed() (macOS speed_t values are the baud rates
//    themselves) as a fall-back.
//  * CLOCAL and CREAD (ignore modem-control lines, enable the receiver) are
//    set explicitly: the Linux USB serial drivers set them by default, macOS
//    drivers do not necessarily do so.
//  * The Linux ASYNC_LOW_LATENCY flag (TIOCSSERIAL) does not exist. Its
//    counterpart is the IOKit ioctl IOSSDATALAT, which sets the receive
//    latency to 1 microsecond. As on Linux a failure is only a warning.
//  * On the retry after a failed handshake the open, configured port is
//    re-used (so DTR stays asserted and the Arduino is not reset again).
{
	struct termios termios_params;  // Terminal parameters for serial comms.
	unsigned char bytes_received[128];
	char bytes_sent[32];
 int retval=0,receptval;
 char infostr[320];
 size_t bytecount;
 speed_t baudrate = 500000;      // The PARD serial rate, for IOSSIOSPEED
 unsigned long latency_us = 1UL; // Receive latency (microseconds), IOSSDATALAT
 uint8_t attempt_number = 0;

 as_portName = open(devpath, O_RDWR | O_NONBLOCK);
 if(as_portName<0){
   sprintf(infostr,": Failed to open %.200s: %s",devpath,strerror(errno));
   show_message(infostr,fnn,MT_ERR,0);
   retval=1;
   goto end_as_mac_try_port;
  }

 // Get the current terminal configuration attributes into the termios_params
 // structure
	tcgetattr(as_portName, &termios_params);

 // Set 'raw' mode i.e. input is available character by character, echoing is
 // disabled, and all special processing of terminal input and output characters
 // is disabled.
 cfmakeraw(&termios_params);

 // Ignore the modem-control lines and enable the receiver (Linux USB serial
 // drivers have these set by default).
 termios_params.c_cflag |= (CLOCAL | CREAD);

 // Apply the customised terminal configuration parameters after flushing any
 // data on the input or output buffers:
	if (tcsetattr(as_portName, TCSAFLUSH, &termios_params)){
    show_message(": Failed to implement serial port attributes.",
                                        fnn,MT_ERR,0);
    retval=1;
    goto end_as_mac_try_port;
   };

 // Set baud rate for input and output: 500000 baud. B500000 does not exist on
 // macOS, so use the IOKit IOSSIOSPEED ioctl (it must follow tcsetattr, which
 // would otherwise reset the speed) and fall back on the numeric termios speed.
 if(ioctl(as_portName, IOSSIOSPEED, &baudrate) == -1){
   cfsetispeed(&termios_params, (speed_t)500000);
   cfsetospeed(&termios_params, (speed_t)500000);
   if(tcsetattr(as_portName, TCSANOW, &termios_params)){
     sprintf(infostr,": Failed to set the serial speed to 500000 baud: %s",
                                                               strerror(errno));
     show_message(infostr,fnn,MT_ERR,0);
     retval=1;
     goto end_as_mac_try_port;
    }
  }

 // Reduce the receive latency (the macOS counterpart of ASYNC_LOW_LATENCY):
 if(ioctl(as_portName, IOSSDATALAT, &latency_us) == -1){
   show_message(": Warning: Failed to set low latency mode.", fnn, MT_ERR, 0);
  }

 handshake:
 // If we failed once, it may be that the Arduino reset is blocking us so try
 // once more with a 3 second delay ...
 if(attempt_number){
   // Loop to handle the 3-second animated countdown
   for (int seconds_left = 3; seconds_left > 0; seconds_left--) {
       // Construct the message text string manually
       char countdown_str[128];
       snprintf(countdown_str, sizeof(countdown_str),
                ": Waiting %d seconds for Arduino to boot... ", seconds_left);

       show_message(countdown_str, fnn, MT_INFO, 0);

       // Force the terminal console buffer to flush the text stream immediately
       fflush(stdout);

       // Wait exactly 1 second before looping to the next number
       sleep(1);
   }

   // Clean up the terminal line context right before proceeding to try again
   char clear_str[128];
   snprintf(clear_str, sizeof(clear_str),
                            ": Arduino boot wait completed.                  \n");
   show_message(clear_str, fnn, MT_INFO, 0);
 }


	// Send acknowledge command to Arduino

 sprintf(bytes_sent,"PARDack");
 sprintf(infostr,": Sending: %s",bytes_sent);
 show_message(infostr,fnn,MT_INFO,0);
	// Clear the input buffer to ensure we only see the reply to THIS command
 tcflush(as_portName, TCIFLUSH);
	bytecount=write(as_portName, bytes_sent, 7);
 if(bytecount!=7){
   show_message(": Failed to send PARDack initiation signal. Quitting.",
                                        fnn,MT_ERR,0);
   retval=1;
   goto end_as_mac_try_port;
  }
 tcdrain(as_portName); // Wait until all output has been sent
 msleep(20); // Safe 20ms delay for Arduino to process and reply

 // Listen for the expected reply

 receptval=as_read_sigstr((char *)bytes_received, "PARD_OK");
 if(receptval==0) show_message(": Comms established.\n",fnn,MT_INFO,0);
   else {
         sprintf(infostr,": Received wrong number (%s) of bytes. Quitting.\n",
                                                                bytes_received);
         show_message(infostr,fnn,MT_ERR,0);
         retval = 1;
        }

 if(retval && attempt_number == 0){
    show_message(": Trying once more but with a 3 second delay ...\n",
                                                                 fnn,MT_INFO,0);
    attempt_number = 1;
    retval=0;
    goto handshake; // the port stays open and configured (see above)
  }

end_as_mac_try_port:
 // Release a device that could not be brought into service, so that the
 // 'auto' search can go on to the next candidate without leaking descriptors.
 if(retval && as_portName >= 0){
   close(as_portName);
   as_portName = -1;
  }
 return retval;
}

int as_open_comms_with_Arduino(char *arduinopath)
// Open and configure the serial port and send a handshake to the Arduino if it
// can be successfully found on arduinopath. 'nobob' opens /dev/null. The
// 'auto' mode (arduinopath==NULL) searches the USB serial devices.
// macOS version (revised 14.11.26):
//  * A named device is opened by as_mac_try_port().
//  * 'auto': macOS names USB CDC-ACM ports /dev/cu.usbmodem<id> (cf.
//    /dev/ttyACM0) and USB-serial bridges /dev/cu.usbserial-<id> (or
//    /dev/cu.wchusbserial<id>, /dev/cu.SLAB_USBtoUART with the vendors'
//    drivers; cf. /dev/ttyUSB0), where <id> depends on the USB socket. The
//    search lists all of them (as_mac_list_candidates), CDC-ACM devices first,
//    and performs the PARD handshake on each in turn until one answers. The
//    list is printed, so that a failure shows which devices were seen.
//    (The first version opened only the first device of each class and did
//    not try a second device when the first one did not answer.)
{
 AsMacCandidate cand[AS_MAC_MAX_CANDIDATES];
 char infostr[PATH_MAX + 80];
 int ncand, idx;

	sprintf(infostr,": Attempting serial USB port communication with Arduino: %s",
                                         (arduinopath==NULL)?"...":arduinopath);
 show_message(infostr,fnn,MT_INFO,0);

 // Check for the 'no breakout board' option first
 if (arduinopath != NULL && strcmp(arduinopath, "nobob") == 0) {
     show_message(": ⚠️ Bypassing hardware breakout board.", fnn, MT_INFO, 0);

     // Open to the null device so as_portName holds a valid file descriptor for cleanup
     as_portName = open("/dev/null", O_RDWR);
     if (as_portName < 0) {
         show_message(": Failed to open /dev/null context.", fnn, MT_ERR, 0);
         return 1;
     }

     // Set connection status flags so the server knows it is initialized
     as_connected = 1;

     show_message(": Comms established (Virtual Loopback).\n", fnn, MT_INFO, 0);
     return 0; // Return success immediately and skip the rest of the function!
 }

 // A named device
 if(arduinopath != NULL) return as_mac_try_port(arduinopath);

 // 'auto': list the candidate devices and try them in turn
 ncand = as_mac_list_candidates(cand, AS_MAC_MAX_CANDIDATES);
 if(ncand == 0){
   show_message(": Failed: no /dev/cu.usbmodem*, /dev/cu.usbserial*,"
                " /dev/cu.wchusbserial* or /dev/cu.SLAB_USBtoUART* device is"
                " present (is the Arduino plugged in?).\n",fnn,MT_ERR,0);
   return 1;
  }
 snprintf(infostr,sizeof(infostr),": %d candidate device(s) found:",ncand);
 show_message(infostr,fnn,MT_INFO,0);
 for(idx = 0; idx < ncand; idx++){
    snprintf(infostr,sizeof(infostr),":   %s (%s)",cand[idx].path,
              (cand[idx].rank == 0) ? "USB CDC-ACM" : "USB-serial bridge");
    show_message(infostr,fnn,MT_INFO,0);
   }
 for(idx = 0; idx < ncand; idx++){
    snprintf(infostr,sizeof(infostr),": Attempting coms on %s ...",cand[idx].path);
    show_message(infostr,fnn,MT_INFO,0);
    if(as_mac_try_port(cand[idx].path) == 0){
      snprintf(infostr,sizeof(infostr),": Arduino found on %s.",cand[idx].path);
      show_message(infostr,fnn,MT_INFO,0);
      return 0;
     }
   }
 show_message(": Failed: none of the candidate devices answered the PARD"
              " handshake. Name the device explicitly in the configuration"
              " file if the Arduino is on one of them.\n",fnn,MT_ERR,0);
 return 1;
}
#else
int as_open_comms_with_Arduino(char *arduinopath)
// Open and configure the serial port and send a handshake to the Arduino if it
// can be successfully found on arduinopath. Te 'auto' mode will check the two
// common paths of either /dev/ttyACM0 or /dev/ttyUSB0.
{
	struct termios termios_params;  // Terminal parameters for serial comms.
	unsigned char bytes_received[128];
	char bytes_sent[32];
 int retval=0,receptval;
 char infostr[320];
 size_t bytecount;
 struct serial_struct ser_info; // To reduce latency for faster comms.
 uint8_t acm=0;
 uint8_t attempt_number = 0;

try_again:
	sprintf(infostr,": Attempting serial USB port communication with Arduino: %s",
                                         (arduinopath==NULL)?"...":arduinopath);	
 show_message(infostr,fnn,MT_INFO,0);

 // A retry only follows a failed handshake, i.e. the port is already open and
 // configured, so go straight to the boot wait and the handshake. (Fixed
 // 14.11.26: the port used to be opened a second time here, leaving the first
 // descriptor open for the life of the program.)
 if(attempt_number) goto wait_for_arduino_boot;

 // Check for the 'no breakout board' option first
 if (arduinopath != NULL && strcmp(arduinopath, "nobob") == 0) {
     show_message(": ⚠️ Bypassing hardware breakout board.", fnn, MT_INFO, 0);
     
     // Open to the null device so as_portName holds a valid file descriptor for cleanup
     as_portName = open("/dev/null", O_RDWR);
     if (as_portName < 0) {
         show_message(": Failed to open /dev/null context.", fnn, MT_ERR, 0);
         return 1;
     }
     
     // Set connection status flags so the server knows it is initialized
     as_connected = 1; 
     
     show_message(": Comms established (Virtual Loopback).\n", fnn, MT_INFO, 0);
     return 0; // Return success immediately and skip the rest of the function!
 }

	//Open communication with Arduino.
 if(arduinopath!=NULL){
   as_portName = open(arduinopath, O_RDWR | O_NONBLOCK);
   if(as_portName<0){
     sprintf(infostr,": Failed: %s",strerror(errno));
     show_message(infostr,fnn,MT_ERR,0);
     retval=1;
     goto end_as_open_comms_with_Arduino;
    }
  } else {
   show_message(": Attempting coms on /dev/ttyACM0 ...",
                                        fnn,MT_INFO,0);
   as_portName = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK);
   if(as_portName<0){
     sprintf(infostr,": Failed: %s\n",strerror(errno));
     show_message(infostr,fnn,MT_ERR,0);
     show_message(": Attempting coms on /dev/ttyUSB0 ...",
                                        fnn,MT_INFO,0);
     as_portName = open("/dev/ttyUSB0", O_RDWR | O_NONBLOCK);
     if(as_portName<0){
       sprintf(infostr,": Failed: %s\n",strerror(errno));
       show_message(infostr,fnn,MT_ERR,0);
       retval=1;
       goto end_as_open_comms_with_Arduino;
      }	
    } else acm=1;
  }

 // Get the current terminal configuration attributes into the termios_params
 // structure
	tcgetattr(as_portName, &termios_params);

 // Set baud rate for input and output
	cfsetispeed(&termios_params, B500000);
	cfsetospeed(&termios_params, B500000);

 // Set 'raw' mode i.e. input is available character by character, echoing is
 // disabled, and all special processing of terminal input and output characters
 // is disabled. 
 cfmakeraw(&termios_params);

 // Apply the customised terminal configuration parameters after flushing any
 // data on the input or output buffers:
	if (tcsetattr(as_portName, TCSAFLUSH, &termios_params)){
    show_message(": Failed to implement serial port attributes.",
                                        fnn,MT_ERR,0);
    retval=1;
    goto end_as_open_comms_with_Arduino;
   };


 // 1. Get the current serial driver settings
 if(ioctl(as_portName, TIOCGSERIAL, &ser_info) < 0) {
   // If this fails, it's usually because the driver doesn't support TIOCGSERIAL
   // unless we are on ACM0, so:
   if(acm==0)
     show_message(": Warning: Could not read serial driver info for low latency.",
                                                                fnn, MT_ERR, 0);
  } else {
    // 2. Set the ASYNC_LOW_LATENCY flag
    ser_info.flags |= ASYNC_LOW_LATENCY;

    // 3. Apply the settings back to the driver
    if(ioctl(as_portName, TIOCSSERIAL, &ser_info) < 0) {
       show_message(": Warning: Failed to set low latency mode.", fnn, MT_ERR, 0);
    } else {
        // Success
    }
  }

 wait_for_arduino_boot:
 // If we failed once, it may be that the Arduino reset is blocking us so try
 // once more with a 3 second delay ...
 if(attempt_number){
   // Loop to handle the 3-second animated countdown
   for (int seconds_left = 3; seconds_left > 0; seconds_left--) {
       // Construct the message text string manually
       char countdown_str[128];
       snprintf(countdown_str, sizeof(countdown_str), 
                ": Waiting %d seconds for Arduino to boot... ", seconds_left);
       
       show_message(countdown_str, fnn, MT_INFO, 0);
       
       // Force the terminal console buffer to flush the text stream immediately
       fflush(stdout);
       
       // Wait exactly 1 second before looping to the next number
       sleep(1);
   }
   
   // Clean up the terminal line context right before proceeding to try again
   char clear_str[128];
   snprintf(clear_str, sizeof(clear_str),
                            ": Arduino boot wait completed.                  \n");
   show_message(clear_str, fnn, MT_INFO, 0);
 }


	// Send acknowledge command to Arduino

 sprintf(bytes_sent,"PARDack");
 sprintf(infostr,": Sending: %s",bytes_sent);
 show_message(infostr,fnn,MT_INFO,0);
	// Clear the input buffer to ensure we only see the reply to THIS command
 tcflush(as_portName, TCIFLUSH); 
	bytecount=write(as_portName, bytes_sent, 7);
 if(bytecount!=7){
   show_message(": Failed to send PARDack initiation signal. Quitting.",
                                        fnn,MT_ERR,0);
   retval=1;
   goto end_as_open_comms_with_Arduino;
  }
 tcdrain(as_portName); // Wait until all output has been sent
 msleep(20); // Safe 20ms delay for Arduino to process and reply

 // Listen for the expected reply
 
 receptval=as_read_sigstr((char *)bytes_received, "PARD_OK");
 if(receptval==0) show_message(": Comms established.\n",fnn,MT_INFO,0);
   else {
         sprintf(infostr,": Received wrong number (%s) of bytes. Quitting.\n",
                                                                bytes_received);
         show_message(infostr,fnn,MT_ERR,0);
         retval = 1;
        }

 if(retval && attempt_number == 0){
    show_message(": Trying once more but with a 3 second delay ...\n",
                                                                 fnn,MT_INFO,0);
    attempt_number = 1;
    retval=0;
    goto try_again;
  }

end_as_open_comms_with_Arduino:
 // Release a port that could not be brought into service (added 14.11.26).
 if(retval && as_portName >= 0){
   close(as_portName);
   as_portName = -1;
  }
 return retval;
}
#endif

int as_read_sigstr(char *receptbuff, char *expectstr)
// Read a string of chars from the serial port and compare it to the string
// expectstr. It is up to the user to ensure recepbuff has sufficient capcity.
// Returns 0 on success. On error the return value will be one of these:
// 1 = timeout reached or number of bytes read is not the expected number.
// 2 = received the expected number of bytes but does not match expectbuff
// 3 = expectstr has strlen of 0
// If there is a timeout error, the global as_portErr will be set to AS_ERR_TIME
// An error message will be printed into receptbuff accordingly so ensure that
// this is at least 128 chars long or the function may crash. 
{
 size_t elen,bytecount;
 int retval=0;


 elen=strlen(expectstr);
 if(elen==0){
   sprintf(receptbuff,"%s: Programming error (empty string requested).",fnn);
   retval=3;
   goto end_as_read_signal;
  }

 bytecount=as_read_bytes(receptbuff,(int)elen);
 if(bytechecksum(bytecount,elen, "received")){
   if(as_portErr==AS_ERR_TIME)
     sprintf(receptbuff,
              "%s: Communications timed out (%zu bytes received).",fnn,bytecount);
    else
     sprintf(receptbuff,
            "%s: Checksum error: %zu expected, %zu received.",fnn,elen,bytecount); 
   retval=1;
   goto end_as_read_signal;
  }
 receptbuff[elen] = '\0';

 if(strcmp(receptbuff,expectstr)){
   sprintf(receptbuff,"%s: Wrong acknowledgement signal returned.",fnn);
   retval=2;
   goto end_as_read_signal;
  }

end_as_read_signal:
 return retval;
}

#if IS_WINDOWS_NATIVE
size_t as_read_bytes(char *buff,int length)
// Equivalent of Arduino Serial.readBytes() function but for POSIX terminal
// Returns the number of bytes received (should be identical to length if all
// went well. It is up to the user to ensure length is <= the storage available
// in buff.
// If no data is received over the port for >= as_portTimeout ms then the
// function returns and the global as_portErr marker is set to AS_ERR_TIME and
// errno will be set according to what the POSIX read() function last set it to. 
// MS Windows version: ReadFile() on the COM port handle as_hCom replaces read().
// The port time-outs set in as_open_comms_with_Arduino make ReadFile()
// return at once with 0 bytes when nothing has arrived (as a non-blocking
// read() does); a failed ReadFile() is treated in the same way as a failed
// read() (no byte - wait and retry until as_portTimeout). The Win32 error
// code is then available from GetLastError() instead of errno.
{
 int pos;
 long noresp;
 DWORD readresult;
 size_t bytecount;

 as_portErr=AS_ERR_NONE;

 for(pos=0;pos<length;pos++) buff[pos]=0;
 pos=0;
 noresp=0;
 bytecount=0;

	//Read bytes_received answer
	while( pos < length ) {

		if(!ReadFile(as_hCom, buff+pos, 1, &readresult, NULL)) readresult=0;
  if(readresult>0){
    bytecount+=(size_t)readresult;
		  pos++;
   } else {
    // If the global as_NoWait is set, just return without error:
    if(as_NoWait) goto end_sread_bytes;
    // If no response, wait a little and listen again, unless you've tried for
    // too long without success (in which case, give up).
    if(noresp>as_portTimeout){
      as_portErr=AS_ERR_TIME;
      goto end_sread_bytes;
     }
    msleep(50);
    noresp+=50;
    continue;
   }
  noresp=0;
	}
 end_sread_bytes:
 as_NoWait=0;
 return bytecount;
}

uint8_t as_check_move_status(uint8_t *actype, uint8_t *actid, uint8_t *pmerr,
                                                                   char *errmsg)
// Checks the incoming serial port for an 'end of movement' feedback string that
// motion commands report when they are done (called a PARD Move Report or PMR).
// This function does not wait on the serial port - so if a message isn't there
// when it looks it will just return. If a PMR is received, the global coord
// variables (PMRep1,PMRep2,PMRep3,PMRep4) will receive the sent values.
// Returns 1 if the PMR is received.
// Returns 0 if not but no error.
// Returns 2 if something was received but not conforming to a PMR or if there
// was some other error.
// Returns 3 (AUDIT 26.10.2026) if a complete PMR with a correct checksum was
// received carrying a FATAL error code (below PARDERR_NON_FATAL, for example
// PARDU_ERR_KILLSWON): the PMRep values are valid, the move failed.
// MS Windows version: PurgeComm(as_hCom, PURGE_RXCLEAR) replaces tcflush(TCIFLUSH).
{
 int  bytecount,parderr;
	char bytes_received[4];

 // Now check for incomming PARD Move Report (PMR)
 as_NoWait=1; bytecount=as_read_bytes(bytes_received,3); as_NoWait=0;
 if(bytecount==0) return 0; // Nothing to see here. Move along.
 
 // If we saw data but it was cut short by serial port slicing, wait briefly or read the rest
 if(bytecount < 3){
    // Force a rapid microsecond block read to capture the remaining missing signature bytes
    bytecount += as_read_bnum(bytes_received + bytecount, 3 - bytecount);
 }

 if(bytecount==3){
    if(!(bytes_received[0]=='P' && 
         bytes_received[1]=='M' &&
         bytes_received[2]=='R') ) {
       bytes_received[3]='\0';
       sprintf(errmsg,": Error: Expected 'PMR' but got '%s'\n,",bytes_received);
       show_message(errmsg,fnn,MT_ERR,0);
       PurgeComm(as_hCom, PURGE_RXCLEAR); // Clear anything else waiting to be read
       return 2;
      }
   }

 // PMR initiated, receive values:
 bytecount =as_read_bnum(actype,  1);
 bytecount+=as_read_bnum(actid,   1);
 bytecount+=as_read_bnum(pmerr,   1);
 bytecount+=as_read_bnum(&PMRep1, 4);
 bytecount+=as_read_bnum(&PMRep2, 4);
 bytecount+=as_read_bnum(&PMRep3, 4);
 bytecount+=as_read_bnum(&PMRep4, 4);

 if(bytechecksum(bytecount,ARDU_PMRBYTES, "received")){
    sprintf(errmsg,": Checksum error receiving PMR coords\n");
    show_message(errmsg,fnn,MT_ERR,0);
    PurgeComm(as_hCom, PURGE_RXCLEAR); // Clear anything else waiting to be read
    return 2;
   } 

 if(*pmerr){
   parderr=(int)(*pmerr);
   Pard_Err=parderr;
   sprintf(errmsg,": %s",Pard_err_str());
   if(parderr >= PARDERR_NON_FATAL){
     // If this is a non-fatal error just print it locally to the server but
     // return 1 (i.e. the 'PMR received with no error' signal).
     show_message(errmsg,fnn,MT_ERR,0);
     return 1;
    }
   // For all other errors, return the error condition signal:
   // AUDIT (26.10.2026): but this PMR arrived complete and its checksum is
   // right, so PMRep1..PMRep4 are where the motors really are (a move stopped
   // by the kill switch or by a loss of Vcc: the firmware counts every step it
   // made, and pulses no more). Return 3, not 2, so that act_movereport can
   // keep those positions before it reports the error.
   return 3;
  }

 return 1;
}
#else
size_t as_read_bytes(char *buff,int length)
// Equivalent of Arduino Serial.readBytes() function but for POSIX terminal
// Returns the number of bytes received (should be identical to length if all
// went well. It is up to the user to ensure length is <= the storage available
// in buff.
// If no data is received over the port for >= as_portTimeout ms then the
// function returns and the global as_portErr marker is set to AS_ERR_TIME and
// errno will be set according to what the POSIX read() function last set it to. 
{
 int pos;
 long noresp;
 int readresult;
 size_t bytecount;

 as_portErr=AS_ERR_NONE;

 for(pos=0;pos<length;pos++) buff[pos]=0;
 pos=0;
 noresp=0;
 bytecount=0;

	//Read bytes_received answer
	while( pos < length ) {

		readresult=read(as_portName, buff+pos, 1);
  if(readresult>0){
    bytecount+=(size_t)readresult;
		  pos++;
   } else {
    // If the global as_NoWait is set, just return without error:
    if(as_NoWait) goto end_sread_bytes;
    // If no response, wait a little and listen again, unless you've tried for
    // too long without success (in which case, give up).
    if(noresp>as_portTimeout){
      as_portErr=AS_ERR_TIME;
      goto end_sread_bytes;
     }
    msleep(50);
    noresp+=50;
    continue;
   }
  noresp=0;
	}
 end_sread_bytes:
 as_NoWait=0;
 return bytecount;
}

uint8_t as_check_move_status(uint8_t *actype, uint8_t *actid, uint8_t *pmerr,
                                                                   char *errmsg)
// Checks the incoming serial port for an 'end of movement' feedback string that
// motion commands report when they are done (called a PARD Move Report or PMR).
// This function does not wait on the serial port - so if a message isn't there
// when it looks it will just return. If a PMR is received, the global coord
// variables (PMRep1,PMRep2,PMRep3,PMRep4) will receive the sent values.
// Returns 1 if the PMR is received.
// Returns 0 if not but no error.
// Returns 2 if something was received but not conforming to a PMR or if there
// was some other error.
// Returns 3 (AUDIT 26.10.2026) if a complete PMR with a correct checksum was
// received carrying a FATAL error code (below PARDERR_NON_FATAL, for example
// PARDU_ERR_KILLSWON): the PMRep values are valid, the move failed.
{
 int  bytecount,parderr;
	char bytes_received[4];

 // Now check for incomming PARD Move Report (PMR)
 as_NoWait=1; bytecount=as_read_bytes(bytes_received,3); as_NoWait=0;
 if(bytecount==0) return 0; // Nothing to see here. Move along.
 
 // If we saw data but it was cut short by serial port slicing, wait briefly or read the rest
 if(bytecount < 3){
    // Force a rapid microsecond block read to capture the remaining missing signature bytes
    bytecount += as_read_bnum(bytes_received + bytecount, 3 - bytecount);
 }

 if(bytecount==3){
    if(!(bytes_received[0]=='P' && 
         bytes_received[1]=='M' &&
         bytes_received[2]=='R') ) {
       bytes_received[3]='\0';
       sprintf(errmsg,": Error: Expected 'PMR' but got '%s'\n,",bytes_received);
       show_message(errmsg,fnn,MT_ERR,0);
       tcflush(as_portName, TCIFLUSH); // Clear anything else waiting to be read
       return 2;
      }
   }

 // PMR initiated, receive values:
 bytecount =as_read_bnum(actype,  1);
 bytecount+=as_read_bnum(actid,   1);
 bytecount+=as_read_bnum(pmerr,   1);
 bytecount+=as_read_bnum(&PMRep1, 4);
 bytecount+=as_read_bnum(&PMRep2, 4);
 bytecount+=as_read_bnum(&PMRep3, 4);
 bytecount+=as_read_bnum(&PMRep4, 4);

 if(bytechecksum(bytecount,ARDU_PMRBYTES, "received")){
    sprintf(errmsg,": Checksum error receiving PMR coords\n");
    show_message(errmsg,fnn,MT_ERR,0);
    tcflush(as_portName, TCIFLUSH); // Clear anything else waiting to be read
    return 2;
   } 

 if(*pmerr){
   parderr=(int)(*pmerr);
   Pard_Err=parderr;
   sprintf(errmsg,": %s",Pard_err_str());
   if(parderr >= PARDERR_NON_FATAL){
     // If this is a non-fatal error just print it locally to the server but
     // return 1 (i.e. the 'PMR received with no error' signal).
     show_message(errmsg,fnn,MT_ERR,0);
     return 1;
    }
   // For all other errors, return the error condition signal:    
   // AUDIT (26.10.2026): but this PMR arrived complete and its checksum is
   // right, so PMRep1..PMRep4 are where the motors really are (a move stopped
   // by the kill switch or by a loss of Vcc: the firmware counts every step it
   // made, and pulses no more). Return 3, not 2, so that act_movereport can
   // keep those positions before it reports the error.
   return 3;
  }

 return 1;
}
#endif

size_t as_read_bnum(void *iptr, int ilen)
// Reads a binary number of size ilen bytes from the serial port. It assumes the
// binary number sent is in the same size, endianness and format of the
// equivalent binary number on this local machine. The data type of the binary
// number is up to the user: int, char, unsigned long int, float, double, etc.
// This function reads one byte at a time and checks timeout with each byte.
// Returns the number of bytes received (should be identical to ilen if all went
// well). It is up to the user to ensure ilen is <= the storage available in
// iptr.
// If no data is received over the port for >= as_portTimeout ms then the
// function returns 0 or the number of bytes read so far and the global
// as_portErr marker is set to AS_ERR_TIME and errno will be set according to
// what the read() function last set it to.
{
   return as_read_bytes((char *)iptr, ilen);
}

#if IS_WINDOWS_NATIVE
int as_send_order(void)
// Send order information to the Arduino and receive its report.
// Returns one of the PARDU_ERR_ values (PARDU_ERR_NONE (==0) if all went well).
// MS Windows version: the serial port is the Win32 handle as_hCom and
//   write(as_portName, ...)     -> as_win_write(...)  (same return values)
//   tcflush(as_portName, TCIFLUSH) -> PurgeComm(as_hCom, PURGE_RXCLEAR)
//   tcflush(as_portName, TCOFLUSH) -> PurgeComm(as_hCom, PURGE_TXCLEAR)
//   tcdrain(as_portName)        -> FlushFileBuffers(as_hCom)
// Everything else (the PARD serial protocol) is identical to Linux.
{
 int retval,bytecount,receptval;
	char bytes_sent[32],bytes_received[128];
 char log_message[320];
 uint32_t nmsg;
 size_t checksumadjust;
 uint8_t error_state;

 // Log the call
 sprintf(log_message,": Sending order %"PRIu32", actid[%"PRIu8"], otype[%"PRIu8"]"
                                    "\n",CurrMsg,Order_ID.actid,Order_ID.otype);
 show_message(log_message,fnn,MT_INFO,0);

 // Start witha clean slate - we'll change this if necessary as we go along
 retval = PARDU_ERR_NONE;

 // Clear the input buffer in case an unread PMR was hanging around there.
 PurgeComm(as_hCom, PURGE_RXCLEAR);

 // Send order commencement signal:
 sprintf(bytes_sent,PARD_ORDER_START);
 bytecount=0;
	bytecount+=as_win_write(bytes_sent,PARD_ORDER_START_BYTES);

 // Send the common ID information
	bytecount+=as_win_write(&Vb1,             1);
	bytecount+=as_win_write(&CurrMsg,         4);
	bytecount+=as_win_write(&Order_ID.otype,  1);
	bytecount+=as_win_write(&Order_ID.actype, 1);
	bytecount+=as_win_write(&Order_ID.actid,  1);

 // Ensure the common info was all sent, otherwise clear the write buffer
 if(bytechecksum(bytecount,8 + PARD_ORDER_START_BYTES, "sent"))
 {PurgeComm(as_hCom, PURGE_TXCLEAR); retval=PARDU_ERR_CHKSUM; goto end_send_order;}

 // Send other order data items to Arduino as per the Order_ID.otype
 bytecount=0;

 switch(Order_ID.otype){
   case ORDER_HALT:
   case ORDER_CREATE:
   case ORDER_DESTROY:
   // Nothing more to send for these
   break;
   case ORDER_STAGE_MOVE:
	    bytecount+=as_win_write(&Order_DirSteps.dirsteps_x,    4);
	    bytecount+=as_win_write(&Order_DirSteps.dirsteps_y,    4);
     if(bytechecksum(bytecount,8, "sent"))// Clear the write buffer
 {PurgeComm(as_hCom, PURGE_TXCLEAR); retval=PARDU_ERR_CHKSUM; goto end_send_order;}
     bytecount=0;
   // Don't 'break', just follow through:
   case ORDER_STEPPER_MOVE:
	    bytecount+=as_win_write(&Order_DirSteps.dirsteps_z,    4);
	    bytecount+=as_win_write(&Order_Motion.delay_max,       4);
     bytecount+=as_win_write(&Order_Motion.ash       ,      4);
     bytecount+=as_win_write(&Order_Motion.delay_diff_diff, 4);
     bytecount+=as_win_write(&Order_Motion.shift_s,         1);
     bytecount+=as_win_write(&Order_Motion.xsresid   ,      4);
     bytecount+=as_win_write(&Order_Motion.cvsteps   ,      4);
     if(bytechecksum(bytecount,25, "sent"))// Clear the write buffer
 {PurgeComm(as_hCom, PURGE_TXCLEAR); retval=PARDU_ERR_CHKSUM; goto end_send_order;}
   break;
   case ORDER_STAGE_HOME:
     bytecount+=as_win_write(&Order_Home.dirn_x,      1);
     bytecount+=as_win_write(&Order_Home.dirn_y,      1);
     if(bytechecksum(bytecount,2, "sent"))// Clear the write buffer
 {PurgeComm(as_hCom, PURGE_TXCLEAR); retval=PARDU_ERR_CHKSUM; goto end_send_order;}
     bytecount=0;
   // Don't 'break', just follow through:
   case ORDER_STEPPER_HOME:
     bytecount+=as_win_write(&Order_Home.dirn_z,      1);
     if(bytechecksum(bytecount,1, "sent"))// Clear the write buffer
 {PurgeComm(as_hCom, PURGE_TXCLEAR); retval=PARDU_ERR_CHKSUM; goto end_send_order;}
   break;
   case ORDER_GET:
     bytecount+=as_win_write(&Order_Info.infotype,    1);
     if(bytechecksum(bytecount,1, "sent"))// Clear the write buffer
 {PurgeComm(as_hCom, PURGE_TXCLEAR); retval=PARDU_ERR_CHKSUM; goto end_send_order;}
   break;
   case ORDER_SET:
     bytecount+=as_win_write(&Order_Info.infotype,    1);
     checksumadjust=1;
     switch(Order_Info.infotype){
        case INFO_STEPCOUNT: // Read only for client, but server needs to set it
        case INFO_BLSTEPS:   // Read only for client, but server needs to set it
        case INFO_MSDENOM:
        case INFO_ONESTEP: 
        case INFO_FASTMSD:
        case INFO_ISD_HOBL:  // same as INFO_SRV_ISD
        case INFO_TENSTEPS1: // same as INFO_SRV_QPOS
        case INFO_TENSTEPS2: // same as INFO_SRV_SPOS
        case INFO_SW_LIM1:   // same as INFO_SRV_MIN
        case INFO_SW_LIM2:   // same as INFO_SRV_MAX
         bytecount+=as_win_write(&Order_Info.value.ui32val,4);
         checksumadjust+=4;
        break;
        case INFO_ML_LEVEL:
         if(Mlevellist[Order_ID.actid]->ml_type == MLT_PWM){
            bytecount+=as_win_write(&Order_Info.value.biuinval,8);
            checksumadjust+=8;
           } else { // It must be a parallel type so just send the value
            bytecount+=as_win_write(&Order_Info.value.ui8val,1);
            checksumadjust++;
           }
        break;
        case INFO_NHS:
        case INFO_XYZCOORDS:
         bytecount+=as_win_write(Order_Info.value.triuinval,12);
         checksumadjust+=12;
        break;
        default: // Everything else is uint8_t so ...
         bytecount+=as_win_write(&Order_Info.value.ui8val,     1);
         checksumadjust++;
        break;
       }
     if(bytecount!=checksumadjust)// Clear the write buffer
 {PurgeComm(as_hCom, PURGE_TXCLEAR); retval=PARDU_ERR_CHKSUM; goto end_send_order;}
   break;
   case ORDER_ACCEL:
     // Send the acceleration constant byte
     bytecount+=as_win_write(&sv_Accparam[Order_ID.actype].s_acc,1);
     checksumadjust=1;
     // Send the motion duration value
     bytecount+=as_win_write(&sv_Accparam[Order_ID.actype].duration,4);
     checksumadjust+=4;
     if(bytecount!=checksumadjust)// Clear the write buffer
 {PurgeComm(as_hCom, PURGE_TXCLEAR); retval=PARDU_ERR_CHKSUM; goto end_send_order;}
   break;
   case ORDER_SERVO_MOVE:
     // Send 1-byte binary on/off execution trigger parameter
     bytecount += as_win_write(&Order_Info.value.ui8val, 1);
     checksumadjust = 1;
     if(bytecount != checksumadjust) {
        PurgeComm(as_hCom, PURGE_TXCLEAR); retval = PARDU_ERR_CHKSUM; goto end_send_order;
     }
   break;
   default:
     PurgeComm(as_hCom, PURGE_TXCLEAR);
     retval=PARDU_ERR_UNKORDER;
     goto end_send_order;
   break;
  }

 // Send order termination signal:
 sprintf(bytes_sent,PARD_ORDER_END);
	bytecount=as_win_write(bytes_sent, PARD_ORDER_END_BYTES);

 // Clear the write buffer if we did not receive the correct number of bytes
 if(bytechecksum(bytecount,PARD_ORDER_END_BYTES, "sent"))
 {PurgeComm(as_hCom, PURGE_TXCLEAR); retval=PARDU_ERR_CHKSUM; goto end_send_order;}

 FlushFileBuffers(as_hCom); // Wait until all output has been sent

 sprintf(log_message,": Order sent, waiting for report.\n");
 show_message(log_message,fnn,MT_INFO,0);

 // All data sent, now wait for the Arduino's report
 bytes_received[0]='\0';
 receptval=as_read_sigstr(bytes_received, PARD_REPORT_START);
 if(receptval){
     sprintf(log_message,
     ": Error: Expected '%s' but got '%s'\n,",PARD_REPORT_START,bytes_received);
     show_message(log_message,fnn,MT_ERR,0);
     retval = PARDU_ERR_INFINVAL; goto end_send_order;
   }

 // Arduino report initiated, receive message number:
 bytecount=as_read_bnum(&nmsg, 4);

 if(bytechecksum(bytecount, 4, "received")){
    sprintf(log_message,": Checksum error receiving message number\n");
    show_message(log_message,fnn,MT_ERR,0);
    retval = PARDU_ERR_CHKSUM; goto end_send_order;
   } else {
    
   sprintf(log_message,": PARD_Report[%"PRIu32"]:\n",nmsg);
   show_message(log_message,fnn,MT_ERR,0);
  }

 // Receive error state value
 bytecount=as_read_bnum(&error_state, 1);

 if(bytechecksum(bytecount, 1, "received")){
    sprintf(log_message,": Checksum error receiving error state number\n");
    show_message(log_message,fnn,MT_ERR,0);
    retval = PARDU_ERR_CHKSUM; goto end_send_order;
   } else {
    
   sprintf(log_message,": Error_state = %"PRIu8"\n",error_state);
   show_message(log_message,fnn,MT_ERR,0);
  }

 // Receive more as per error_state value, actuator type and order type
 if(error_state == PARDU_ERR_NONE){
   switch(Order_ID.actype){
     case ACT_STEPPER:
      switch(Order_ID.otype){

         case ORDER_STAGE_MOVE:
         case ORDER_STAGE_HOME:
          // Send back XYZ coords (stepcounts) + steps done + time taken + limit switches status
         break;

         case ORDER_GET:
           // Send back the information got
           switch(Order_Info.infotype){
              case INFO_STEPCOUNT:
              case INFO_ONESTEP:
              case INFO_BLSTEPS:
              case INFO_MSDENOM:
              case INFO_FASTMSD:
              case INFO_TMCSGVAL :
              case INFO_TMCSTATUS:
              case INFO_TMCCHCONF:
              case INFO_TMCMSCNT :
              case INFO_DURALAM:
              case INFO_STEPLAM:
                bytecount=as_read_bnum(&(Order_Info.value.ui32val), 4);
                if(bytechecksum(bytecount, 4, "received")){
                   sprintf(log_message,": Checksum error receiving stepper ui32val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              case INFO_ENA_STATE: // same as INFO_ML_STATE and INFO_BIN_STATE
              case INFO_MMODE:
              case INFO_TURBO_BST:
              case INFO_SG_BUFFPC:
              case INFO_SG_THRESH:
              case INFO_HOMED_TO:
              case INFO_DIRNC:    // same as INFO_ML_TIMER and INFO_SRV_TIMER
              case INFO_DIRNP:
              case INFO_PULWIDTH:
              case INFO_LIMSTATE:
              case INFO_PIN_PUL:
              case INFO_PIN_DIR:
              case INFO_PIN_LIM1:
              case INFO_PIN_LIM2:
              case INFO_PIN_DIAG:
              case INFO_PIN_ENA:
              case INFO_PIN_MS1:
              case INFO_PIN_MS2:
              case INFO_PIN_MS3:
              case INFO_SD_TYPE:
                bytecount=as_read_bnum(&(Order_Info.value.ui8val), 1);
                if(bytechecksum(bytecount, 1, "received")){
                   sprintf(log_message,": Checksum error receiving stepper ui8val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              case INFO_XYZCOORDS:
                bytecount=as_read_bnum(Order_Info.value.triuinval, 12);
                if(bytechecksum(bytecount, 12, "received")){
                   sprintf(log_message,": Checksum error receiving stepper triuinval.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              default: // This should never happen
              break;
            }
         break;
     
         case ORDER_STEPPER_MOVE:
         case ORDER_STEPPER_HOME:
          // Send back stepper coord (stepcount) + steps done + time taken + limit switch status
         break;

         default:
         break;
        }  // End of: switch(Order_ID.otype)

     break;
     case ACT_BINARY:
      switch(Order_ID.otype){

         case ORDER_GET:
           // Send back the information got
           switch(Order_Info.infotype){
              case INFO_BIN_STATE:
                bytecount=as_read_bnum(&(Order_Info.value.ui8val), 1);
                if(bytechecksum(bytecount, 1, "received")){
                   sprintf(log_message,": Checksum error receiving binary ui8val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              default: // This should never happen
              break;
            }
         break;
     
         default:
         break;
        }  // End of: switch(Order_ID.otype)

     break;
     case ACT_MLEVEL:
      switch(Order_ID.otype){

         case ORDER_GET:
           // Send back the information got
           switch(Order_Info.infotype){
              case INFO_ML_STATE:
              case INFO_ML_TIMER:
                bytecount=as_read_bnum(&(Order_Info.value.ui8val), 1);
                if(bytechecksum(bytecount, 1, "received")){
                   sprintf(log_message,": Checksum error receiving M-level ui8val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              case INFO_ML_FREQ:
                bytecount=as_read_bnum(&(Order_Info.value.ui32val), 4);
                if(bytechecksum(bytecount, 4, "received")){
                   sprintf(log_message,": Checksum error receiving M-level ui32val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              default: // This should never happen
              break;
            }
         break;
     
         default:
         break;
        }  // End of: switch(Order_ID.otype)
     break;

     case ACT_SERVO:
      switch(Order_ID.otype){

         case ORDER_GET:
           // Send back the information got
           switch(Order_Info.infotype){
              case INFO_SRV_STATE:
              case INFO_SRV_TIMER:
                bytecount=as_read_bnum(&(Order_Info.value.ui8val), 1);
                if(bytechecksum(bytecount, 1, "received")){
                   sprintf(log_message,": Checksum error receiving servo ui8val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              case INFO_SRV_AFB:
              case INFO_SRV_SPOS:
                bytecount=as_read_bnum(&(Order_Info.value.ui32val), 4);
                if(bytechecksum(bytecount, 4, "received")){
                   sprintf(log_message,": Checksum error receiving servo ui32val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              default: // This should never happen
              break;
            }
         break;
     
         case ORDER_SERVO_MOVE:
            // Extract the 1-byte PMR pending flag
            bytecount=as_read_bnum(&(sv_Accparam[ACT_SERVO].pmr_pending), 1);
            if(bytechecksum(bytecount, 1, "received")){
               sprintf(log_message,": Checksum error receiving servo pmr_pending.\n");
               show_message(log_message,fnn,MT_ERR,0);
               retval = PARDU_ERR_CHKSUM; goto end_send_order;
              }
         break;

         default:
         break;
        }  // End of: switch(Order_ID.otype)
     break;

     case ACT_SENSOR:
      switch(Order_ID.otype){

         case ORDER_GET:
           // Send back the information got
           switch(Order_Info.infotype){
              case INFO_SEN_VAL:
                bytecount=as_read_bnum(&(Order_Info.value.ui32val), 4);
                if(bytechecksum(bytecount, 4, "received")){
                   sprintf(log_message,": Checksum error receiving sensor ui32val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              default: // This should never happen
              break;
            }
         break;
         default:
         break;
        }  // End of: switch(Order_ID.otype)
     break;

     default:
     break;
    } // End of: switch(Order_ID.actype){

   } else retval = error_state;


 // That should be all for this report. So look for the termination signal:
 receptval=as_read_sigstr(bytes_received, PARD_REPORT_END);
 if(receptval){
          sprintf(log_message,": Error: %s\n,",bytes_received);
          show_message(log_message,fnn,MT_ERR,0);
          retval = PARDU_ERR_NOTERM; goto end_send_order;
        } else {
   sprintf(log_message,": PARD_Report_end.\n");
   show_message(log_message,fnn,MT_ERR,0);
  }

 // Message cycle complete. Increment serial message counter:
 CurrMsg++;

end_send_order:
 return retval;
}
#else
int as_send_order(void)
// Send order information to the Arduino and receive its report.
// Returns one of the PARDU_ERR_ values (PARDU_ERR_NONE (==0) if all went well).
{
 int retval,bytecount,receptval;
	char bytes_sent[32],bytes_received[128];
 char log_message[320];
 uint32_t nmsg;
 size_t checksumadjust;
 uint8_t error_state;

 // Log the call
 sprintf(log_message,": Sending order %"PRIu32", actid[%"PRIu8"], otype[%"PRIu8"]"
                                    "\n",CurrMsg,Order_ID.actid,Order_ID.otype);
 show_message(log_message,fnn,MT_INFO,0);

 // Start witha clean slate - we'll change this if necessary as we go along
 retval = PARDU_ERR_NONE;

 // Clear the input buffer in case an unread PMR was hanging around there.
 tcflush(as_portName, TCIFLUSH);

 // Send order commencement signal:
 sprintf(bytes_sent,PARD_ORDER_START);
 bytecount=0;
	bytecount+=write(as_portName, bytes_sent,PARD_ORDER_START_BYTES);

 // Send the common ID information
	bytecount+=write(as_portName, &Vb1,             1);
	bytecount+=write(as_portName, &CurrMsg,         4);
	bytecount+=write(as_portName, &Order_ID.otype,  1);
	bytecount+=write(as_portName, &Order_ID.actype, 1);
	bytecount+=write(as_portName, &Order_ID.actid,  1);

 // Ensure the common info was all sent, otherwise clear the write buffer
 if(bytechecksum(bytecount,8 + PARD_ORDER_START_BYTES, "sent"))
 {tcflush(as_portName, TCOFLUSH); retval=PARDU_ERR_CHKSUM; goto end_send_order;}

 // Send other order data items to Arduino as per the Order_ID.otype
 bytecount=0;

 switch(Order_ID.otype){
   case ORDER_HALT:
   case ORDER_CREATE:
   case ORDER_DESTROY:
   // Nothing more to send for these
   break;
   case ORDER_STAGE_MOVE:
	    bytecount+=write(as_portName, &Order_DirSteps.dirsteps_x,    4);
	    bytecount+=write(as_portName, &Order_DirSteps.dirsteps_y,    4);
     if(bytechecksum(bytecount,8, "sent"))// Clear the write buffer
 {tcflush(as_portName, TCOFLUSH); retval=PARDU_ERR_CHKSUM; goto end_send_order;}
     bytecount=0;
   // Don't 'break', just follow through:
   case ORDER_STEPPER_MOVE:
	    bytecount+=write(as_portName, &Order_DirSteps.dirsteps_z,    4);
	    bytecount+=write(as_portName, &Order_Motion.delay_max,       4);
     bytecount+=write(as_portName, &Order_Motion.ash       ,      4);
     bytecount+=write(as_portName, &Order_Motion.delay_diff_diff, 4);
     bytecount+=write(as_portName, &Order_Motion.shift_s,         1);
     bytecount+=write(as_portName, &Order_Motion.xsresid   ,      4);
     bytecount+=write(as_portName, &Order_Motion.cvsteps   ,      4);
     if(bytechecksum(bytecount,25, "sent"))// Clear the write buffer
 {tcflush(as_portName, TCOFLUSH); retval=PARDU_ERR_CHKSUM; goto end_send_order;}
   break;
   case ORDER_STAGE_HOME:
     bytecount+=write(as_portName, &Order_Home.dirn_x,      1);
     bytecount+=write(as_portName, &Order_Home.dirn_y,      1);
     if(bytechecksum(bytecount,2, "sent"))// Clear the write buffer
 {tcflush(as_portName, TCOFLUSH); retval=PARDU_ERR_CHKSUM; goto end_send_order;}
     bytecount=0;
   // Don't 'break', just follow through:
   case ORDER_STEPPER_HOME:
     bytecount+=write(as_portName, &Order_Home.dirn_z,      1);
     if(bytechecksum(bytecount,1, "sent"))// Clear the write buffer
 {tcflush(as_portName, TCOFLUSH); retval=PARDU_ERR_CHKSUM; goto end_send_order;}
   break;
   case ORDER_GET:
     bytecount+=write(as_portName, &Order_Info.infotype,    1);
     if(bytechecksum(bytecount,1, "sent"))// Clear the write buffer
 {tcflush(as_portName, TCOFLUSH); retval=PARDU_ERR_CHKSUM; goto end_send_order;}
   break;
   case ORDER_SET:
     bytecount+=write(as_portName, &Order_Info.infotype,    1);
     checksumadjust=1;
     switch(Order_Info.infotype){
        case INFO_STEPCOUNT: // Read only for client, but server needs to set it
        case INFO_BLSTEPS:   // Read only for client, but server needs to set it
        case INFO_MSDENOM:
        case INFO_ONESTEP: 
        case INFO_FASTMSD:
        case INFO_ISD_HOBL:  // same as INFO_SRV_ISD
        case INFO_TENSTEPS1: // same as INFO_SRV_QPOS
        case INFO_TENSTEPS2: // same as INFO_SRV_SPOS
        case INFO_SW_LIM1:   // same as INFO_SRV_MIN
        case INFO_SW_LIM2:   // same as INFO_SRV_MAX
         bytecount+=write(as_portName, &Order_Info.value.ui32val,4);
         checksumadjust+=4;
        break;
        case INFO_ML_LEVEL:
         if(Mlevellist[Order_ID.actid]->ml_type == MLT_PWM){
            bytecount+=write(as_portName, &Order_Info.value.biuinval,8);
            checksumadjust+=8;
           } else { // It must be a parallel type so just send the value
            bytecount+=write(as_portName, &Order_Info.value.ui8val,1);
            checksumadjust++;
           }
        break;
        case INFO_NHS:
        case INFO_XYZCOORDS:
         bytecount+=write(as_portName, Order_Info.value.triuinval,12);
         checksumadjust+=12;
        break;
        default: // Everything else is uint8_t so ...
         bytecount+=write(as_portName, &Order_Info.value.ui8val,     1);
         checksumadjust++;
        break;
       }
     if(bytecount!=checksumadjust)// Clear the write buffer
 {tcflush(as_portName, TCOFLUSH); retval=PARDU_ERR_CHKSUM; goto end_send_order;}
   break;
   case ORDER_ACCEL:
     // Send the acceleration constant byte
     bytecount+=write(as_portName, &sv_Accparam[Order_ID.actype].s_acc,1);
     checksumadjust=1;
     // Send the motion duration value
     bytecount+=write(as_portName, &sv_Accparam[Order_ID.actype].duration,4);
     checksumadjust+=4;
     if(bytecount!=checksumadjust)// Clear the write buffer
 {tcflush(as_portName, TCOFLUSH); retval=PARDU_ERR_CHKSUM; goto end_send_order;}
   break;
   case ORDER_SERVO_MOVE:
     // Send 1-byte binary on/off execution trigger parameter
     bytecount += write(as_portName, &Order_Info.value.ui8val, 1);
     checksumadjust = 1;
     if(bytecount != checksumadjust) {
        tcflush(as_portName, TCOFLUSH); retval = PARDU_ERR_CHKSUM; goto end_send_order;
     }
   break;
   default:
     tcflush(as_portName, TCOFLUSH);
     retval=PARDU_ERR_UNKORDER;
     goto end_send_order;
   break;
  }

 // Send order termination signal:
 sprintf(bytes_sent,PARD_ORDER_END);
	bytecount=write(as_portName, bytes_sent, PARD_ORDER_END_BYTES);

 // Clear the write buffer if we did not receive the correct number of bytes
 if(bytechecksum(bytecount,PARD_ORDER_END_BYTES, "sent"))
 {tcflush(as_portName, TCOFLUSH); retval=PARDU_ERR_CHKSUM; goto end_send_order;}

 tcdrain(as_portName); // Wait until all output has been sent

 sprintf(log_message,": Order sent, waiting for report.\n");
 show_message(log_message,fnn,MT_INFO,0);

 // All data sent, now wait for the Arduino's report
 bytes_received[0]='\0';
 receptval=as_read_sigstr(bytes_received, PARD_REPORT_START);
 if(receptval){
     sprintf(log_message,
     ": Error: Expected '%s' but got '%s'\n,",PARD_REPORT_START,bytes_received);
     show_message(log_message,fnn,MT_ERR,0);
     retval = PARDU_ERR_INFINVAL; goto end_send_order;
   }

 // Arduino report initiated, receive message number:
 bytecount=as_read_bnum(&nmsg, 4);

 if(bytechecksum(bytecount, 4, "received")){
    sprintf(log_message,": Checksum error receiving message number\n");
    show_message(log_message,fnn,MT_ERR,0);
    retval = PARDU_ERR_CHKSUM; goto end_send_order;
   } else {
    
   sprintf(log_message,": PARD_Report[%"PRIu32"]:\n",nmsg);
   show_message(log_message,fnn,MT_ERR,0);
  }

 // Receive error state value
 bytecount=as_read_bnum(&error_state, 1);

 if(bytechecksum(bytecount, 1, "received")){
    sprintf(log_message,": Checksum error receiving error state number\n");
    show_message(log_message,fnn,MT_ERR,0);
    retval = PARDU_ERR_CHKSUM; goto end_send_order;
   } else {
    
   sprintf(log_message,": Error_state = %"PRIu8"\n",error_state);
   show_message(log_message,fnn,MT_ERR,0);
  }

 // Receive more as per error_state value, actuator type and order type
 if(error_state == PARDU_ERR_NONE){
   switch(Order_ID.actype){
     case ACT_STEPPER:
      switch(Order_ID.otype){

         case ORDER_STAGE_MOVE:
         case ORDER_STAGE_HOME:
          // Send back XYZ coords (stepcounts) + steps done + time taken + limit switches status
         break;

         case ORDER_GET:
           // Send back the information got
           switch(Order_Info.infotype){
              case INFO_STEPCOUNT:
              case INFO_ONESTEP:
              case INFO_BLSTEPS:
              case INFO_MSDENOM:
              case INFO_FASTMSD:
              case INFO_TMCSGVAL :
              case INFO_TMCSTATUS:
              case INFO_TMCCHCONF:
              case INFO_TMCMSCNT :
              case INFO_DURALAM:
              case INFO_STEPLAM:
                bytecount=as_read_bnum(&(Order_Info.value.ui32val), 4);
                if(bytechecksum(bytecount, 4, "received")){
                   sprintf(log_message,": Checksum error receiving stepper ui32val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              case INFO_ENA_STATE: // same as INFO_ML_STATE and INFO_BIN_STATE
              case INFO_MMODE:
              case INFO_TURBO_BST:
              case INFO_SG_BUFFPC:
              case INFO_SG_THRESH:
              case INFO_HOMED_TO:
              case INFO_DIRNC:    // same as INFO_ML_TIMER and INFO_SRV_TIMER
              case INFO_DIRNP:
              case INFO_PULWIDTH:
              case INFO_LIMSTATE:
              case INFO_PIN_PUL:
              case INFO_PIN_DIR:
              case INFO_PIN_LIM1:
              case INFO_PIN_LIM2:
              case INFO_PIN_DIAG:
              case INFO_PIN_ENA:
              case INFO_PIN_MS1:
              case INFO_PIN_MS2:
              case INFO_PIN_MS3:
              case INFO_SD_TYPE:
                bytecount=as_read_bnum(&(Order_Info.value.ui8val), 1);
                if(bytechecksum(bytecount, 1, "received")){
                   sprintf(log_message,": Checksum error receiving stepper ui8val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              case INFO_XYZCOORDS:
                bytecount=as_read_bnum(Order_Info.value.triuinval, 12);
                if(bytechecksum(bytecount, 12, "received")){
                   sprintf(log_message,": Checksum error receiving stepper triuinval.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              default: // This should never happen
              break;
            }
         break;
     
         case ORDER_STEPPER_MOVE:
         case ORDER_STEPPER_HOME:
          // Send back stepper coord (stepcount) + steps done + time taken + limit switch status
         break;

         default:
         break;
        }  // End of: switch(Order_ID.otype)

     break;
     case ACT_BINARY:
      switch(Order_ID.otype){

         case ORDER_GET:
           // Send back the information got
           switch(Order_Info.infotype){
              case INFO_BIN_STATE:
                bytecount=as_read_bnum(&(Order_Info.value.ui8val), 1);
                if(bytechecksum(bytecount, 1, "received")){
                   sprintf(log_message,": Checksum error receiving binary ui8val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              default: // This should never happen
              break;
            }
         break;
     
         default:
         break;
        }  // End of: switch(Order_ID.otype)

     break;
     case ACT_MLEVEL:
      switch(Order_ID.otype){

         case ORDER_GET:
           // Send back the information got
           switch(Order_Info.infotype){
              case INFO_ML_STATE:
              case INFO_ML_TIMER:
                bytecount=as_read_bnum(&(Order_Info.value.ui8val), 1);
                if(bytechecksum(bytecount, 1, "received")){
                   sprintf(log_message,": Checksum error receiving M-level ui8val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              case INFO_ML_FREQ:
                bytecount=as_read_bnum(&(Order_Info.value.ui32val), 4);
                if(bytechecksum(bytecount, 4, "received")){
                   sprintf(log_message,": Checksum error receiving M-level ui32val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              default: // This should never happen
              break;
            }
         break;
     
         default:
         break;
        }  // End of: switch(Order_ID.otype)
     break;

     case ACT_SERVO:
      switch(Order_ID.otype){

         case ORDER_GET:
           // Send back the information got
           switch(Order_Info.infotype){
              case INFO_SRV_STATE:
              case INFO_SRV_TIMER:
                bytecount=as_read_bnum(&(Order_Info.value.ui8val), 1);
                if(bytechecksum(bytecount, 1, "received")){
                   sprintf(log_message,": Checksum error receiving servo ui8val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              case INFO_SRV_AFB:
              case INFO_SRV_SPOS:
                bytecount=as_read_bnum(&(Order_Info.value.ui32val), 4);
                if(bytechecksum(bytecount, 4, "received")){
                   sprintf(log_message,": Checksum error receiving servo ui32val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              default: // This should never happen
              break;
            }
         break;
     
         case ORDER_SERVO_MOVE:
            // Extract the 1-byte PMR pending flag
            bytecount=as_read_bnum(&(sv_Accparam[ACT_SERVO].pmr_pending), 1);
            if(bytechecksum(bytecount, 1, "received")){
               sprintf(log_message,": Checksum error receiving servo pmr_pending.\n");
               show_message(log_message,fnn,MT_ERR,0);
               retval = PARDU_ERR_CHKSUM; goto end_send_order;
              }
         break;

         default:
         break;
        }  // End of: switch(Order_ID.otype)
     break;

     case ACT_SENSOR:
      switch(Order_ID.otype){

         case ORDER_GET:
           // Send back the information got
           switch(Order_Info.infotype){
              case INFO_SEN_VAL:
                bytecount=as_read_bnum(&(Order_Info.value.ui32val), 4);
                if(bytechecksum(bytecount, 4, "received")){
                   sprintf(log_message,": Checksum error receiving sensor ui32val.\n");
                   show_message(log_message,fnn,MT_ERR,0);
                   retval = PARDU_ERR_CHKSUM; goto end_send_order;
                  }
              break;
              default: // This should never happen
              break;
            }
         break;
         default:
         break;
        }  // End of: switch(Order_ID.otype)
     break;

     default:
     break;
    } // End of: switch(Order_ID.actype){

   } else retval = error_state;


 // That should be all for this report. So look for the termination signal:
 receptval=as_read_sigstr(bytes_received, PARD_REPORT_END);
 if(receptval){
          sprintf(log_message,": Error: %s\n,",bytes_received);
          show_message(log_message,fnn,MT_ERR,0);
          retval = PARDU_ERR_NOTERM; goto end_send_order;
        } else {
   sprintf(log_message,": PARD_Report_end.\n");
   show_message(log_message,fnn,MT_ERR,0);
  }

 // Message cycle complete. Increment serial message counter:
 CurrMsg++;

end_send_order:
 return retval;
}
#endif

//---------------------------------------------------------------------------//.
 //                            Arduino serial comms                         //.
  //-----------------------------------------------------------------------//.
#endif

  //-----------------------------------------------------------------------\\.
 //                       Logging and local messaging                       \\.
//---------------------------------------------------------------------------\\.
#ifdef PARDUS


#if IS_WINDOWS_NATIVE
void show_message(const char *msg,const char *title,int mtype,int interactive)
// All written information to user is channelled through this function
// MS Windows version: security notices (MT_SEC) are routed to the Windows
// Event Log (Application log, source "pardserver", warning level) with
// win_syslog() instead of to the Linux syslog. Everything else is identical.
{
 char endstr[3];
 int is_error;

 // Verbosity RULE 1: Level 0 means show none and log none. Instant bailout.
 if(Verbosity <= 0){
   if(mtype == MT_QUIT) goto end_program; // Must still allow safe shutdown
   if(mtype != MT_SEC)  return; // Must allow security notices.
 }

 // Establish whether this specific message packet is flagged as an error type
 is_error = (mtype == MT_ERR    ||
             mtype == MT_QUIT   ||
             mtype == MT_SESS_E ||
             mtype == MT_SEC);

 // Verbosity RULE 2: If Level 1 is active, completely block all non-errors
 if (Verbosity == 1 && !is_error) {
     return;
 }

 sprintf(endstr,"%s",(interactive)?"":"\n");

 switch(mtype){
    case MT_INFO : // Equivalent of printf - usually for info only
        if (FP_Sessionlog) fprintf(FP_Sessionlog,"%s%s%s",title,msg,endstr);
                                         else printf("%s%s%s",title,msg,endstr);
    break;
    case MT_ERR  : // Equivalent of fprintf(sderr, - to indicate an error
        if (FP_Sessionlog) fprintf(FP_Sessionlog,"%s%s%s",title,msg,endstr);
                                 else fprintf(stderr,"%s%s%s",title,msg,endstr);
    break;
    case MT_SEC  : // Equivalent of fprintf(sderr, - to indicate an error
        // Unlike MT_ERR, security notices are printed to BOTH the log (if it
        // exists) AND terminal regardless of the log file existence.
        if (FP_Sessionlog) fprintf(FP_Sessionlog,"%s%s%s",title,msg,endstr);
        fprintf(stderr,"%s%s%s",title,msg,endstr);
        // ROUTE TO THE WINDOWS EVENT LOG (the counterpart of the Linux syslog)
        // Combines title and message into a unified, clean system log string
        win_syslog(EVENTLOG_WARNING_TYPE, "SECURITY NOTICE: %s%s", title, msg);
    break;
    case MT_QUIT : // Message given upon program quit request.
        if (FP_Sessionlog) fprintf(FP_Sessionlog,"%s%s%s",title,msg,endstr);
                                         else printf("%s%s%s",title,msg,endstr);
    goto end_program;
    case MT_SESS_S : // Message to server console [Yellow] - from server
    case MT_SESS_C : // Message to server console [Cyan]   - from client
    case MT_SESS_T : // Message to server console [White]  - Translation
    case MT_SESS_E : // Message to server console [Red]    - Comms error
        if (FP_Sessionlog) fprintf(FP_Sessionlog,"%s%s",title,msg);
                                                  else printf("%s%s",title,msg);
    break;
    default: // Only a programming error can get here. It is a terminal event.
        fprintf(stderr,"Invalid message type received. This is a programming error. The program will terminate now.\n");
    goto end_program;
  }

 return;

 end_program:
   tidy_up();
   exit(1);
}
#else
void show_message(const char *msg,const char *title,int mtype,int interactive) 
// All written information to user is channelled through this function
{
 char endstr[3];
 int is_error;

 // Verbosity RULE 1: Level 0 means show none and log none. Instant bailout.
 if(Verbosity <= 0){
   if(mtype == MT_QUIT) goto end_program; // Must still allow safe shutdown
   if(mtype != MT_SEC)  return; // Must allow security notices.
 }

 // Establish whether this specific message packet is flagged as an error type
 is_error = (mtype == MT_ERR    || 
             mtype == MT_QUIT   ||
             mtype == MT_SESS_E ||
             mtype == MT_SEC);

 // Verbosity RULE 2: If Level 1 is active, completely block all non-errors
 if (Verbosity == 1 && !is_error) {
     return;
 }

 sprintf(endstr,"%s",(interactive)?"":"\n");
 
 switch(mtype){
    case MT_INFO : // Equivalent of printf - usually for info only
        if (FP_Sessionlog) fprintf(FP_Sessionlog,"%s%s%s",title,msg,endstr);
                                         else printf("%s%s%s",title,msg,endstr);
    break;
    case MT_ERR  : // Equivalent of fprintf(sderr, - to indicate an error
        if (FP_Sessionlog) fprintf(FP_Sessionlog,"%s%s%s",title,msg,endstr);
                                 else fprintf(stderr,"%s%s%s",title,msg,endstr);
    break;
    case MT_SEC  : // Equivalent of fprintf(sderr, - to indicate an error
        // Unlike MT_ERR, security notices are printed to BOTH the log (if it
        // exists) AND terminal regardless of the log file existence.
        if (FP_Sessionlog) fprintf(FP_Sessionlog,"%s%s%s",title,msg,endstr);
        fprintf(stderr,"%s%s%s",title,msg,endstr);
        // ROUTE TO LINUX SYSLOG
        // Combines title and message into a unified, clean system log string
        syslog(LOG_WARNING, "SECURITY NOTICE: %s%s", title, msg);
    break;
    case MT_QUIT : // Message given upon program quit request.
        if (FP_Sessionlog) fprintf(FP_Sessionlog,"%s%s%s",title,msg,endstr);
                                         else printf("%s%s%s",title,msg,endstr);
    goto end_program;
    case MT_SESS_S : // Message to server console [Yellow] - from server
    case MT_SESS_C : // Message to server console [Cyan]   - from client
    case MT_SESS_T : // Message to server console [White]  - Translation
    case MT_SESS_E : // Message to server console [Red]    - Comms error
        if (FP_Sessionlog) fprintf(FP_Sessionlog,"%s%s",title,msg);
                                                  else printf("%s%s",title,msg);
    break;
    default: // Only a programming error can get here. It is a terminal event.
        fprintf(stderr,"Invalid message type received. This is a programming error. The program will terminate now.\n");
    goto end_program;
  } 

 return; 
 
 end_program:
   tidy_up();
   exit(1);
}
#endif

static const char *get_os_temp_dir(char *buf, size_t len)
{
#if defined(_WIN32) || IS_WINDOWS_NATIVE
    DWORD n = GetTempPathA((DWORD)len, buf);
    if (n > 0 && n < len) {
        // Strip trailing backslash if present (except root like "C:\")
        if (n > 3 && (buf[n - 1] == '\\' || buf[n - 1] == '/')) {
            buf[n - 1] = '\0';
        }
        return buf;
    }
    const char *env = getenv("TEMP");
    if (!env || !*env) env = getenv("TMP");
    if (!env || !*env) env = "C:\\Windows\\Temp";
    snprintf(buf, len, "%s", env);
    return buf;
#else
    const char *env = getenv("TMPDIR");
    if (!env || !*env) env = getenv("TMP");
    if (!env || !*env) env = getenv("TEMP");
    if (!env || !*env) env = "/tmp";
    snprintf(buf, len, "%s", env);
    size_t n = strlen(buf);
    if (n > 1 && buf[n - 1] == '/') buf[n - 1] = '\0';
    return buf;
#endif
}

FILE *fopen_path_cwd_or_tmp(const char *filepath, const char *mode, char *resolved_path, size_t resolved_len)
// 1. Attempts to open `filepath` directly.
// 2. If that fails and `filepath` included a directory prefix, extracts the basename
//    and attempts to open it in the current working directory (CWD).
// 3. If that also fails, attempts to open the basename in the OS temp directory.
// 4. Records the actual opened path in `resolved_path` (if non-NULL), or returns NULL if all fail.
{
    if (filepath == NULL || filepath[0] == '\0') return NULL;

    // 1. Attempt the given path directly
    FILE *fp = fopen(filepath, mode);
    if (fp != NULL) {
        if (resolved_path != NULL && resolved_len > 0) {
            snprintf(resolved_path, resolved_len, "%s", filepath);
        }
        return fp;
    }

    // Extract basename (check both '/' and '\' separators)
    const char *slash = strrchr(filepath, '/');
    const char *bslash = strrchr(filepath, '\\');
    const char *sep = (slash > bslash) ? slash : bslash;
    const char *base = (sep != NULL) ? (sep + 1) : filepath;

    if (base[0] == '\0' || strcmp(base, ".") == 0 || strcmp(base, "..") == 0) {
        return NULL;
    }

    // 2. Fall back to CWD (only if `filepath` had a directory prefix)
    if (base != filepath) {
        fp = fopen(base, mode);
        if (fp != NULL) {
            if (resolved_path != NULL && resolved_len > 0) {
                snprintf(resolved_path, resolved_len, "%s", base);
            }
            return fp;
        }
    }

    // 3. Fall back to the system temporary directory
    char tmp_dir[PATH_MAX];
    char tmp_file[PATH_MAX];
    get_os_temp_dir(tmp_dir, sizeof(tmp_dir));
#if defined(_WIN32) || IS_WINDOWS_NATIVE
    snprintf(tmp_file, sizeof(tmp_file), "%s\\%s", tmp_dir, base);
#else
    snprintf(tmp_file, sizeof(tmp_file), "%s/%s", tmp_dir, base);
#endif

    fp = fopen(tmp_file, mode);
    if (fp != NULL && resolved_path != NULL && resolved_len > 0) {
        snprintf(resolved_path, resolved_len, "%s", tmp_file);
    }

    return fp;
}

void close_session_logfile(void)
// Closes user session log file (if open) and sets FP_Sessionlog to NULL.
{
    if (FP_Sessionlog!=NULL) {
        fflush(FP_Sessionlog); fclose(FP_Sessionlog);
    }
    FP_Sessionlog = NULL;
}

void open_session_logfile(void)
// Opens user session log file for appending. The initial opening of the log
// file is done on successful login to server (where Log_SessionFilename is also
// set)
{
    if (sv_Client_logged_in) {
        if (FP_Sessionlog != NULL) close_session_logfile();
        char resolved_log[PATH_MAX] = {0};
        FP_Sessionlog = fopen_path_cwd_or_tmp(Log_SessionFilename, "ab", resolved_log, sizeof(resolved_log));
        if (FP_Sessionlog != NULL && resolved_log[0] != '\0') {
            if (strcmp(Log_SessionFilename, resolved_log) != 0) {
                char msgtxt[PATH_MAX + 64];
                snprintf(Log_SessionFilename, sizeof(Log_SessionFilename), "%s", resolved_log);
                snprintf(msgtxt, sizeof(msgtxt), "Session log file redirected to: %s\n", Log_SessionFilename);
                show_message(msgtxt, "FYI: ", MT_INFO, 0);
            }
        }
    }
 return;
}


int print_any_error(char *msghdr)
// Checks the value of Pard_Err and prints an error message if it is not
// PARD_OK. If it is PARD_OK this function returns 0, otherwise 1.
{
 if(Pard_Err!=PARD_OK){
   show_message(Pard_err_str(),msghdr,MT_ERR,1);
   return 1;
  }
 return 0;
}

//---------------------------------------------------------------------------//.
 //                       Logging and local messaging                       //.
  //-----------------------------------------------------------------------//.
#endif

  //-----------------------------------------------------------------------\\.
 //                          Server-side Auto-focus                         \\.
//---------------------------------------------------------------------------\\.
#ifdef PARDUS

void af_vars_init(void)
// Initialisations for Autofocus variables
{
 Autofocussing = 0;
 AF_issz = 1.0;
 AF_centre = 0.0;
 AF_dirn_up = DIRN_UNS;
 AF_max_retries = 0;
 AF_max_itn=64;
 AF_timeout=0;
 AF_maxdiff=0.0;
 AF_min_peak=0.0;
 AF_tc_threshlow=-1.0;
 AF_tc_threshupp=-1.0;
 AF_tc_curr = 0.0;
 AF_ref_set = 0;
 AF_lk_maxdiff = AF_maxdiff;
 AF_lk_peak = AF_min_peak;
 AF_colchan = CCHAN_G;
return;
}

uint8_t af_get_current_tc(char *errmsg)
// Retrieve the current tissue content value from the pardcap server.
// Return 0 on success.
// Return 1 on failure with some error cause text printed into errmsg
{
 int cmdstatus,sending_bloberr;
 char cmdtriad[TRIADMAX];

 cmdstatus= sv_process_cuss(PS_CUSS_GTCVL,"get_preview_tc",
                           "on",errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_ONFF);

 if(cmdstatus!=CMD_ERR){
   // Does the blob have TCBYTES bytes (as it should)?
   if(ps_Instruction.blen!=TCBYTES){
     sprintf(errmsg,"wrong size blob (%"PRId32,ps_Instruction.blen);
     return 1;
    }    
   // If we get here we got the tissue content blob, so put it in AF_tc_curr
   memcpy(&AF_tc_curr,ps_Instruction.blob,ps_Instruction.blen);
  } else return 1;

 return 0;
}

uint8_t af_get_current_focus(char *errmsg, double *fval, uint8_t colchan)
// Retrieve the current value of the focus function from the pardcap server.
// Return 0 on success.
// Return 1 on failure with some error cause text printed into errmsg
{
 int cmdstatus, sending_bloberr;
 char cmdtriad[TRIADMAX];
 unsigned char *ucblob;
 int offset;

 cmdstatus = sv_process_cuss(PS_CUSS_GPFOC, "get_preview_focus",
                       NULL, errmsg, cmdtriad, &sending_bloberr, CUSS_VAL_NONE);

 if(cmdstatus == CMD_ERR) return 1;

 // Validate Blob Size
 if(ps_Instruction.blen != FOCUSBYTES) {
    sprintf(errmsg, "Received blob is wrong size (%"PRId32") for focus package", 
             ps_Instruction.blen);
    return 1;
 }

 // Determine the byte offset based on channel
 // Assumes CCHAN_R=0, CCHAN_G=1, CCHAN_B=2. If your constants are different, 
 // keep your switch statement logic instead!
 switch(colchan) {
   case CCHAN_R: offset = 0;  break;
   case CCHAN_G: offset = 8;  break;
   case CCHAN_B: offset = 16; break;
   default:
     sprintf(errmsg, "af_get_current_focus: Unrecognised colour channel (%d)",
                                                                       colchan);
     return 1;
 }

 ucblob = (unsigned char *)ps_Instruction.blob;
 
 // Direct extraction of the requested channel
 *fval = ps_uc_to_dou(ucblob + offset);

 return 0;
}

int af_tc_threshold_check(void)
// Tests AF_tc_curr against the current upper and lower tissue content threshold
// limits. Upper threshold test is ignored if AF_tc_threshupp < 0.
// Returns 1 if threshold(s) are passed (tissue is present).
// Returns 0 if threshold(s) fail (this is a blank field).
{
 uint8_t score, passmark;

 score=0;
    
 // Lower bound thresholding is obligatory (we would not reach this function is
 // AF_tc_threshlow was negative):
 if(AF_tc_curr>=AF_tc_threshlow) score++;
 passmark=1;  

 // Only use upper bound thresholding if the user chooses to implement it
 if(AF_tc_threshupp>=0.0){
   if(AF_tc_curr<=AF_tc_threshupp) score++;
   passmark++;
 }
  
 // Only declare 'tissue present' if all used thresholds permit it:
 if(score==passmark) return 1;
 
 return 0;
}

int af_auto_focus(char *aferr)
// Performs the autofocus action
// Returns a code (one of the 'AF_...' #defines, which see).
// If return value is AF_IMPOSSIBLE or AF_MOVEERR then aferr is set to
// indicate the reason.
{
 char msgtxt[128];
 uint32_t ccrit_maxitn;  // Maximum AF search steps before giving up.
 double   ccrit_maxdiff; // If the new 'best focus' is more than this amount
                         // different to the previous best focus, then fail.
 double ccrit_falsepeak; // If the function value for the new 'best focus' is
                         // less than this value then it is considered to be a
                         // local (false) peak and so AF has failed.
 long ccrit_tmo;         // If the search lasts longer this this time (in ms)
                         // then it will fail. 
 uint32_t ccrit;         // Stores the reason for convergence.
 uint32_t itern;         // Keeps count of how many search steps we have made.
 uint8_t  currdirn;      // The current direction of travel for the Z stage.
 double   maxfval;       // A value that represents how good the best focus is.
 double   max_fvpos;     // The Z location of the maximum fval.
 uint32_t reqsteps;      // For motion control.
 uint32_t reqsteps_orig; // For motion control.
 double   fv1,fv2,fv3;   // Store the focus values for hill-climbing search.
 double   zp1,zp2,zp3;   // Z positions corresponding to fv1, fv2, fv3
 int false_peak;         // Flag to set if test shows best focus found is likely
                         // to be a false result.
 int returnval;          // Value the function returns: 0 = success, anything
                         // else is a failure of some kind (see #defines).
 double afstart;         // The starting point of the search.
 long elapsed_sec;       // To implement the timeout CCRIT
 long elapsed_nsec;      // To implement the timeout CCRIT
 double total_elapsed;   // To implement the timeout CCRIT
 struct timespec start;  // To implement the timeout CCRIT
 struct timespec now;    // To implement the timeout CCRIT
 uint8_t dirn_up;        // What direction is 'up' - must be 0 or 1
 uint8_t dirn_down;      // The complement to dirn_up
 uint8_t colchan;        // What colour channel to use for the focus value
 uint8_t on_the_way_up;  // Unilateral adaptive acceleration flag.

 // If using tissue content thresholding then we don't do autofocus unless there
 // is tissue under the scope so check for this.
 if(USING_TC_THRESHOLDING){
  if(af_get_current_tc(msgtxt)){
    sprintf(aferr,"Failed to get the tissue content value (%s). Cannot do AF.",
                                                                        msgtxt);
    return AF_IMPOSSIBLE;
  // Return without AF and without error if there is no tissue to focus on.
  } else if(af_tc_threshold_check()==0) return AF_SUCCESS; 
 }

 // Convergence criteria initialise to values provided by the client
 ccrit_maxdiff   = AF_maxdiff;
 ccrit_maxitn    = AF_max_itn;
 ccrit_falsepeak = AF_min_peak;
 ccrit_tmo       = AF_timeout;
  
 // I have the following motion-related functions at my disposal:
 // sv_stepper_move_to(uint8_t id, double position, char *errmsg)
 // sv_stepper_move_by(uint8_t id, uint32_t dirsteps, char *errmsg)
 // sv_stepper_set_dirn(uint8_t id, uint8_t dirn, char *errmsg)
 // 
 // All of them return CMD_OK if all went well or CMD_ERR with a message in
 // errmsg if something went wrong. Also, the move functions will not return
 // until the motion is complete or there is an error. I do not have to
 // implement a 'look for PMR to see if the motion has finished' loop as I would
 // in a PCS client side move command because sv_stepper_move_by (which
 // sv_stepper_move_to uses to do its motion) waits for the PMR before
 // returning.
 //
 // When using move_to or move_by I need to:
 // 1. If using sv_stepper_move_by, ensure the motor is pointing in the required
 //    direction of motion first by calling sv_stepper_set_dirn (this is not
 //    required for sv_stepper_move_to because it calculates and sets the
 //    required direction in order to get to wherever 'to' is.
 // 2. Ensure all distances (for move by) and coordinate positions (for move_to)
 //    are already in units of the motor's current microstep size (not real
 //    world calibrated units or units of another step size).

 // Some macros to make the AF search code maths more readable:

#define MOVE_Z_BY(steps) \
   do { \
       if (sv_stepper_move_by(ACT_ID_STAGE_Z, (steps), 0, aferr) == CMD_ERR) { \
           returnval = AF_MOVEERR; \
           goto cleanup_af; \
       } \
   } while (0)

#define MOVE_Z_TO(coord) \
   do { \
       if (sv_stepper_move_to(ACT_ID_STAGE_Z,(coord),0,aferr) == CMD_ERR) { \
           returnval = AF_MOVEERR; \
           goto cleanup_af; \
       } \
   } while (0)

#define SET_Z_DIRN(dirn) \
   do { \
       if (sv_stepper_set_dirn(ACT_ID_STAGE_Z,(dirn),aferr) == CMD_ERR) { \
           returnval = AF_MOVEERR; \
           goto cleanup_af; \
       } \
   } while (0)

#define REVERSE_DIRN() \
    do { \
        currdirn = (currdirn == dirn_down) ? dirn_up : dirn_down; \
        if (sv_stepper_set_dirn(ACT_ID_STAGE_Z, currdirn, aferr) == CMD_ERR) { \
            returnval = AF_MOVEERR; \
            goto cleanup_af; \
        } \
    } while (0)

#define GET_FOCUS_AND_UPDATE(fv_ptr, zp_var) \
    do { \
        if (af_get_current_focus(aferr, (fv_ptr), colchan)) { \
            returnval = AF_IMPOSSIBLE; \
            goto cleanup_af; \
        } \
        (zp_var) = PMRep3; \
        if (*(fv_ptr) > maxfval) { \
            maxfval = *(fv_ptr); \
            max_fvpos = (zp_var); \
        } \
    } while (0)


 // Set the startpoint Z position to AF_issz steps above the current centre Z
 // position (so the starting focal plane is *below* the focal plane represented
 // by AF_centre).
 // We choose this because it it more likely that there will be in-focus
 // artefacts (specks on the coverslip, small areas of puckered or folded
 // tissue) just above the focal plane of the majority of the section than below
 // it. So if we set the starting point to a slightly higher focal plane (i.e.
 // moved the stage downwards to begin the search) then those artefacts would be
 // in focus and so create the best signal that the search will latch onto
 // before it sees the main bulk of the tissue. This is more likely a problem
 // for very high magnification imaging like x40 and oil objectives. For lower
 // power objective you can start the search at the best known focal plane and
 // not bother with this initial displacement.

 // It is assumed the stage is homed to the lower Z limit switch so 0 is the
 // lowest position the specimen can be w.r.t. the objective. Thus to make the
 // starting focal plane below the current best or 'centre' focal place we must
 // add AF_issz to it (to elevate the specimen closer to the objective so the
 // focus of the objective is below the centre position):
  afstart=AF_centre + (AF_issz)/2; 



 // DEBUG
 printf("Starting at Z = %.9g\n",afstart);
 // DEBUG

 // Prepare for the AF search by moving the stage to the afstart position.
 MOVE_Z_TO(afstart);

 sprintf(msgtxt,"Now at: [%"PRIu32"]",PMRep3);
 show_message(msgtxt,"FYI: ",MT_INFO,0);
 
 // DEBUG
 printf("%s\n",msgtxt);
 // DEBUG
  
 // Bootstrap the iterative process:

 // Establish the direction values:
 dirn_up = AF_dirn_up; // Use the value set in the af_setup command
 dirn_down = 1 - dirn_up;

 // Establish what colour channel to collect the focus value from
 colchan = AF_colchan;

 // Initialise the current direction of travel - we chose to start by moving the
 // stage downwards because we started by elevating the specimen above the
 // centre point. Howevermoving the stage 'downards' is equivalent to moving the
 // objective 'upward' and the direction for Z motion relates to the objective's
 // relative motion (dirn_down increases step count - it brings the specimen
 // closer to the objective and therefore pushes the objective's focal plane
 // downwards 'into' the specimen towards the bottom of the slide. Thus we set
 // the initial direction as dirn_up. This reduces the stepcount with each move
 // thereby counteracting the offset we added above: 
 currdirn  = dirn_up;
 SET_Z_DIRN(currdirn);

 // Bootstrap the requested steps to the current AF_issz value (auto-focus 
 // initial step size - set by the user via the GUI or program script).
 // This converts AF_issz from atomic (1/256) microsteps to current msdenom
 // steps:
 reqsteps = (uint32_t)(0.5 + (AF_issz / Stepperlist[ACT_ID_STAGE_Z]->onestep));
 if(reqsteps<8) reqsteps = 8;  
 
 // Let other functions know we are doing autofocus.
 Autofocussing = 1;

 // Start the clock
 clock_gettime(CLOCK_MONOTONIC, &start);

  
 // Begin the search for best focus
 sprintf(msgtxt,"AF begin: [iteration][currzpos]: [zp1][fv1], [zp2][fv2], zp3[fv3]");
 show_message(msgtxt,"FYI: ",MT_INFO,0);
 
 // DEBUG
 printf("%s\n",msgtxt);
 // DEBUG
 
 ccrit=0; // Initialise the convergence criteria reporter variable
 
 maxfval = 0.0; // This will contain the maximum value of the af function
 max_fvpos=0.0; // This will contain the Z position at which maxfval is recorded
    
 // --- BOOTSTRAP (Before the loop) ---
 MOVE_Z_BY(reqsteps); GET_FOCUS_AND_UPDATE(&fv1, zp1);
 MOVE_Z_BY(reqsteps); GET_FOCUS_AND_UPDATE(&fv2, zp2);

 // INITIAL SLOPE CHECK: If fv2 < fv1, we are going the wrong way. 
 if (fv2 < fv1) {
     // Move back past start and re-bootstrap
     REVERSE_DIRN();
     MOVE_Z_BY(reqsteps * 2); GET_FOCUS_AND_UPDATE(&fv1, zp1);
     MOVE_Z_BY(reqsteps);     GET_FOCUS_AND_UPDATE(&fv2, zp2);
 }

 // --- MAIN SEARCH LOOP ---
 reqsteps_orig = reqsteps;
 on_the_way_up=1; // We are at the foot of the hill

 for(itern=0;;itern++){
    // 1. Get the third data point
    MOVE_Z_BY(reqsteps); GET_FOCUS_AND_UPDATE(&fv3, zp3);
    reqsteps = reqsteps_orig;

    // DEBUG
    snprintf(msgtxt, sizeof(msgtxt), 
        "AF[%"PRIu32"][%"PRIu32"]: [%.9g][%8.3f], [%.9g][%8.3f], [%.9g][%8.3f]",
        itern, PMRep3, 
        zp1, fv1, 
        zp2, fv2, 
        zp3, fv3);
    printf("%s\n",msgtxt);
    // DEBUG   

    // 2. WE HAVE FOUND A PEAK BRACKET
    if(fv2 > fv1 && fv2 > fv3) { // if ( (/\) )
        // If our step size is still large, halve it and keep going.
        if (reqsteps > 8) { 
            on_the_way_up=0; // The hill has been climbed - disable acceleration
            reqsteps /= 2;
            reqsteps_orig = reqsteps; // Reset orig
            // Jump back to the start of this bracket to refine it
            MOVE_Z_TO(zp1); GET_FOCUS_AND_UPDATE(&fv1, zp1);
            MOVE_Z_BY(reqsteps); GET_FOCUS_AND_UPDATE(&fv2, zp2);
            // Continue up the hill with our smaller steps...
            goto assess_ccrit; 
        } else {
            // Steps are small enough. We are at the best peak.
            reqsteps /= 2; // This ensures we exit at ccrit (i.e. reqsteps < 8)
            goto assess_ccrit; 
        }
    }

    // 3. WE HAVE OVERSHOT THE PEAK: REVERSE AND SLOW DOWN
    if (fv1 >= fv2 && fv2 >= fv3 && fv1 > fv3) { // if ( (\) or (\_) or (¯\) )
        on_the_way_up=0; // The hill has been climbed - disable acceleration
        REVERSE_DIRN();
        // We probably got here because we leapfrogged over the peak so halve
        // the step size for the journey back so we don't just leapfrong back
        // and forth over the peak but home in on it.
        reqsteps /= 2;
        reqsteps_orig = reqsteps; // Reset orig
        MOVE_Z_BY(reqsteps); GET_FOCUS_AND_UPDATE(&fv1, zp1);
        MOVE_Z_BY(reqsteps); GET_FOCUS_AND_UPDATE(&fv2, zp2);
        goto assess_ccrit; 
    }

    // 4. WE HAVE HIT NOISE OR A MINIMUM
    if (fv2 <= fv1 && fv2 <= fv3) { // if ( (\/) or (__) )
       // Move faster to get out of the rut.
       // But only if we are still on the first approach to the peak
       if(on_the_way_up){
         reqsteps = (uint32_t)(0.5 + (1.5*(double)reqsteps) );
         MOVE_Z_BY(reqsteps); GET_FOCUS_AND_UPDATE(&fv1, zp1);
         MOVE_Z_BY(reqsteps); GET_FOCUS_AND_UPDATE(&fv2, zp2);
         goto assess_ccrit; 
      }
     } 

    // 5. WE ARE GOING THE RIGHT WAY BUT HAVEN'T FOUND A PEAK
    // if( (/) or (/¯) or (_/) )
    // Slide and carry on:
    fv1 = fv2; zp1 = zp2;
    fv2 = fv3; zp2 = zp3;
    
    // 6. CONVERGENCE CRITERIA
    assess_ccrit:
    if(itern >= ccrit_maxitn){ccrit=1; break;}
    if(reqsteps < 8){ccrit=3; break;}
    
    clock_gettime(CLOCK_MONOTONIC, &now);
    elapsed_sec  = now.tv_sec  - start.tv_sec;
    elapsed_nsec = now.tv_nsec - start.tv_nsec;
    total_elapsed = (double)(elapsed_sec * 1000.0) + (double)(elapsed_nsec / 1000000.0);  
    if(total_elapsed > ccrit_tmo){ccrit=4; break;}
 }
 
 
 sprintf(msgtxt,"AF complete. Iteration= %"PRIu32" MaxFval = %.9g, Posn= %.9g",
                                                       itern,maxfval,max_fvpos);
 show_message(msgtxt,"FYI: ",MT_INFO,0);
 // DEBUG
 printf("%s\n",msgtxt); fflush(stdout);
 // DEBUG

 false_peak = 0;
 
 AF_lk_maxdiff=fabs(AF_centre-max_fvpos);
 if(ccrit_maxdiff>=1.0 && (AF_lk_maxdiff>ccrit_maxdiff)) ccrit=2;
 
 switch(ccrit){
   case 0:
    sprintf(msgtxt,"FAILED - error");
   break;
   case 1:
    sprintf(msgtxt,"FAILED - maxitn");
   break;
   case 2:
    sprintf(msgtxt,"FAILED - maxdiff");
   break;
   case 3:
    // The AF procedure has exited without error but it may not have found true
    // focus. Check if this is the case...
    if(maxfval < ccrit_falsepeak){
        false_peak = 1;
        sprintf(msgtxt,"FAILED - false peak found.");
        AF_lk_peak = maxfval;
    } else {
       // Update the new default centre of focus to the found value
       AF_centre = max_fvpos;
       sprintf(msgtxt,"SUCCESS in %"PRIu32" steps, new Z=%.9g",itern,AF_centre);
    }
   break;
   case 4:
    sprintf(msgtxt,"FAILED - timeout");
   break;
   default: // This should not happen unless there is a bug.
    sprintf(msgtxt,"FAILED - Program Error");  
    ccrit = 0; // Ensures this function returns an error code.
   break;
 }
 show_message(msgtxt,"FYI: AF CCRIT: ",MT_INFO,0);

 // DEBUG
 printf("%s\n",msgtxt);
 printf("Moving Z to %.9g\n",AF_centre); fflush(stdout);
 // DEBUG

 // move the stage in Z to go to the best focus position
 MOVE_Z_TO(AF_centre);

 // Set the return value according to how successful the result was
 returnval = AF_SUCCESS;
 switch(ccrit){
   case 0: returnval = AF_CCRITERR;  break;
   case 1: returnval = AF_CCRITITN;  break;
   case 2: returnval = AF_CCRITMDF;  break;
   case 3: 
       if(false_peak == 1) returnval = AF_FALSEPEAK; 
       else returnval = AF_SUCCESS; 
   break;
   case 4: returnval = AF_CCRITTMO;  break;
   default: break; // Success.
  }

 

cleanup_af:

 // Let other functions know that Autofocussing is over 
 Autofocussing = 0;
 // Update the local struct copy of the current position:
 Stepperlist[ACT_ID_STAGE_Z]->stepcount=PMRep3;

 // DEBUG
 // Timing info to compare algorithms
      // Check elapsed time
     clock_gettime(CLOCK_MONOTONIC, &now);
     elapsed_sec  = now.tv_sec  - start.tv_sec;
     elapsed_nsec = now.tv_nsec - start.tv_nsec;
     // Total_elapsed in milliseconds
     total_elapsed = (double)(elapsed_sec * 1000.0) + 
                     (double)(elapsed_nsec / 1000000.0);  
 printf("Elapsed time = %.9g ms\n",total_elapsed); fflush(stdout);

 return returnval;
}

//---------------------------------------------------------------------------//.
 //                          Server-side Auto-focus                         //.
  //-----------------------------------------------------------------------//.
#endif

  //-----------------------------------------------------------------------\\.
 //                             The PARD Server                             \\.
//---------------------------------------------------------------------------\\.
#ifdef PARDUS

void sv_fmtstr(int fmt, char *fstr) // fmtstr in pardclient (?not used there)
// save as format string provider - yuv, pgm, bm8, ppm, bmp, png, int, jpg or
// the raw doubles types: dou, do3
{
    switch(fmt){
    case SAF_YUYV: sprintf(fstr, "yuv"); break;
    case SAF_YP5:  sprintf(fstr, "pgm"); break;
    case SAF_BM8:  sprintf(fstr, "bm8"); break;
    case SAF_RGB:  sprintf(fstr, "ppm"); break;
    case SAF_BMP:  sprintf(fstr, "bmp"); break;
    case SAF_PNG:  sprintf(fstr, "png"); break;
    case SAF_INT:  sprintf(fstr, "int"); break;
    case SAF_JPG:  sprintf(fstr, "jpg"); break;
    case SAF_DOU:  sprintf(fstr, "dou"); break;
    case SAF_DO3:  sprintf(fstr, "do3"); break;
    }
    return;
}

int sv_blob_resize(size_t bsz)
// Change the size of the sv_Blob with housekeeping. Return 1 on failure.
{
 free(sv_Blob);
 sv_Blob=calloc(bsz,sizeof(unsigned char));
 if(sv_Blob==NULL){
    show_message(": Failed to get memory for the sv_Blob.",fnn,MT_ERR,1);
    sv_Blob=calloc(2,sizeof(unsigned char));
    if(sv_Blob==NULL){
      show_message(": Could not get even re-initialise the sv_Blob. This is a serious memory error. Please save your work and exit ASAP"
      " because the program has become unstable and may crash at any time without further notice.",fnn,MT_ERR,1);
      sv_Blobsize=0;
      return 1;
    } else  sv_Blobsize=2;
    return 1;
  }
 sv_Blobsize=bsz;
 return 0;
}

int sv_triad_resize(size_t bsz)
// Change the size of the sv_Triad with housekeeping. Return 1 on failure.
{
 free(sv_Triad);
 sv_Triad=calloc(bsz,sizeof(char));
 if(sv_Triad==NULL){
    show_message(": Failed to get memory for the sv_Triad.",fnn,MT_ERR,1);
    sv_Triad=calloc(2,sizeof(char));
    if(sv_Triad==NULL){
      show_message(": Could not get even re-initialise the sv_Triad. This is a serious memory error. Please save your work and exit ASAP"
      " because the program has become unstable and may crash at any time without further notice.",fnn,MT_ERR,1);
      return 1;
    }
    return 1;
  }
 return 0;
}

int sv_triadarg_resize(size_t bsz)
// Change the size of the sv_Triadarg with housekeeping.
// Return 1 on failure.
{
 free(sv_Triadarg);
 sv_Triadarg=calloc(bsz,sizeof(char));
 if(sv_Triadarg==NULL){
    show_message(": Failed to get memory for the sv_Triadarg.",fnn,MT_ERR,1);
    sv_Triadarg=calloc(2,sizeof(char));
    if(sv_Triadarg==NULL){
      show_message(": Could not get even re-initialise the sv_Triadarg. This is a serious memory error. Please save your work and exit ASAP"
      " because the program has become unstable and may crash at any time without further notice.",fnn,MT_ERR,1);
      return 1;
    }
    return 1;
  }
 return 0;
}

#if IS_WINDOWS_NATIVE
int sv_gen_keys(const char *filepath)
// Generate ZMQ curve encryption key pair (public and private).
// MS Windows version: on Windows umask() governs only the read-only attribute,
// not who may open a file, so the protection of the Linux version (umask 0077:
// the key file, created by root, gets mode 0600 - root only) is obtained with
// a security descriptor instead. The key file is created with a PROTECTED
// discretionary access control list (nothing is inherited from the folder)
// which grants full access to SYSTEM and to BUILTIN\Administrators - the
// Windows counterparts of root - and to nobody else: "D:P(A;;FA;;;SY)(A;;FA;;;BA)".
// This is also the access list that Microsoft's OpenSSH for Windows demands
// for a server's private host keys. (Key generation is offered only to an
// elevated Administrator, just as it is offered only to root on Linux.) As
// with umask on Linux this applies only when the file is created; an existing
// file keeps its access list. The file is then written exactly as on Linux
// (text mode via a C stream).
{
 char public_key[41];
 char secret_key[41];
 PSECURITY_DESCRIPTOR psd = NULL;
 SECURITY_ATTRIBUTES sa;
 HANDLE hfile;
 int fd;

 // Generate the cryptographic keys
 if (zmq_curve_keypair(public_key, secret_key) != 0) {
     fprintf(stderr, "Error: Could not generate Curve keys."
                                                    " Is libsodium active?\n\n");
     return 1;
 }


 // RESTRICT PERMISSIONS: Build the SYSTEM + Administrators only, non-
 // inheriting security descriptor (the Windows counterpart of root's 600) ...
 if(!ConvertStringSecurityDescriptorToSecurityDescriptorA(
                            "D:P(A;;FA;;;SY)(A;;FA;;;BA)", SDDL_REVISION_1,
                                                                 &psd, NULL)){
     fprintf(stderr, "Error: Could not build the restricted security descriptor"
                     " for %s (%s)\n\n", filepath, win_errstr(GetLastError()));
     return 1;
 }
 sa.nLength              = sizeof(sa);
 sa.lpSecurityDescriptor = psd;
 sa.bInheritHandle       = FALSE;


 // ... and open the destination deployment file with it (the Windows counterpart
 // of the umask + fopen(filepath, "w") pair: create or truncate for writing).
 hfile = CreateFileA(filepath, GENERIC_WRITE, 0, &sa, CREATE_ALWAYS,
                                                  FILE_ATTRIBUTE_NORMAL, NULL);
 LocalFree(psd); // The descriptor has been copied into the new file


 if (hfile == INVALID_HANDLE_VALUE) {
     fprintf(stderr, "Error: Could not create output file %s\n\n", filepath);
     return 1;
 }

 // Attach a C run-time stream to the Win32 handle so that the keys are written
 // exactly as on Linux (fprintf, text mode).
 fd = _open_osfhandle((intptr_t)hfile, _O_WRONLY | _O_TEXT);
 if (fd == -1) {
     CloseHandle(hfile);
     fprintf(stderr, "Error: Could not create output file %s\n\n", filepath);
     return 1;
 }
 FILE *f = _fdopen(fd, "w");
 if (!f) {
     _close(fd); // Also closes hfile
     fprintf(stderr, "Error: Could not create output file %s\n\n", filepath);
     return 1;
 }

 // Write keys exactly as init_or_load_server_keys will look for them
 fprintf(f, "%s\n", public_key);
 fprintf(f, "%s\n", secret_key);
 fclose(f);


 // Print the Public key to the screen so the Admin can give to the users
 printf("=============================================================\n");
 printf(" SUCCESS: '%s' generated and saved securely.\n",filepath);
 printf("=============================================================\n");
 printf(" SHARE THIS SERVER PUBLIC KEY WITH YOUR USERS:\n\n");
 printf(" %s\n\n", public_key);
 printf("=============================================================\n\n");

 return 0;
}
#else
int sv_gen_keys(const char *filepath)
// Generate ZMQ curve encryption key pair (public and private).
{
 char public_key[41];
 char secret_key[41];
 
 // Generate the cryptographic keys
 if (zmq_curve_keypair(public_key, secret_key) != 0) {
     fprintf(stderr, "Error: Could not generate Curve keys."
                                                    " Is libsodium active?\n\n");
     return 1;
 }


 // RESTRICT PERMISSIONS: Set process umask so any file created is restricted to
 // the owner only (0077). This forces the OS to apply 600 permissions the exact
 // instant fopen touches the disk.
 mode_t old_umask = umask(S_IRWXG | S_IRWXO); 


 // Open the destination deployment file
 FILE *f = fopen(filepath, "w");


 // Restore original system umask for subsequent server operations
 umask(old_umask);


 if (!f) {
     fprintf(stderr, "Error: Could not create output file %s\n\n", filepath);
     return 1;
 }

 // Write keys exactly as init_or_load_server_keys will look for them
 fprintf(f, "%s\n", public_key);
 fprintf(f, "%s\n", secret_key);
 fclose(f);


 // Print the Public key to the screen so the Admin can give to the users
 printf("=============================================================\n");
 printf(" SUCCESS: '%s' generated and saved securely.\n",filepath);
 printf("=============================================================\n");
 printf(" SHARE THIS SERVER PUBLIC KEY WITH YOUR USERS:\n\n");
 printf(" %s\n\n", public_key);
 printf("=============================================================\n\n");

 return 0;
}
#endif

int sv_setup_via_file(const char *config_file_path)
// Parses the administrator supplied configuration file at startup.
// Automatically initializes keys, databases, Arduino comms and any number of
// pardcap servers.
// Returns 0 on complete success
// Returns 1 on any fatal configuration error
{
 FILE *fp = fopen(config_file_path, "r");
 if(!fp){
   fprintf(stderr, "CONFIG ERROR: Configuration file not found at: %s\n", 
                                                              config_file_path);
   return 1;
  }

 char line[PATH_MAX + 64]; // Enough room for full file path + primary directive
 char *endptr;
 int line_num = 0;
 int config_ver_found = 0;
 int retval = 0; // Success till proven otherwise
 int key_file_set, user_database, bob_path_set, verbosity_found, pardport_found;
 int pp_strike_lim_found,pp_strikeout_delay_found,pp_min_length_found;
 int pp_needs_upper_found,pp_needs_lower_found,pp_needs_digit_found;
 int pp_needs_special_found,pp_expiry_timeout_found,pardname_found;
 int config_sum;
 int ver = 0;
 time_t parsed_value;
 size_t parsed_len;

 uint32_t idx;

 key_file_set    = 0;
 user_database   = 0;
 bob_path_set    = 0;
 pardport_found  = 0;
 pardname_found  = 0;
 verbosity_found = 0;
 pp_strike_lim_found      = 0;
 pp_strikeout_delay_found = 0;
 pp_min_length_found      = 0;
 pp_needs_upper_found     = 0;
 pp_needs_lower_found     = 0;
 pp_needs_digit_found     = 0;
 pp_needs_special_found   = 0;
 pp_expiry_timeout_found  = 0;

 fprintf(stdout, "FYI: Parsing configuration file: %s ...\n", config_file_path);

 while (fgets(line, sizeof(line), fp)) {
     line_num++;

     // 1. Strip trailing carriage returns and newlines safely
     line[strcspn(line, "\r\n")] = '\0';

     // 2. Skip empty lines or pure whitespace leading to comments
     char *start = line;
     while (*start && isspace((unsigned char)*start)) start++;
     if (*start == '\0' || *start == '#') continue;

     // 3. Extract the primary directive token (first word)
     char *token = start;
     char *delim = strpbrk(start, " \t");
     char *value = NULL;

     if(delim){
       *delim = '\0'; // Split the string
       value = delim + 1;
       // Trim leading whitespace from the value segment
       while (*value && isspace((unsigned char)*value)) value++;
       // ... and trailing whitespace (added 14.11.26: "bob_path auto " with a
       // trailing space or tab - easily left by copying from a manual or by an
       // editor - was taken as the port name "auto " instead of 'auto').
       {
        size_t vlen = strlen(value);
        while (vlen > 0 && isspace((unsigned char)value[vlen - 1])) value[--vlen] = '\0';
       }
       if (*value == '\0') value = NULL; // a directive followed only by blanks
      }

     // --- DIRECTIVE 1: HARD EXIT MARKER ---
     if (strcmp(token, "exit") == 0) {
         fprintf(stdout, "FYI: Reached 'exit' marker on line %d."
                                " Configuration parsing complete.\n", line_num);
         break;
     }

     // Every directive other than 'exit' needs a value (check added 14.11.26:
     // several directives - user_database, bob_path ... - used the value
     // without checking it, so such a line crashed the parser on a NULL
     // pointer instead of giving an error message).
     if(!value){
       fprintf(stderr, "CONFIG ERROR: Line %d specifies '%s' without a value.\n",
                                                             line_num, token);
       retval = 1; break;
      }

     // --- DIRECTIVE 2: FILE VERSION COMPATIBILITY CHECK ---
     if(strcmp(token, "config_version") == 0){
       if(!value){
         fprintf(stderr, "CONFIG ERROR: Line %d specifies 'config_version'"
                                               " without a value.\n", line_num);
         retval = 1; break;
        }
       ver = atoi(value);
       if(ver != 1){
         fprintf(stderr, "CONFIG ERROR: Unsupported configuration profile file"
                                       " version: '%s' (Expected 1).\n", value);
         retval = 1; break;
        }
       config_ver_found = 1;
       continue;
      }

     // Enforce that config_version must precede any resource allocations
     if(!config_ver_found){
       fprintf(stderr, "CONFIG ERROR: Parsable line %d reached before a valid"
                       " 'config_version' directive was declared.\n", line_num);
       retval = 1; break;
      }

     if(!value || *value == '\0'){
       fprintf(stderr, "CONFIG ERROR: Directive '%s' on line %d is missing its"
                                              " arguments.\n", token, line_num);
       retval = 1;  break;
     }

     // --- DIRECTIVE 3: ASYMMETRIC ENCRYPTION KEYS ---
     if(strcmp(token, "key_file") == 0){
       fprintf(stdout, "  > Loading encryption keys from: %s\n", value);
       if(sv_init_or_load_keys(value) != 0){
         fprintf(stderr, "CONFIG FATAL: Failed to initialize security keys from"
                                      " path declared on line %d.\n", line_num);
         retval = 1;  break;
        }
       key_file_set = 1;
       continue;
     }

     // --- DIRECTIVE 4: PASSWORD STRIKE LIMIT ---
     if(strcmp(token, "pwdpol_strike_limit") == 0){
       if(!value){
         fprintf(stderr, "CONFIG ERROR: Line %d specifies 'pwdpol_strike_limit'"
                                               " without a value.\n", line_num);
         retval = 1;  break;
        }
       Strike_limit = (uint8_t)atoi(value);
       pp_strike_lim_found = 1;
       fprintf(stdout, "  > Pwd Strike_limit will be set to: %"PRIu8"\n",
                                                                   Strike_limit);
       continue;
      }

     // --- DIRECTIVE 5: PASSWORD STRIKEOUT DELAY ---
     if(strcmp(token, "pwdpol_strikeout_delay") == 0){
       if(!value){
         fprintf(stderr, "CONFIG ERROR: Line %d specifies"
                      " 'pwdpol_strikeout_delay' without a value.\n", line_num);
         retval = 1;  break;
        }

       // Reset errno before the call
       errno = 0; 
       parsed_value = (time_t)strtoll(value, &endptr, 10);
       // Check for parsing errors
       if (endptr == value) {
         fprintf(stderr, "CONFIG ERROR: Line %d specifies"
             " 'pwdpol_strikeout_delay' but no digits were found.\n", line_num);
         retval = 1;  break;
       } else if (errno == ERANGE) {
         fprintf(stderr, "CONFIG ERROR: Line %d specifies"
        " 'pwdpol_strikeout_delay' but the value is out of range.\n", line_num);
         retval = 1;  break;
       }
       if(parsed_value < 0) {
         fprintf(stderr, "CONFIG ERROR: Line %d specifies"
            " 'pwdpol_strikeout_delay' but the value is negative.\n", line_num);
         retval = 1;  break;
       }
       Strikeout_delay = parsed_value;
       pp_strikeout_delay_found = 1;
       fprintf(stdout, "  > Pwd Strikeout_delay will be set to: %lld\n",
                                                (long long int)Strikeout_delay);
       continue;
      }

     // --- DIRECTIVE 6: PASSWORD EXPIRY TIMEOUT ---
     if(strcmp(token, "pwdpol_expiry_timeout") == 0){
       if(!value){
         fprintf(stderr, "CONFIG ERROR: Line %d specifies"
                       " 'pwdpol_expiry_timeout' without a value.\n", line_num);
         retval = 1;  break;
        }

       // Reset errno before the call
       errno = 0; 
       parsed_value = (time_t)strtoll(value, &endptr, 10);
       // Check for parsing errors
       if (endptr == value) {
         fprintf(stderr, "CONFIG ERROR: Line %d specifies"
              " 'pwdpol_expiry_timeout' but no digits were found.\n", line_num);
         retval = 1;  break;
       } else if (errno == ERANGE) {
         fprintf(stderr, "CONFIG ERROR: Line %d specifies"
         " 'pwdpol_expiry_timeout' but the value is out of range.\n", line_num);
         retval = 1;  break;
       }
       if(parsed_value < 0) {
         fprintf(stderr, "CONFIG ERROR: Line %d specifies"
             " 'pwdpol_expiry_timeout' but the value is negative.\n", line_num);
         retval = 1;  break;
       }
       Password_expiry_timeout = parsed_value;
       pp_expiry_timeout_found = 1;
       fprintf(stdout, "  > Pwd Expiry Timeout will be set to: %lld seconds.\n",
                                        (long long int)Password_expiry_timeout);
       continue;
      }

     // --- DIRECTIVE 7: PASSWORD MINIMUM LENGTH ---
     if(strcmp(token, "pwdpol_min_length") == 0){
       if(!value){
         fprintf(stderr, "CONFIG ERROR: Line %d specifies 'pwdpol_min_length'"
                                               " without a value.\n", line_num);
         retval = 1;  break;
        }
       // Reset errno before the call
       errno = 0; 
       parsed_len = (time_t)strtoll(value, &endptr, 10);
       // Check for parsing errors
       if (endptr == value) {
         fprintf(stderr, "CONFIG ERROR: Line %d specifies"
              " 'pwdpol_min_length' but no digits were found.\n", line_num);
         retval = 1;  break;
       } else if (errno == ERANGE) {
         fprintf(stderr, "CONFIG ERROR: Line %d specifies"
         " 'pwdpol_min_length' but the value is out of range.\n", line_num);
         retval = 1;  break;
       }
       if(parsed_len > MAX_PASS_LEN) {
         fprintf(stderr, "CONFIG ERROR: Line %d specifies"
                " 'pwdpol_min_length' but the value is too large.\n", line_num);
         retval = 1;  break;
       }
       Password_min_length = parsed_len;
       pp_min_length_found = 1;
       fprintf(stdout, "  > Pwd Min. Length will be set to: %zu bytes.\n",
                                                           Password_min_length);
       continue;
      }

     // --- DIRECTIVE 8: PASSWORD MUST HAVE UPPER CASE ---
     if(strcmp(token, "pwdpol_needs_upper") == 0){
       if(!value){
         fprintf(stderr, "CONFIG ERROR: Line %d specifies 'pwdpol_needs_upper'"
                                               " without a value.\n", line_num);
         retval = 1;  break;
        }
       Pwd_needs_upper = (uint8_t)atoi(value);
       if(Pwd_needs_upper > 1){
         fprintf(stderr, "CONFIG ERROR: Unsupported pwdpol_needs_upper value"
                                           " '%s' (Expected 0 or 1).\n", value);
         retval = 1; break;
        }
       pp_needs_upper_found = 1;
       fprintf(stdout, "  > Pwd must have upper case will be set to: %"PRIu8
                                                          "\n",Pwd_needs_upper);
       continue;
      }

     // --- DIRECTIVE 9: PASSWORD MUST HAVE LOWER CASE ---
     if(strcmp(token, "pwdpol_needs_lower") == 0){
       if(!value){
         fprintf(stderr, "CONFIG ERROR: Line %d specifies 'pwdpol_needs_lower'"
                                               " without a value.\n", line_num);
         retval = 1;  break;
        }
       Pwd_needs_lower = (uint8_t)atoi(value);
       if(Pwd_needs_lower > 1){
         fprintf(stderr, "CONFIG ERROR: Unsupported pwdpol_needs_lower value"
                                           " '%s' (Expected 0 or 1).\n", value);
         retval = 1; break;
        }
       pp_needs_lower_found = 1;
       fprintf(stdout, "  > Pwd must have lower case will be set to: %"PRIu8
                                                          "\n",Pwd_needs_lower);
       continue;
      }

     // --- DIRECTIVE 10: PASSWORD MUST HAVE A DIGIT ---
     if(strcmp(token, "pwdpol_needs_digit") == 0){
       if(!value){
         fprintf(stderr, "CONFIG ERROR: Line %d specifies 'pwdpol_needs_digit'"
                                               " without a value.\n", line_num);
         retval = 1;  break;
        }
       Pwd_needs_digit = (uint8_t)atoi(value);
       if(Pwd_needs_digit > 1){
         fprintf(stderr, "CONFIG ERROR: Unsupported pwdpol_needs_digit value"
                                           " '%s' (Expected 0 or 1).\n", value);
         retval = 1; break;
        }
       pp_needs_digit_found = 1;
       fprintf(stdout, "  > Pwd must have a digit will be set to: %"PRIu8
                                                          "\n",Pwd_needs_digit);
       continue;
      }

     // --- DIRECTIVE 11: PASSWORD MUST HAVE SPECIAL ---
     if(strcmp(token, "pwdpol_needs_special") == 0){
       if(!value){
         fprintf(stderr, "CONFIG ERROR: Line %d specifies 'pwdpol_needs_special'"
                                               " without a value.\n", line_num);
         retval = 1;  break;
        }
       Pwd_needs_special = (uint8_t)atoi(value);
       if(Pwd_needs_special > 1){
         fprintf(stderr, "CONFIG ERROR: Unsupported pwdpol_needs_special value"
                                           " '%s' (Expected 0 or 1).\n", value);
         retval = 1; break;
        }
       pp_needs_special_found = 1;
       fprintf(stdout, "  > Pwd must have a special char will be set to: %"
                                                     PRIu8"\n",Pwd_needs_special);
       continue;
      }

     // --- DIRECTIVE 12: DYNAMIC USER DATABASE ---
     if(strcmp(token, "user_database") == 0){
       fprintf(stdout,"  > Loading user account definitions from: %s\n", value);
       if(sv_load_user_file(value) != 0){
         fprintf(stderr, "CONFIG FATAL: Failed to parse user database from"
                                      " path declared on line %d.\n", line_num);
         retval = 1; break;
        }
       user_database = 1;
       // Set the name of the 
       snprintf(sv_Userfile, sizeof(sv_Userfile), "%s", value);
       continue;
      }

     // --- DIRECTIVE 13: BREAKOUT BOARD LINK ---
     if(strcmp(token, "bob_path") == 0){
         int callresult;
         if(strcmp(value, "auto") == 0){
           fprintf(stdout, "  > Initializing connection with Arduino"
                                                " (Auto-Detecting Port) ...\n");
           callresult = as_open_comms_with_Arduino(NULL);
         } else {
           fprintf(stdout, "  > Initializing connection with Arduino at port:"
                                                                " %s\n", value);
           callresult = as_open_comms_with_Arduino(value);
         }

         if(callresult != 0){
           fprintf(stderr, "CONFIG FATAL: Failed to establish serial"
         " communications with hardware breakout board (Line %d).\n", line_num);
           retval = 1;  break;
         }
         as_connected = 1;
         CurrMsg = 0; // Initialize serial message counter frame safely
         bob_path_set = 1;
         continue;
     }

     // --- DIRECTIVE 14: PARDCAP CAPTURE SERVER CONNECTIONS ---
     if(strcmp(token, "pardcap") == 0){
         char ip[17];
         char port[7];

         // Extract the IP and Port segments cleanly from the value payload
         if(sscanf(value, "%16s %6s", ip, port) != 2){
           fprintf(stderr, "CONFIG ERROR: Line %d 'pardcap' mapping layout is"
                " malformed. Expected format: pardcap <ip> <port>\n", line_num);
           retval = 1; break;
         }

         fprintf(stdout, "  > Connecting to PARDcapture server: %s:%s ...\n",
                                                                      ip, port);
         
         // Invoke the system-native connection engine
         if(ps_connect(ip, port) != 0){
           fprintf(stderr, "CONFIG FATAL: Critical connection failure to"
                               " targeted PARDcap server at %s:%s (Line %d).\n",
                                                            ip, port, line_num);
           retval = 1; break;
         }

         // Set the first connection as the default current camera server.
         if(NPSs == 1){
             ps_Comms_fd = PSlist[0].fd;
             ps_CliSrv_connected = 1;
             CurrPS = 1;
         }
         continue;
     }

     // --- DIRECTIVE 15: PARDERVER PORT ---
     if(strcmp(token, "server_port") == 0){
       if(!value){
         fprintf(stderr, "CONFIG ERROR: Line %d specifies 'server_port'"
                                               " without a value.\n", line_num);
         retval = 1;  break;
        }
       Pardport = atoi(value);
       if(Pardport > 65535 || Pardport < 1024){
         fprintf(stderr, "CONFIG ERROR: Unsupported server_port value '%s' "
                                         " (Expected 1024 to 65535).\n", value);
         retval = 1; break;
        }
       pardport_found = 1;
       fprintf(stdout, "  > PARDserver will connect on port: %d\n",Pardport);

       continue;
      }

     // --- DIRECTIVE 16: PARDERVER NAME ---
     if(strcmp(token, "server_name") == 0){
       if(!value){
         fprintf(stderr, "CONFIG ERROR: Line %d specifies 'server_name'"
                                               " without a value.\n", line_num);
         retval = 1;  break;
        }
       snprintf(sv_Pardname,sizeof(sv_Pardname),"%s",value);
       pardname_found = 1;
       fprintf(stdout, "  > PARDserver will be called: %s\n",sv_Pardname);

       continue;
      }

     // --- DIRECTIVE 17: VERBOSITY MODE ---
     if(strcmp(token, "verbosity") == 0){
       if(!value){
         fprintf(stderr, "CONFIG ERROR: Line %d specifies 'verbosity'"
                                               " without a value.\n", line_num);
         retval = 1;  break;
        }
       Verbosity = atoi(value);
       if(Verbosity > 2 || Verbosity < 0){
         fprintf(stderr, "CONFIG ERROR: Unsupported verbosity value '%s' "
                                                " (Expected 0 to 2).\n", value);
         retval = 1; break;
        }
       verbosity_found = 1;
       fprintf(stdout, "  > Verbosity will be set to: %d\n",Verbosity);
       continue;
      }

     // Catch unrecognised configuration command arguments safely
     fprintf(stderr, "CONFIG WARNING: Unrecognised configuration token"
        " directive '%s' encountered on line %d. Skipping.\n", token, line_num);
 }

 fclose(fp);

 if(retval == 0){
 // Issue information as to the number of pardcapture servers connected:
  if(NPSs){
   sprintf(line,"You have %u PARDcap server connection%s.",NPSs,
                                                               (NPSs>1)?"s":"");
   show_message(line,"\n",MT_INFO,0);
   show_message("The default connection is set to server 1.","",MT_INFO,0);
  } else {
   show_message("\nYou have no PARDcap server connections so you will not be"
                " able to use camera functions.","sv_setup_via_file",MT_INFO,0);
   ps_Comms_fd=-1;
   ps_CliSrv_connected=0;
  }
 show_message("","",MT_INFO,0); // Print new line

  // Do a checksum to see if all required settings were made:
  config_sum = key_file_set + user_database + bob_path_set
             + verbosity_found + pardport_found + pardname_found
             + pp_strike_lim_found + pp_strikeout_delay_found 
             + pp_expiry_timeout_found + pp_min_length_found
             + pp_needs_upper_found + pp_needs_lower_found 
             + pp_needs_digit_found + pp_needs_special_found;
  
  switch(ver){
    case 1:
      if(config_sum != 14){
        // Use strncat to enforce the 256-byte constraint defensively
        char missing_msg[340] = {0};
        if (!key_file_set)            strncat(missing_msg, " [key_file]",              sizeof(missing_msg) - strlen(missing_msg) - 1);
        if (!pp_strike_lim_found)     strncat(missing_msg, " [pwdpol_strike_limit]",   sizeof(missing_msg) - strlen(missing_msg) - 1);
        if (!pp_strikeout_delay_found)strncat(missing_msg, " [pwdpol_strikeout_delay]",sizeof(missing_msg) - strlen(missing_msg) - 1);
        if (!pp_expiry_timeout_found) strncat(missing_msg, " [pwdpol_expiry_timeout]", sizeof(missing_msg) - strlen(missing_msg) - 1);
        if (!pp_min_length_found)     strncat(missing_msg, " [pwdpol_min_length]",     sizeof(missing_msg) - strlen(missing_msg) - 1);
        if (!pp_needs_upper_found)    strncat(missing_msg, " [pwdpol_needs_upper]",    sizeof(missing_msg) - strlen(missing_msg) - 1);
        if (!pp_needs_lower_found)    strncat(missing_msg, " [pwdpol_needs_lower]",    sizeof(missing_msg) - strlen(missing_msg) - 1);
        if (!pp_needs_digit_found)    strncat(missing_msg, " [pwdpol_needs_digit]",    sizeof(missing_msg) - strlen(missing_msg) - 1);
        if (!pp_needs_special_found)  strncat(missing_msg, " [pwdpol_needs_special]",  sizeof(missing_msg) - strlen(missing_msg) - 1);
        if (!user_database)           strncat(missing_msg, " [user_database]",         sizeof(missing_msg) - strlen(missing_msg) - 1);
        if (!bob_path_set)            strncat(missing_msg, " [bob_path]",              sizeof(missing_msg) - strlen(missing_msg) - 1);
        if (!pardport_found)          strncat(missing_msg, " [server_port]",           sizeof(missing_msg) - strlen(missing_msg) - 1);
        if (!pardname_found)          strncat(missing_msg, " [server_name]",           sizeof(missing_msg) - strlen(missing_msg) - 1);
        if (!verbosity_found)         strncat(missing_msg, " [verbosity]",             sizeof(missing_msg) - strlen(missing_msg) - 1);



        fprintf(stderr,"CONFIG FATAL: Incomplete configuration file structure."
                        "\nThe following mandatory directives are missing:%s\n",
                                                                   missing_msg);
        retval = 1;
        goto server_config_failed;
       } 
    break;
    default:
      fprintf(stdout, "CONFIG FATAL: Program bug.\n\n");
      retval = 1;
    goto server_config_failed;
   }

 fprintf(stdout, "SUCCESS: Server environment fully initialized."
                                        " Launching server socket daemon.\n\n");
 } else {
  server_config_failed:
  // Hang up on any pardcap connections that may have been made.
  for(idx=0;idx<NPSs;idx++){
     show_message("> ","sv_setup_via_file",MT_INFO,1);
     ps_hangup(idx);
    }

  // Issue the bad news
  fprintf(stdout, "CONFIG FATAL: Server setup from config file failed.\n\n");
 }

 return retval;
}

int sv_init_or_load_keys(const char *key_file_path)
// Initialise or load server keys for ZMQ Curve encryption layer
// Return 0 on success
// Return 1 on error
{
 FILE *fp = fopen(key_file_path, "r");
 if (!fp) {
     // Fail loudly so the administrator knows they forgot to install the keys
     fprintf(stderr, "CRITICAL ERROR: Server key file not found at: %s\n", key_file_path);
     fprintf(stderr, "Please generate a Z85 keypair and place it in this path.\n");
     return 1;
 }

 // Read the keys line by line
 // Each Z85 key is exactly 40 characters long, plus a newline character
 char line1[64] = {0};
 char line2[64] = {0};

 if (!fgets(line1, sizeof(line1), fp) || !fgets(line2, sizeof(line2), fp)) {
     fprintf(stderr, "CRITICAL ERROR: Key file format is invalid.\n");
     fclose(fp);
     return 1;
 }
 fclose(fp);

 // Strip out any trailing newline characters safely using strcspn
 line1[strcspn(line1, "\r\n")] = '\0';
 line2[strcspn(line2, "\r\n")] = '\0';

 // Verify both strings meet the exact 40-character requirements of Z85 format
 if (strlen(line1) != 40 || strlen(line2) != 40) {
     fprintf(stderr, 
           "CRITICAL ERROR: Loaded keys are not exactly 40 characters long.\n");
     return 1;
 }

 // Secure copy into global network variables
 strncpy(server_pub, line1, 40);
 server_pub[40] = '\0';
 strncpy(server_sec, line2, 40);
 server_sec[40] = '\0';
 
 return 0; // Success
}

int sv_load_user_file(const char *user_file_path)
// Reads the administrator supplied credentials file and populates the global
// users database.
// Returns 0 on success
// Returns 1 on failure.
{
 FILE *fp = fopen(user_file_path, "r");
 if(!fp){
   fprintf(stderr, "CRITICAL ERROR: User credentials file not found at: %s\n",
                                                                user_file_path);
   return 1;
 }

 // Only calloc if the pointer is NULL.
 if(sv_UserDb == NULL) {
    sv_UserDb = calloc(MAX_TOTAL_USERS, sizeof(UserCred));
 }
 if(sv_UserDb == NULL) {
   fprintf(stderr, "CRITICAL ERROR: Failed to allocate RAM for the user account"
                                                                " database.\n");
   fclose(fp);
   return 1;
 }

 char line[512];
 int line_num = 0;
 sv_Upllen = 0;

 while(fgets(line, sizeof(line), fp)){
     line_num++;

     // 1. Strip trailing newlines safely using strcspn
     line[strcspn(line, "\r\n")] = '\0';

     // 2. Skip empty lines or administrative comments
     char *start = line;
     while (*start && isspace((unsigned char)*start)) start++;
     if (*start == '\0' || *start == '#') continue;

     // 3. Extract the fields based on sequential colons
     
     // --- FIELD 1: Username ---
     char *username_token = start;
     char *colon1 = strchr(username_token, ':');
     if(!colon1) {
       fprintf(stderr, "CONFIG WARNING: Line %d is malformed (missing"
                                        " timecreated). Skipping.\n", line_num);
       continue;
      }
     *colon1 = '\0'; // Split string
     // SECURITY ADDITION: Explicitly reject spaces or tabs in usernames
     if(strchr(username_token, ' ') || strchr(username_token, '\t')){
       fprintf(stderr, "CONFIG ERROR: Line %d contains an invalid username"
                           " (spaces/tabs not allowed). Skipping.\n", line_num);
       continue;
     }

     // --- FIELD 2: Time Created ---
     char *time_token = colon1 + 1;
     char *colon2 = strchr(time_token, ':');
     if(!colon2) {
       fprintf(stderr, "CONFIG WARNING: Line %d is malformed (missing"
                                          " singleuse). Skipping.\n", line_num);
       continue;
      }
     *colon2 = '\0'; // Split string

     // --- FIELD 3: Single Use Flag ---
     char *otp_token = colon2 + 1;
     char *colon3 = strchr(otp_token, ':');
     if(!colon3) {
       fprintf(stderr, "CONFIG WARNING: Line %d is malformed (missing password"
                                               " hash). Skipping.\n", line_num);
       continue;
      }
     *colon3 = '\0'; // Split string

     // --- FIELD 4: Argon2id Password Hash ---
     char *hash_token = colon3 + 1;

     // Trim any leftover leading whitespace preceding the password token
     while(*hash_token && isspace((unsigned char)*hash_token)) hash_token++;

     if(*hash_token == '\0'){
       fprintf(stderr, "CONFIG WARNING: Line %d contains an empty password"
                                               " entry. Skipping.\n", line_num);
       continue;
     }

     // 4. Memory Bounds Check: Prevent buffer overflow attacks
     if(strlen(username_token) >= MAX_USER_LEN ||
        strlen(hash_token) >= MAX_PASS_LEN) {
       fprintf(stderr, "CONFIG ERROR: Credentials on line %d exceed system"
                                                 " length limits.\n", line_num);
       continue;
     }

     if(sv_Upllen >= MAX_TOTAL_USERS){
       fprintf(stderr, "CONFIG WARNING: Maximum user allocation limits reached"
                                  " (%d). Truncating file.\n", MAX_TOTAL_USERS);
       break;
     }

     // 5. Commit the parsed types securely to the global structure array
     strcpy(sv_UserDb[sv_Upllen].username, username_token);
     strcpy(sv_UserDb[sv_Upllen].password, hash_token);
     
     // Safely convert ascii values down into structural variables
     sv_UserDb[sv_Upllen].time_created = (time_t)strtoll(time_token, NULL, 10);
     sv_UserDb[sv_Upllen].single_use   = (uint8_t)atoi(otp_token);
     
     // Initialize memory runtime protections
     sv_UserDb[sv_Upllen].strike_count = 0;
     sv_UserDb[sv_Upllen].lockout_until = 0;
     sv_Upllen++;
 }

 fclose(fp);

 if(sv_Upllen == 0) {
   fprintf(stderr, "CRITICAL ERROR: No valid user accounts could be loaded from"
                                                                    " file.\n");
   free(sv_UserDb);
   sv_UserDb = NULL;
   return 1;
  }

 fprintf(stdout, "SUCCESS: Loaded %d user account profiles securely into"
                                             " database context.\n", sv_Upllen);
 return 0;
}

// Assuming standard buffer boundaries for keyboard responses
#define INPUT_BUF_SIZE 256

int sv_build_user_database(const char *filepath)
// Safely allows the sysadmin to interactively add one or more users.
// Leverages the in-memory array and updates the file atomically to prevent corruption.
{
 char username[INPUT_BUF_SIZE] = {0};
 char password[INPUT_BUF_SIZE] = {0};
 char choice[INPUT_BUF_SIZE] = {0};
 char hashed_password[crypto_pwhash_STRBYTES] = {0};

 // 1. Try to load the file if it already exists to preserve existing accounts
 printf("Checking for existing database at '%s'...\n", filepath);
 if (sv_load_user_file(filepath) != 0) {
     printf("No valid existing database found or file is empty.\n"
            "Creating a new database context in RAM.\n");
     // Allocate clean heap space if sv_load_user_file hasn't already done it
     if (sv_UserDb == NULL) {
         sv_UserDb = calloc(MAX_TOTAL_USERS, sizeof(UserCred));
         if(sv_UserDb == NULL){
           fprintf(stderr,
                "FATAL ERROR: Failed to allocate RAM for the user database.\n");
           return 1;
         }
     }
     sv_Upllen = 0; // Initialize user count to 0 for a brand new file
 }

 while (1) {
     // Bounds Check: Ensure we don't exceed the global capacity array limits
     if (sv_Upllen >= MAX_TOTAL_USERS) {
         fprintf(stderr, "\nERROR: Maximum user allocation limits reached (%d)."
                                  " Cannot add more users.\n", MAX_TOTAL_USERS);
         break;
     }

     // 2. Prompt for the Username
     printf("\n\n  > Enter Username: ");
     get_keyboard_input(username, sizeof(username));

     // Bounds check and character validation
     if(strlen(username) == 0 || strlen(username) >= MAX_USER_LEN || 
        strchr(username, ' ') || strchr(username, '\t') ||
        strchr(username, ':')) {
        printf("     Invalid username. Must be under %d chars and cannot"
                           " contain spaces, tabs, or colons.\n", MAX_USER_LEN);
        continue;
      }

     // =======================================================
     // ACCOUNT MANAGEMENT (DUPLICATES/UPDATES/DELETIONS)
     // =======================================================
     int user_index = -1;
     for (int idx = 0; idx < sv_Upllen; idx++) {
         if (strcmp(username, sv_UserDb[idx].username) == 0) {
             user_index = idx;
             break;
         }
     }

     if (user_index != -1) {
         printf("   \nUser '%s' is already on the database. Choose an option:\n"
                                                                    , username);
         printf("      1. Ignore and skip this entry [Default]\n");
         printf("      2. Change this user's password\n");
         printf("      3. Delete this user from the system\n");
         printf("     > Option (1-3): ");
         
         char opt_choice[INPUT_BUF_SIZE] = {0};
         get_keyboard_input(opt_choice, sizeof(opt_choice));
         
         int option = (strlen(opt_choice) > 0) ? atoi(opt_choice) : 1;

         switch(option){
           case 2: // OPTION 2: CHANGE PASSWORD
             printf("     > Enter New Password for '%s': ", username);
             get_keyboard_input(password, sizeof(password));

             if(strlen(password) == 0 || strchr(password, ' ') ||
                                                       strchr(password, '\t')) {
               printf("        Invalid password. Change aborted.\n\n");
               sodium_memzero(password, sizeof(password));
               goto prompt_for_another_user;
              }

             printf("        Generating new Argon2id cryptographic hash... ");
             fflush(stdout);
             
             if(crypto_pwhash_str(sv_UserDb[user_index].password, password,
                                                          strlen(password),
                                        crypto_pwhash_OPSLIMIT_INTERACTIVE, 
                                   crypto_pwhash_MEMLIMIT_INTERACTIVE) != 0) {
               fprintf(stderr,
                        "\nFATAL CRYPTO ERROR: Hashing resource exhaustion.\n");
               sodium_memzero(password, sizeof(password));
               return 1;
              }
             printf("Done.\n");
             sodium_memzero(password, sizeof(password));
             
             // Reset strikes natively when updating a password
             sv_UserDb[user_index].strike_count = 0; 
             sv_UserDb[user_index].lockout_until = 0;
             // (14.11.26: these two lines used the index sv_Upllen - the free
             //  slot after the last user - instead of user_index, so the new
             //  password was neither made a one-time password nor given a new
             //  creation time, and a full database was written out of bounds.)
             sv_UserDb[user_index].time_created = time(NULL);
             sv_UserDb[user_index].single_use = 1;
             printf("        Successfully updated password for user '%s'.\n\n",
                                                                      username);
           break;

           case 3:  // OPTION 3: DELETE USER FROM THE SYSTEM
             // To delete in a contiguous array, shift all items up by one slot
             for(int i = user_index; i < sv_Upllen - 1; i++){
                sv_UserDb[i] = sv_UserDb[i + 1];
              }
             
             // Clear the newly vacant trailing array slot cleanly to prevent
             // memory artefacts
             memset(&sv_UserDb[sv_Upllen - 1], 0, sizeof(UserCred));
             sv_Upllen--; // Decrement the total user array allocation count
             
             printf("        Successfully deleted user '%s' from memory state."
                                                              "\n\n", username);
           break;

           default:
             // OPTION 1 / OTHER: IGNORE AND MOVE ON
             printf("        Skipping entry. No changes made.\n\n");
           break;
         }

         // In all cases for an existing user, skip the standard "Add New" code path 
         // and ask the sysadmin if they want to enter another username.
         goto prompt_for_another_user; 
     }
     // =======================================================


     // 3. Prompt for the Plain text Password
     printf("  > Enter Password: ");
     get_keyboard_input(password, sizeof(password));

     if(strlen(password) == 0 || strchr(password, ' ') ||
       strchr(password, '\t')){
         printf("     Invalid password. Passwords cannot be empty, contain"
                                                       " spaces, or tabs.\n\n");
         sodium_memzero(password, sizeof(password)); // Clear immediately
         continue;
     }

     // 4. Hash the password with Libsodium (Argon2id)
     printf("     Generating secure Argon2id cryptographic hash... ");
     fflush(stdout);

     if (crypto_pwhash_str(
             hashed_password,
             password,
             strlen(password),
             crypto_pwhash_OPSLIMIT_INTERACTIVE, 
             crypto_pwhash_MEMLIMIT_INTERACTIVE
         ) != 0) {
         fprintf(stderr, "\nFATAL CRYPTO ERROR: Memory or CPU resource"
                                             " exhaustion during hashing.\n\n");
         sodium_memzero(password, sizeof(password));
         return 1;
     }
     printf("Done.\n");

     // Clear sensitive temporary password memory from RAM completely right
     // after hashing
     sodium_memzero(password, sizeof(password));

     // 5. Commit the entry securely to the global RAM struct array
     strcpy(sv_UserDb[sv_Upllen].username, username);
     strcpy(sv_UserDb[sv_Upllen].password, hashed_password);
     sv_UserDb[sv_Upllen].strike_count = 0;
     sv_UserDb[sv_Upllen].time_created = time(NULL);
     sv_UserDb[sv_Upllen].single_use = 1;
     sv_Upllen++;


     printf("     Successfully staged user '%s' in memory.\n\n", username);

 prompt_for_another_user:
     // 6. Check if the Sysadmin wants to add another user
     printf("\n  > Add or manage another user (y/N)? ");
     get_keyboard_input(choice, sizeof(choice));
     
     char repeat_char = 'N';
     if(strlen(choice) >= 1){
        repeat_char = (char)toupper((unsigned char)choice[0]);
      }

     // Loop condition break
     if(repeat_char != 'Y'){
        break; 
      }
 }

 // 7. Commit changes to disk ATOMICALLY via your save function
 printf("\nWriting database changes securely to disk...\n");
 if (sv_save_user_file(filepath) != 0) {
     fprintf(stderr, "FATAL ERROR: Failed to write user database to file.\n\n");
     return 1;
 }

 printf("User database creation completed successfully.\n\n");
 return 0;
}

uint8_t sv_file_to_safmt(char *ffile)
// Takes in a standard filename string for a corrections image (flat field, dark
// field or a corrections mask) name composed by the pardclient and returns
// the format SAF_ constant representing the file format defined by that name.
{
 if(!strcmp(ffile, "ffimg_Y.pgm"))    return SAF_YP5;
 if(!strcmp(ffile, "ffimg_Y.bmp"))    return SAF_BM8;
 if(!strcmp(ffile, "cmask_Y.bmp"))    return SAF_BM8;
 if(!strcmp(ffile, "pmask_Y.bmp"))    return SAF_BM8;
 if(!strcmp(ffile, "ffimg_RGB.ppm"))  return SAF_RGB;
 if(!strcmp(ffile, "ffimg_RGB.bmp"))  return SAF_BMP;
 if(!strcmp(ffile, "pcdark_RGB.bmp")) return SAF_BMP;
 if(!strcmp(ffile, "ffimg_Y.dou"))    return SAF_DOU;
 if(!strcmp(ffile, "dfimg_Y.dou"))    return SAF_DOU;
 if(!strcmp(ffile, "pmdark_Y.dou"))   return SAF_DOU;
 if(!strcmp(ffile, "pmflat_Y.dou"))   return SAF_DOU;
 if(!strcmp(ffile, "ffimg_R.dou"))    return SAF_DO3;
 if(!strcmp(ffile, "dfimg_R.dou"))    return SAF_DO3;
 return SAF_YUYV; // This will effectively raise an error because no corrections
                  // image will have this formt.
}

int sv_evalonoff(const char *sst)
// Tests string sst to see if it is one of "ON" or "OFF" (case insensitive)
// Returns 0 if sst is "OFF"
// Returns 1 if sst is "ON"
// Returns -1 if it is neither or if local memory allocation fails.
{
 char *caseblindsst;
 size_t len,idx;
 int rv=-1;

 // Make an all lower case copy of sst
 len=strlen(sst)+1;
 caseblindsst=calloc(len,sizeof(char));
 if(caseblindsst==NULL){rv=35;  goto end_sv_evalonoff;}
 for(idx=0;idx<len;idx++) caseblindsst[idx]=tolower(sst[idx]);

 if(!strcmp(caseblindsst, "on"))  rv = 1;
 else if(!strcmp(caseblindsst, "off")) rv = 0;

end_sv_evalonoff:
 if(rv!=35) free(caseblindsst);
 else {
   show_message("Failed to get memory for caseblind comparison of ON/OFF\n",
                                                        "Error: ", MT_ERR, 0);
   rv = -1;
  }

 return rv;
}

int sv_switch_string(const char *sst)
// Tests string sst to see if it is one of a standard set of strings.
// These are: "ON", "OFF", "FITS" or a word ending in ".bmp", ".pgm", ".ppm" or
// one of the raw doubles types (.dou). Matching is case insensitive.
// Returns one of the SWS_ constants.
{
 char *caseblindsst;
 size_t len,idx;
 int rv=0;

 // Make an all lower case copy of sst
 len=strlen(sst)+1;
 caseblindsst=calloc(len,sizeof(char));
 if(caseblindsst==NULL){rv=35;  goto end_switch_string;}
 for(idx=0;idx<len;idx++) caseblindsst[idx]=tolower(sst[idx]);

 if(!strcmp(caseblindsst, "on"))  rv = SWS_ON;
 else if(!strcmp(caseblindsst, "off")) rv = SWS_OFF;
 else if(!strcmp(caseblindsst, "fits")) rv = SWS_FITS;
 else if(!strcmp(((strrchr(caseblindsst, '.')) + 1), "ppm"))    rv = SWS_PPM;
 else if(!strcmp(((strrchr(caseblindsst, '.')) + 1), "pgm"))    rv = SWS_PGM;
 else if(!strcmp(((strrchr(caseblindsst, '.')) + 1), "bmp"))    rv = SWS_BMP;
 else if(!strcmp(((strrchr(caseblindsst, '.')) - 2), "_y.dou")) rv = SWS_DOU;
 else if(!strcmp(((strrchr(caseblindsst, '.')) - 2), "_i.dou")) rv = SWS_DOU;
 else if(!strcmp(((strrchr(caseblindsst, '.')) - 2), "_r.dou")) rv = SWS_DO3;
 else if(!strcmp(((strrchr(caseblindsst, '.')) - 2), "_g.dou")) rv = SWS_DO3;
 else if(!strcmp(((strrchr(caseblindsst, '.')) - 2), "_b.dou")) rv = SWS_DO3;
 else rv = SWS_NOS;

end_switch_string:
 if(rv!=35) free(caseblindsst);
 else {
   show_message("Failed to get memory for caseblind comparison\n",
                                                        "Error: ", MT_ERR, 0);
   rv = SWS_NOS;
  }


 return rv;
}

static int sv_receive_data_zmq(void *socket)
// Receive a Pardus message from the client. If correct this will be:
//    1. the first 4 parts of a multipart message: the header
//       (into sv_Msghdr[3][]) and the sv_Triad.
//    2. An optional sv_Blob.
// Errors that are not fatal will result in an appropriate response being issued
// (if a message was received) and continued listening by the server. The server
// function should not be disrupted.
// Fatal errors usually result in the server attempting a reboot.
// Returns the number of parts read (upto 5) or <0 if there is an error.
// Returns -1 if a zmq function returned an error (other than a timeout)
// Returns -2 if timeout occurred.
// Returns -3 if any message part is empty.
// Returns -4 if any message part is too big (bigger than expected for a
//            correct PARDUS message).
// Returns -5 if the message has more than 5 parts.
// Returns -6 if the message was addressed to the wrong server or received from
//            an invalid client or client misunderstood the last message. 
// Returns -7 if memory allocation to sv_Triad or sv_Blob failed - this is
//            serious, the user must close the program ASAP
// Returns -8 if there has been an error with remaining parts and the attempt to
              // flush the remaining parts prior to returning the error code
              // also fails. This is serious. The user must quit the program
              // because they won't be able to send / receive any more messages.
{
 int rc,rcvmore=0,pdx;
 size_t len,rcvmore_size = sizeof(rcvmore);
 zmq_msg_t message;
 
  sv_Wrong_pard=0;
  sv_Wrong_client=0;
  sv_Wrong_version=0;
 
 // Initialise our message container
 rc = zmq_msg_init (&message); if(rc) return -1; // Error: ZMQ

 // Get header
 for(pdx=0;pdx<3;pdx++){
    // Get message part with its size in bytes.
    len=zmq_msg_recv(&message,socket,0);      
    if(len==-1){
      if(errno==EAGAIN){pdx=-2;goto error_return;} // Error: timeout
                  else {pdx=-1;goto error_return;} // Error: ZMQ
     }

    // ==========================================
    // SECURITY ADDITION: EXTRACT IP ADDRESS 
    // ==========================================
    if (pdx == 0) { // Only do this once on the very first part
        const char *ip_meta = zmq_msg_gets(&message, "Peer-Address");
        if (ip_meta != NULL) {
            strncpy(sv_Client_IP, ip_meta, sizeof(sv_Client_IP) - 1);
            sv_Client_IP[sizeof(sv_Client_IP) - 1] = '\0'; // Ensure termination
        } else {
            strcpy(sv_Client_IP, "Unknown IP");
        }
    }
    // ==========================================

    if(len<1){pdx=-3; goto error_return;}      // Error: Size too small 
    if(len>=TXT_BUFMAX){pdx=-4; goto error_return;}// Error: Size too big
    // We now have the message part in 'message' and we know the size (in
    // 'len'). We assume it is a string (it should be if it is compliant with
    // the PARDUS protocol). So copy it to the header section and terminate it
    // with '\0'
    memcpy(sv_Msghdr[pdx],zmq_msg_data(&message),len);sv_Msghdr[pdx][len]='\0';
    // Now check if there is another part.    
    rc=zmq_getsockopt(socket,ZMQ_RCVMORE,&rcvmore,&rcvmore_size);
    if(rc){pdx=-1;goto error_return;}// Error: ZMQ
    if(rcvmore==0)goto error_return; // Error: We shouldn't run out of parts if
                                     // this is a correctly formatted message.
   }
   
 // If we get to here then all three parts of the header received data and there
 // is another part (at least 1) to read (that is the sv_Triad string - the main
 // textual message client is sending us).
 
 // First check if the message is for our PARDUS server (we don't want to do
 // sv_Triad and sv_Blob processing if the message is not for us:
 if(strcmp(sv_Msghdr[0],sv_Pardname)){
                                    pdx=-6; sv_Wrong_pard=1; goto error_return;}
 
 // The message is addressed to us and has a properly formatted header.
 // If we have a logged-in client check if the message is:
 // a) from them and, if so,
 // b) reporting an error with reception of our last message so we can
 // resend it before overwriting the sv_Triad and sv_Blob data.
 if(sv_Client_logged_in){
    if(strcmp(sv_Msghdr[1],sv_Client_id)){
       pdx=-6; 
       sv_Wrong_client=1; 
       goto error_return; // Error: Not our client
      } 
    rc=atoi(sv_Msghdr[2]); // Get the client's message index
    if(rc<0){ // Client reports a problem interpreting the last message we sent.
    // The value of the negative number indicates the version of the
    // communications protocol that the client is operating. As it is unlikely
    // the message was corrupted in transit, the most likely cause of this
    // 'misunderstanding' is either a programming bug or a version
    // incompatibility (client program is working to a different communication
    // protocol version to this client).
    // Thus the proper way to deal with this error (apart from checking the
    // source code for bugs) is to take note of the client's vesion, compare it
    // to the server version and attempt to translate the last message the
    // server sent into the format that the client will understand prior to
    // resending it. However, currently, both server and client use version 1 so
    // this cannot be done. So for now we will just resend the last message with
    // a counter - after sv_Resendmax re-tries we will give up and log the
    // client off the system with an error message.
     sv_Wrong_version=rc;
     sv_Resendidx++;
     pdx=-6;
     goto error_return; // Error: Can't understand message.
    }
 }

 // OK. The message is addressed to us - so process the sv_Triad.
 // Get message part with its size in bytes.
 len=zmq_msg_recv(&message,socket,0);        
 if(len==-1){
   if(errno==EAGAIN){pdx=-2;goto error_return;} // Error: timeout
               else {pdx=-1;goto error_return;} // Error: ZMQ
  }
 if(len<1){pdx=-3; goto error_return;}        // Error: Size too small 
 if(len>=TRIADMAX){pdx=-4; goto error_return;}// Error: Size too big
 // (Re-)Allocate memory for client sv_Triad
 if(sv_triad_resize(len+1)){pdx=-7; goto error_return;} // Failed to get memory
 // Copy the (assumed string) into sv_Triad and terminate it.
 memcpy(sv_Triad,zmq_msg_data(&message),len);sv_Triad[len]='\0';
    
 pdx++; // Four parts read so far.
 
 // Now check if there is a sv_Blob - and if so, get it.    
 rc=zmq_getsockopt(socket,ZMQ_RCVMORE,&rcvmore,&rcvmore_size);
 if(rc){pdx=-1;goto error_return;} // Error: ZMQ
 if(rcvmore){ 
   // Get message part with its size in bytes
   len=zmq_msg_recv(&message,socket,0);        
   if(len==-1){
     if(errno==EAGAIN){pdx=-2;goto error_return;} // Error: timeout
                 else {pdx=-1;goto error_return;} // Error: ZMQ
    }
   if(len<1){pdx=-3; goto error_return;}        // Error: Size too small 
   // (Re-)Allocate memory for client sv_Blob
   if(sv_blob_resize(len)){pdx=-7; goto error_return;} // Failed to get memory
   // Copy the presumed sv_Blob into 'sv_Blob'.
   memcpy(sv_Blob,zmq_msg_data(&message),len);
   sv_Blobsize=len; 
   pdx++; // 5 parts read - that should be all. If there are more it is an error
   // so check to see if there are any more parts to the message.    
   rc=zmq_getsockopt(socket,ZMQ_RCVMORE,&rcvmore,&rcvmore_size);
   if(rc){pdx=-1;goto error_return;}      // Error: ZMQ
   if(rcvmore){pdx=-5;goto error_return;} // Error: Too many parts
  } 

 rc = zmq_msg_close(&message);
 if(rc){ // Everything has been read and received so if this error occurs at
         // this stage it might be OK to just carry on.
   show_message("The message was received from the client but there was a poblem closing the message pointer.","ERROR: ",MT_ERR,0);
 }
 
 return pdx;
 
error_return:
 if(pdx==-3 || pdx==-4){ // If message part too big or too small (empty) we need
                         // to check if there are more parts to follow
    rc=zmq_getsockopt(socket,ZMQ_RCVMORE,&rcvmore,&rcvmore_size); if(rc)pdx=-8;
   }
 if(rcvmore){ // There are unread parts of the message but we are terminating
              // with an error so flush out the remaining messages before
              // returning so user can send a message if they need to.
    do {
        rc=zmq_msg_recv(&message,socket,0);
        if(rc==-1){pdx=-8; break;}
        rc=zmq_getsockopt(socket,ZMQ_RCVMORE,&rcvmore,&rcvmore_size);
        if(rc){pdx=-8; break;}
       } while (rcvmore);             
  }
 zmq_msg_close(&message); // I am not checking for errors here -
                          // the caller has bigger fish to fry.
 return pdx;
}

#if IS_WINDOWS_NATIVE
int sv_save_user_file(const char *user_file_path)
// Rewrites the entire memory database back to the file atomically.
// Returns 0 on success, 1 on failure.
// MS Windows version: the C rename() of Windows refuses to replace an existing
// file, so the atomic replacement uses MoveFileExA() with
// MOVEFILE_REPLACE_EXISTING (an atomic rename on NTFS) and
// MOVEFILE_WRITE_THROUGH (returns only once the move is on disk). fsync()
// becomes _commit() (which calls FlushFileBuffers) and unlink() becomes
// DeleteFileA(). Win32 errors are reported with their system message.
{
 char tmp_path[1024];

 // Defensive safety check: ensure the database exists
 if (sv_UserDb == NULL || sv_Upllen == 0) {
     perror("CRITICAL ERROR: There is no database to save to file");
     return 1;
 }

 // 1. Create a safe temporary filename in the same directory
 snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", user_file_path);

 // 2. Open the temporary file for writing
 FILE *fp = fopen(tmp_path, "w");
 if (!fp) {
     perror("CRITICAL ERROR: Cannot create temporary user file");
     return 1;
 }

 // 3. Write administrative header comment for clarity
 fprintf(fp, "# pardserver automated user database file\n");

 // 4. Dump the entire in-memory database to the file
 for (int i = 0; i < sv_Upllen; i++) {
     // Output format matches your loader: "username:timecreated:singleuse:hash"
     fprintf(fp, "%s:%lld:%"PRIu8":%s\n",
             sv_UserDb[i].username,
             (long long)sv_UserDb[i].time_created,
             sv_UserDb[i].single_use,
             sv_UserDb[i].password);
 }


 // 5. Force data to disk platter and close file safely
 fflush(fp);
 int fd = fileno(fp);
 _commit(fd); // Guarantees data physical safety before replacement
 fclose(fp);

 // 6. Atomically replace the old file with the new file
 if (!MoveFileExA(tmp_path, user_file_path,
                  MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
     fprintf(stderr, "CRITICAL ERROR: Atomic database replacement failed: %s\n",
                                                     win_errstr(GetLastError()));
     DeleteFileA(tmp_path); // Clean up the temp file on failure
     return 1;
 }

 return 0; // Success
}
#elif defined(__APPLE__)
int sv_save_user_file(const char *user_file_path)
// Rewrites the entire memory database back to the file atomically.
// Returns 0 on success, 1 on failure.
// macOS version: on macOS fsync() passes the data to the drive but does not
// make the drive flush its own cache to permanent storage (see fsync(2) on
// macOS). The guarantee intended by the Linux code is given by the macOS
// fcntl F_FULLFSYNC, with fsync() as a fall-back for file systems that do not
// support it. rename() is atomic on APFS and HFS+ as on Linux.
{
 char tmp_path[1024];

 // Defensive safety check: ensure the database exists
 if (sv_UserDb == NULL || sv_Upllen == 0) {
     perror("CRITICAL ERROR: There is no database to save to file");
     return 1;
 }

 // 1. Create a safe temporary filename in the same directory
 snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", user_file_path);

 // 2. Open the temporary file for writing
 FILE *fp = fopen(tmp_path, "w");
 if (!fp) {
     perror("CRITICAL ERROR: Cannot create temporary user file");
     return 1;
 }

 // 3. Write administrative header comment for clarity
 fprintf(fp, "# pardserver automated user database file\n");

 // 4. Dump the entire in-memory database to the file
 for (int i = 0; i < sv_Upllen; i++) {
     // Output format matches your loader: "username:timecreated:singleuse:hash"
     fprintf(fp, "%s:%lld:%"PRIu8":%s\n",
             sv_UserDb[i].username,
             (long long)sv_UserDb[i].time_created,
             sv_UserDb[i].single_use,
             sv_UserDb[i].password);
 }


 // 5. Force data to disk platter and close file safely
 fflush(fp);
 int fd = fileno(fp);
 if (fcntl(fd, F_FULLFSYNC) == -1) fsync(fd); // Guarantees data physical
                                              // safety before replacement
 fclose(fp);

 // 6. Atomically replace the old file with the new file
 if (rename(tmp_path, user_file_path) != 0) {
     perror("CRITICAL ERROR: Atomic database replacement failed");
     unlink(tmp_path); // Clean up the temp file on failure
     return 1;
 }

 return 0; // Success
}
#else
int sv_save_user_file(const char *user_file_path)
// Rewrites the entire memory database back to the file atomically.
// Returns 0 on success, 1 on failure.
{
 char tmp_path[1024];
 
 // Defensive safety check: ensure the database exists
 if (sv_UserDb == NULL || sv_Upllen == 0) {
     perror("CRITICAL ERROR: There is no database to save to file");
     return 1; 
 }

 // 1. Create a safe temporary filename in the same directory
 snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", user_file_path);

 // 2. Open the temporary file for writing
 FILE *fp = fopen(tmp_path, "w");
 if (!fp) {
     perror("CRITICAL ERROR: Cannot create temporary user file");
     return 1;
 }

 // 3. Write administrative header comment for clarity
 fprintf(fp, "# pardserver automated user database file\n");

 // 4. Dump the entire in-memory database to the file
 for (int i = 0; i < sv_Upllen; i++) {
     // Output format matches your loader: "username:timecreated:singleuse:hash"
     fprintf(fp, "%s:%lld:%"PRIu8":%s\n", 
             sv_UserDb[i].username, 
             (long long)sv_UserDb[i].time_created, 
             sv_UserDb[i].single_use, 
             sv_UserDb[i].password);
 }


 // 5. Force data to disk platter and close file safely
 fflush(fp);
 int fd = fileno(fp);
 fsync(fd); // Guarantees data physical safety before replacement
 fclose(fp);

 // 6. Atomically replace the old file with the new file
 if (rename(tmp_path, user_file_path) != 0) {
     perror("CRITICAL ERROR: Atomic database replacement failed");
     unlink(tmp_path); // Clean up the temp file on failure
     return 1;
 }

 return 0; // Success
}
#endif

int sv_is_password_strong(const char *password, char *errmsg) 
// Validates plain text password constraints.
// Returns 1 if password meets policy criteria.
// Returns 0 if password is too weak or invalid.
{
 if(password == NULL || errmsg == NULL){
    return 0;
 }

 size_t len = strlen(password);

 // Enforce modern length requirements (NIST standard)
 if(len < Password_min_length){
   snprintf(errmsg, ERRMSG_LEN, "Password rejected. Must be at least %zu"
                                       " characters long.",Password_min_length);
   return 0;
 }

 uint8_t has_upper   = 0;
 uint8_t has_lower   = 0;
 uint8_t has_digit   = 0;
 uint8_t has_special = 0;

 // Evaluate complexity in a single cache-friendly loop pass
 for(size_t i = 0; i < len; i++){
     unsigned char c = (unsigned char)password[i];
     
     if (isupper(c)) {
         has_upper = 1;
     } else if (islower(c)) {
         has_lower = 1;
     } else if (isdigit(c)) {
         has_digit = 1;
     } else if (ispunct(c) || isspace(c)) {
         has_special = 1;
     }
 }

 // Verify all complexity classes are satisfied
 if((!has_upper && Pwd_needs_upper) || (!has_lower && Pwd_needs_lower) ||
    (!has_digit && Pwd_needs_digit) || (!has_special && Pwd_needs_special)){
   // Safe initialization using bounds-checked snprintf
   snprintf(errmsg, ERRMSG_LEN, "Rejected. Missing requirements:");
   
   // Dynamically append text blocks safely using strncat with exact remaining lengths
   if(!has_upper   && Pwd_needs_upper)   strncat(errmsg, " capital letters",    ERRMSG_LEN - strlen(errmsg) - 1);
   if(!has_lower   && Pwd_needs_lower)   strncat(errmsg, " lower case letters", ERRMSG_LEN - strlen(errmsg) - 1);
   if(!has_digit   && Pwd_needs_digit)   strncat(errmsg, " numbers",            ERRMSG_LEN - strlen(errmsg) - 1);
   if(!has_special && Pwd_needs_special) strncat(errmsg, " special characters", ERRMSG_LEN - strlen(errmsg) - 1);
   
   strncat(errmsg, ".", ERRMSG_LEN - strlen(errmsg) - 1);
   return 0;
 }

 return 1; // Password meets policy
}

int sv_change_password(const char *current_plaintext, const char *new_plaintext,
                                                    int user_index,char *errmsg)
// Validates the current password and securely updates it with the new password
// string.
// Resets password lifetimes and clears single-use (OTP) restrictions in RAM.
// Flushes modifications atomically to disk storage using a temporary file.
// Returns 0 on success
// Returns 1 on failure.
{
 char msg[320];

 // 1. Defensive Boundary Check
 if(sv_UserDb == NULL || user_index < 0 || user_index >= sv_Upllen){
   show_message("Password update rejected: Invalid user database index.", 
                                                       "ERROR: ", MT_SESS_E, 0);
   snprintf(errmsg,ERRMSG_LEN,"Program bug: Invalid user database index.");
   return 1;
  }

 // 2. Authentication Validation: Verify the current password matches the user
 //    database record
 if(crypto_pwhash_str_verify(sv_UserDb[user_index].password, 
                              current_plaintext, 
                              strlen(current_plaintext)) != 0) {
   snprintf(msg, sizeof(msg), "Password change attempt for user '%s' failed: "
                          "Current password statement is incorrect. (IP: %s)", 
            sv_UserDb[user_index].username, sv_Client_IP);
   show_message(msg, "Security notice: ", MT_SEC, 0);
   // Don't give too much away, in case it is a hacker:
   snprintf(errmsg,ERRMSG_LEN,"Request denied.");
   return 1;
  }

 // New password must not be the same as the current password
 if(!strcmp(current_plaintext,new_plaintext)){
   snprintf(msg, sizeof(msg), "Password change for user '%s' aborted: "
                              "New password same as current password. (IP: %s)", 
                                  sv_UserDb[user_index].username, sv_Client_IP);

   show_message(msg, "Error: ", MT_ERR, 0);
   snprintf(errmsg, ERRMSG_LEN, "The new password cannot be identical to your"
                                                          " current password.");
   return 1;
  }


 // 3. Security Policy Compliance: Run password strength metrics on the new
 //    choice.
 if(!sv_is_password_strong(new_plaintext,errmsg)) {
   snprintf(msg, sizeof(msg), "Password change for user '%s' aborted: "
                              "New password fails strength metrics. (IP: %s)", 
                                  sv_UserDb[user_index].username, sv_Client_IP);
   // This is not really a high security issue - just a user who needs educating
   // So log as an error, not a security notice:
   show_message(msg, "Error: ", MT_ERR, 0);
   return 1;
  }

 // 4. Cryptographic Hashing: Transform the new raw payload via Argon2id
 char hashed_output[crypto_pwhash_STRBYTES];
 
 // Using interactive resources balances speed with strong defense-in-depth
 // overhead
 if(crypto_pwhash_str(hashed_output, 
                       new_plaintext, 
                       strlen(new_plaintext),
                       crypto_pwhash_OPSLIMIT_INTERACTIVE, 
                       crypto_pwhash_MEMLIMIT_INTERACTIVE) != 0) {
   show_message("FATAL CRYPTO ERROR: Memory or CPU resource exhaustion during"
                                                               " re-hashing.\n", 
                                                       "CRITICAL: ", MT_ERR, 1);
   snprintf(errmsg,ERRMSG_LEN,"There was a technical problem with hashing.");
   return 1;
  }

 // 5. Commit State Modifications Securely to RAM
 strcpy(sv_UserDb[user_index].password, hashed_output);
 sv_UserDb[user_index].time_created  = time(NULL); // Reset password aging countdown timer
 sv_UserDb[user_index].single_use    = 0;          // Permanent unlock; clears all OTP sandboxes
 sv_UserDb[user_index].strike_count  = 0;          // Clear out any residual historical strikes
 sv_UserDb[user_index].lockout_until = 0;

 // 6. Sync Runtime Memory Context to Persistent Storage Atomically
 if(sv_save_user_file(sv_Userfile) != 0){
   snprintf(msg, sizeof(msg), "CRITICAL: Password changed in RAM for user '%s',"
                                  " but atomic disk flushing failed! (IP: %s)", 
                                  sv_UserDb[user_index].username, sv_Client_IP);
   show_message(msg, "Security notice: ", MT_SEC, 0);
   snprintf(errmsg,ERRMSG_LEN,"Could not save your changes. Your new password"
    " will work until the server restarts/resets, then your old password may"
    " need to be used. Contact your system administrator.");
   return 1;
 }

 snprintf(msg,sizeof(msg),"User '%s' successfully updated their password . "
                                "All lifecycle tracking timers reset. (IP: %s)", 
                                  sv_UserDb[user_index].username, sv_Client_IP);
 show_message(msg, "Security notice: ", MT_SEC, 0);

 return 0; // Success
}


int sv_match_credentials(char *unamestr, char *passwdstr,int *udx)
// Check if the username in unamestr is on the list of users and, if it is, if
// the supplied password (in passwdstr) matches the hashed password of that user.
// Returns 1 if the username is present but the password doesn't match.
// Returns 2 if the username is not on the list
// Returns 3 if the username is present but the user is forbidden from log in
//           attempts due to 3 consecutive password failed attempts.
// Returns 4 if a user attempts to log in with an expired password.
// Returns 5 if a user attempts to use a one-time password more than once
// Returns 0 if the username:password pair are on the list (a match).
{
 int idx=0;        
 time_t now = time(NULL);
 char msg[320];                                     
 int retval = 2;
 uint8_t dbedited = 0;

 // Defensive safety check: ensure the database was successfully loaded
 if (sv_UserDb == NULL || sv_Upllen == 0) {
     return 2; 
 }

 for(idx=0; idx<sv_Upllen; idx++){
    // First look for the username entry in the sv_UserDb struct
    if(strcmp(unamestr, sv_UserDb[idx].username) != 0) continue;
    
    // If we get here, we found a match for the username.
    
    // Only proceed to check password if the user has not reached their
    // '3 strikes and you're out' limit:
    if(Strike_limit > 0 && sv_UserDb[idx].strike_count >= Strike_limit){
      if(now < sv_UserDb[idx].lockout_until){
        // Update the lockout to Strikeout_delay seconds from THIS new attempt
        sv_UserDb[idx].lockout_until = now + Strikeout_delay; 
        // To prevent timing attacks, simulate the cryptographic delay anyway
        // Use a dummy hash string that always fails validation
        char dummy_hash[crypto_pwhash_STRBYTES]= "$argon2id$v=19$m=16,t=2,p=1$ZHVtbXlzYWx0$dummyhash";
        if(crypto_pwhash_str_verify(dummy_hash, passwdstr, strlen(passwdstr)));
        // Send the bad news
        dbedited = 1;
        retval = 3; goto end_credentials_check;
      } else {
        // Lockout expired! Reset strikes automatically
        sv_UserDb[idx].strike_count  = 0;
        sv_UserDb[idx].lockout_until = 0;
        dbedited = 1;
      }
    }

    // Verify the incoming raw plaintext password against the stored Argon2id hash.
    // crypto_pwhash_str_verify returns 0 on a perfect match.
    if(crypto_pwhash_str_verify(sv_UserDb[idx].password, passwdstr, strlen(passwdstr)) == 0) {
      // Success: password and username both match

      if (sv_UserDb[idx].strike_count > 0) {
          sv_UserDb[idx].strike_count = 0;
          dbedited = 1;
      }

      // Check if the password has expired
      if(Password_expiry_timeout > 0 &&
         (now - sv_UserDb[idx].time_created) >= Password_expiry_timeout) {
        snprintf(msg, sizeof(msg), "User '%s' rejected: Password has expired."
                                         " (IP: %s)", unamestr, sv_Client_IP);
        show_message(msg, "Security notice: ", MT_SEC, 0);
        // Return specific expiry code to intercept in ZeroMQ loop
        retval = 4;  goto end_credentials_check;
       }

      if(sv_UserDb[idx].single_use){
        if(sv_UserDb[idx].single_use>1){
          snprintf(msg, sizeof(msg), "User '%s' rejected: First-time OTP reset"
                                 " required. (IP: %s)", unamestr, sv_Client_IP);
          show_message(msg, "Security notice: ", MT_SEC, 0);
         }
        // Accept, but only so as to change thier password.
        retval = 5; goto end_credentials_check;
       }

      // Convey the good news
      retval = 0; goto end_credentials_check;
    } else {
      // Username present, but password failed validation
      // 1. Increase the strike count for this user
      sv_UserDb[idx].strike_count++;
      dbedited = 1;
      // 2. If they're struck out, mark the time needed to auto-reset
      if(Strike_limit > 0 && sv_UserDb[idx].strike_count >= Strike_limit) {
        // Lock user out for Strikeout_delay seconds
        sv_UserDb[idx].lockout_until = now + Strikeout_delay;
        // Log the block event next to the user name
        snprintf(msg, sizeof(msg), "User '%s' locked out due to strike limit"
                                   " (IP: %s).", unamestr, sv_Client_IP);
        show_message(msg,"Security notice: ",MT_SEC,0);
       }      
      // 2. Convey the bad news
      retval = 1; goto end_credentials_check;
    }
 }
 snprintf(msg, sizeof(msg), "Unknown username login attempt detected (IP: %s)."
                                                                , sv_Client_IP);
 show_message(msg, "Security notice: ", MT_SEC, 0);

 end_credentials_check:

 if(dbedited){
   // The database was changed in RAM, so no flush this to the database file
   // persistent storage:
   if(sv_save_user_file(sv_Userfile)){
      snprintf(msg, sizeof(msg), "Failed to flush modified RAM user database to"
       "persistent storage after login request from user '%s' (IP: %s)",
                                                        unamestr, sv_Client_IP);
        show_message(msg,"Security notice: ",MT_SEC,0);
    }
   // This is not fatal for continued running of this server instance but the
   // sysadmin need to look into it. Do not alter the return code.
  }

 *udx=idx;

 return retval;
}

void sv_reset_strike_counts(void)
// Resets the strike counts of all users to 0.
// This will be called after server shutdown and before restart so the sysadmin
// can reset the strike counts of blocked users to give them another change to
// log in.
{
 int idx;                                              
 
 // Defensive safety check: ensure the database was successfully loaded
 if (sv_UserDb == NULL || sv_Upllen == 0) {
     return; 
 }

 for(idx=0; idx<sv_Upllen; idx++){
    sv_UserDb[idx].strike_count = 0;
 }
 return;
}

void sv_update_message_header(int msg_error_state)
// Prepares the message header to send (or resend) a message to the client.
{
 sprintf(sv_Msghdr[0],"%s",sv_Pardname); // (i.e. server name)

 switch(msg_error_state){
     case sv_MES_NONE:
     break;
     case sv_MES_ANON_ERR:
       sprintf(sv_Msghdr[1],"Caller"); 
       sprintf(sv_Msghdr[2],"-1");
     return;
     case sv_MES_USER_ERR:
       sprintf(sv_Msghdr[1],"%s",sv_Client_id); 
       sprintf(sv_Msghdr[2],"-1");
     return;
     default: // This shouldn't happen unless the source code was edited wrongly
      show_message(": Program Error: 1",fnn,MT_ERR,1);
     break;
   }

 sprintf(sv_Msghdr[1],"%s",sv_Client_id); 
 if(sv_Dont_update_Mdx) sv_Dont_update_Mdx=0; else sv_Mdx++;
 sprintf(sv_Msghdr[2],"%d",sv_Mdx); 
 
 return;
}

int sv_transmit_data_zmq(void *socket,int have_blob)
// Sends a message to the client. The message is contained in sv_Msghdr[3][],
// sv_Triad and (if have_blob==1) sv_Blob.
// Returns 0 on success and -1 on failure and -2 on timeout.
{
 int mpart,rc;
 size_t rlen;
 char msgtxt[400];
 
 sprintf(msgtxt,"(%s)[%d]: ",sv_Pardname,sv_Mdx);
 show_message(msgtxt,">",MT_SESS_S,1);
 sv_Previous_blob=0;
 
 // Send the header
 for(mpart=0;mpart<3;mpart++){ 
    sprintf(msgtxt,"[%d]:%s",mpart+1,sv_Msghdr[mpart]);
    show_message(msgtxt," ",MT_SESS_S,1);
    rlen=strlen(sv_Msghdr[mpart]);
    rc=zmq_send(socket,sv_Msghdr[mpart],rlen,ZMQ_SNDMORE);
    if(rc!=rlen) {
         if(errno==EAGAIN){show_message("\n","",MT_SESS_S,1); return -2;}
         else {show_message("\n","",MT_SESS_S,1);return -1;}
     }
   }
 // Send the sv_Triad
 show_message(sv_Triad," ",MT_SESS_S,1);
 rlen=strlen(sv_Triad);
 rc=zmq_send(socket,sv_Triad,rlen,(have_blob)?ZMQ_SNDMORE:0);
 if(rc!=rlen) {
    if(errno==EAGAIN){show_message("\n","",MT_SESS_S,1); return -2;}
    else {show_message("\n","",MT_SESS_S,1);return -1;}
   }
 // Send sv_Blob if present
 if(have_blob){
   sv_Previous_blob=1;
   show_message("[BLOB]"," ",MT_SESS_S,1);
   rc=zmq_send(socket,sv_Blob,sv_Blobsize,0);
   if(rc!=sv_Blobsize) {
     if(errno==EAGAIN){show_message("\n","",MT_SESS_S,1); return -2;}
     else {show_message("\n","",MT_SESS_S,1);return -1;}
    }
  }
   
 show_message("\n","",MT_SESS_S,1);
 return 0;
}

void sv_set_log_off_flags(void)
// Sets global flags to show that no user is logged in to the server.
{
 if(sv_Client_logged_in==0) return; // The function need not have been called.
 sv_Mdx=0;
 sv_Climdx=0;
 sprintf(sv_Username,"Caller");
 sprintf(sv_Client_id,"%s","");
 sv_Merrdx=0; 
 sv_Resendidx=0;
 sv_Client_logged_in=0;
 return;
}

void sv_log_off_client(void *socket,char *reason,int due_to_error)
// Logs a client off the server and sends them a message to indicate this.
{
 char msgtxt[320];
 char uname[TXT_BUFMAX];
 char timedatestr[32];
 time_t ts;

 if(sv_Client_logged_in==0) return; // The function need not have been called.
    
 if(due_to_error) sv_update_message_header(sv_MES_USER_ERR);
 else sv_update_message_header(sv_MES_NONE);
 sv_send_message(socket,0);

 sprintf(timedatestr,"%s",((time(&ts)) == -1)?
                                           "[Time not available]" : ctime(&ts));
 timedatestr[strlen(timedatestr)-1]='\0'; // Remove the terminal '\n'
 sprintf(uname,"%s",sv_Username);

 sv_set_log_off_flags();

 if(due_to_error){
   sprintf(msgtxt,
            "(%s)[%d]: Connection with the server was lost (at %s)"
            " due to an error (%s).\n",sv_Pardname,sv_Mdx,timedatestr,reason);
   show_message(msgtxt,">",MT_SESS_E,1);
  } else {
   sprintf(msgtxt,"(%s)[%d]: You have successfully logged off at %s. Bye.\n",
                                                sv_Pardname,sv_Mdx,timedatestr);
   show_message(msgtxt,">",MT_SESS_T,1);
  }
 sprintf(Log_SessionFilename, "Log_PARDsv.txt");
 close_session_logfile();
 sprintf(msgtxt,"%s: User %s logged out at %s.\n",
                                                 sv_Pardname,uname,timedatestr);
 show_message(msgtxt,"FYI: ",MT_SESS_T,0);
 return;
}

static int sv_recv_data(void *socket,int *have_blob)
// Listens for a response from the client and stores it in sv_Msghdr[3][],
// sv_Triad and sv_Blob. The above is all done via a call to sv_receive_data_zmq
// Sets have_blob to 1 if a sv_Blob was included in the message. 
// Applies the first stage of processing the message and return codes from
// sv_receive_data_zmq. Formulates and presents appropriate messages (including
// errors) and may forcibly log the client off the server if a fatal error is
// received from sv_receive_data_zmq.
// Returns 0 on success.
// Returns 1 on message received with error not requiring a log out.
// Returns 2 on message error that HAS resulted in a forced logout.
// Returns 3 on no message received (due to network timeout or
//           disconnect) - may be recovered by re-polling client at a
//           later time.
{
 char msgtxt[262];
 int rc;

 *have_blob=0; // This will be changed if a sv_Blob is read.
 
 // Get the client's response ...
 rc=sv_receive_data_zmq(socket);
 // If there has not been a timout or ZMQ error, print the first part of
 // any message to console.
 if(rc!=-1 && rc!=-2){
   sprintf(msgtxt,"(%s)[%s]: ",sv_Msghdr[1],sv_Msghdr[2]);
   show_message(msgtxt,"<",MT_SESS_C,1);
  }
 // Process the value returned from sv_receive_data_zmq:
 switch(rc){
     case -1: // Error with running zmq receive functions - the server should
     // automaticaly retry but if this error persists it suggests an
     // unrecoverable position so increment sv_Merrdx and when this reaches
     // sv_Merrmax the server will terminate its infinite listening loop (or re-
     //boot itself). Return the 're-try' code (3).
          sv_Merrdx++;
      return 3; // Return the 're-try' code (3).
     case -2:  // Network timeout. For the server this just means keep on
     // listenting but it provides an opportunity to interrupt the infinite
     // server loop if required. Don't increment sv_Merrdx.
      return 3;  // Return the 're-try' code (3).
     case -3:// At least one sent message is empty. Non-fatal. 
     case 1: // The wrong number of message strings 
     case 2: // was received. At least 4 are
     case 3: // required. Non-fatal.
        show_message("ERROR: Too few (or empty) words received from client.\n",
                                                                "",MT_SESS_E,1);
      return 1; // Return the 'non-fatal message format' error code (1).
     case -4:// At least one received message was too big to be a properly
             //constructed Pardus message. Non-fatal. 
        show_message("ERROR: Part of message was too big.\n","",MT_SESS_E,1);
      return 1; // Return the 'non-fatal message format' error code (1).
     case -5:// The message contained more than 5 parts.
        show_message("ERROR: Message has more than 5 parts.\n","",MT_SESS_E,1);
      return 1; // Return the 'non-fatal message format' error code (1).
     case -6:// The message was addressed to the wrong pard or received from an
             // invalid client 
        if(sv_Wrong_pard) 
           show_message("ERROR: Message addressed to the wrong Pardus.\n","",
                                                                   MT_SESS_E,1);
        if(sv_Wrong_client) 
           show_message("ERROR: Message received from an unlogged client.\n","",
                                                                   MT_SESS_E,1);
        if(sv_Wrong_version) 
           show_message("ERROR: Client misunderstood last message.\n","",
                                                                   MT_SESS_E,1);
      return 1; // Return the 'non-fatal message format' error code (1).
     case -7:// RAM allocation error - fatal - but give the user the
             // chance to make a clean exit. 
          show_message(
              "ERROR: RAM allocation failure. Clients will be logged off.\n","",
                                                                   MT_SESS_E,1);
       goto end_reception; // Force logout and return fata error code: 2
     case -8:// ZMQ flush error - fatal - but give the user the chance to make a
             // clean exit.
          show_message(
          "ERROR: Network communication failure. Clients will be logged off.\n",
                                                                "",MT_SESS_E,1);
       goto end_reception; // Force logout and return fata error code: 2
     case 4: // This is potentially correct - there is no sv_Blob
       *have_blob=0;
      break;
     case 5: // This is potentially correct - there is a sv_Blob
       *have_blob=1;
      break;
     default: // This should not happen unless sv_receive_data_zmq is altered
              // without adjusting related source code.
        show_message("Unknown response from sv_receive_data_zmq.",
                                                    "Program Error: ",MT_ERR,1); 
      goto end_reception;
   }

 // Print the received message details to the console (hide passwords).
 if(strcmp("login",sv_Msghdr[1])) show_message(sv_Triad," ",MT_SESS_C,1);
  else show_message("[*****]"," ",MT_SESS_C,1); 
 show_message("\n","",MT_SESS_C,1); 
 
 return 0; // Success. No errors.
 
end_reception:
  show_message(
      "Unrecoverable communication error - terminating connection with client.",
                                                            "ERROR: ",MT_ERR,0);
  sv_triad_set("TooManyCommsErrors");
  sv_log_off_client(socket,"TooManyCommsErrors",1);
  return 2; // Fatal error code (2)
}

int sv_send_message(void *socket,int have_blob)
// Sends a command stored in the global sv_Msghdr[3][], sv_Triad and
// (optionally) sv_Blob to the client with error handling.
// The function caller must set have_blob = 1 if there is a sv_Blob to send.
// The function caller must ensure the appropriate data is in those
// structures prior to calling this function - this is only the postal
// service!
// Returns 0 on success (although the message may be held in a queue if network
// is down or busy so success does not mean that the client has received the
// message yet.
{
 int send_says; 
    
 send_it:
 send_says=sv_transmit_data_zmq(socket,have_blob);
 switch(send_says){
      case 0: break; // All well - no more to do.
      case -1: // ZMQ Function failure - this is usually unrecoverable so just
               // give a message and return.
        show_message("A network function error occurred when sending the request. Server will attempt a reboot.","ERROR: ",MT_ERR,0);
        return 1;
      case -2: // Message was NOT sent due to a network timeout/disconnection.
               // This gives an opportunity for the program to check or change
               // the value of sv_Server_run so the infinite server loop can
               // broken by an interrupt if desired (by setting sv_Server_run=0)
               // Otherwise the server simply goes back to looking for incoming
               // calls.         
               // Stop server if that is what admin wants
              if(sv_Server_run==0) return 1;
       goto send_it;
      default: // Should not happen unless there has been a programming error.
        show_message("Unknown response from sv_transmit_data_zmq.",
                                                    "Program Error: ",MT_ERR,1);
      return 1;
  }
 return 0;
}

int sv_receive_message(void *socket,int *have_blob)
// Receives a client message with with error handling.
// Returns 0 on success
// Returns 1 on failure
// Sets have_blob=1 if there is a sv_Blob or 0 if not.
{
 int client_says,logerr;

 listen_for_client_message:
 client_says=sv_recv_data(socket,have_blob);
 // Any error messages were given in sv_recv_data() so no need to print those
 // here for each case.
 switch(client_says){ 
      case 0: break; // All is good - no more to do.
      case 1: // Reply was received from client but either not understood
              // (unexpected format) or addressed to the wrong server
              // (sv_Wrong_pard set to 1). Server sends the appropriate error
              // message back and keeps listening (we don't want to return an
              // obviously uninterpretable message back to the main server loop
              // function).
          if(sv_Wrong_pard){
             if(sv_triad_set("ERROR WrongPard Ready")) return 1;
            }
          else if(sv_Wrong_client){
             if(sv_triad_set("ERROR Busy Ready")) return 1;
            }
          else if(sv_Wrong_version<0){ // This will *only* occur if there is a
          // client logged in. Need to reformulate and resend last message
          // according to client's comms protocol version (which is the
          // (negative) value stored in 'sv_Wrong_version'). However as we
          // (client and server) are both at version 1, the only thing to do
          // here is just resend the last message or log the user off due to
          // too many resends.
                  if(sv_Resendidx==sv_Resendmax){ // Too many tries - log off.
                    if(sv_triad_set("ERROR TooManyResends Bye"))return 1;
                    logerr=1;
                    sv_log_off_client(socket,sv_Triad,logerr);    
                    goto listen_for_client_message;               
                  } else { // Re-send the previous message
                    show_message("Resending last message.","FYI: ",MT_INFO,0);
                    sv_update_message_header(sv_MES_NONE); 
                    if(sv_send_message(socket,sv_Previous_blob)) return 1;
                    goto listen_for_client_message; 
                  }
              }
          else {if(sv_triad_set("ERROR WrongFormat Ready")) return 1;}
          // Send reply
          if(sv_Client_logged_in) sv_update_message_header(sv_MES_USER_ERR);
          else sv_update_message_header(sv_MES_ANON_ERR);
          if(sv_send_message(socket,0)) return 1;
      goto listen_for_client_message;
      case 2: // If a client was logged in, the seriousness of the comms error
              // means they have been forcibly logged off (by sv_recv_data).
              // This is unrecoverable so we try to start over by rebooting the
              // server.
      return 1;
      case 3: // Reply was NOT received due to a network timeout or a connection
              // fault. A server's behaviour here is to try again but I should 
              // distinguish between timeouts (these can be infinitely re-tried)
              // and comms failures (return fatal error if too many re-tries
              // fail).          
              // Stop server if that is what admin wants
              if(sv_Server_run==0) return 1;

              if(sv_Merrdx==sv_Merrmax) return 1;
           
              // Note - this will allow me to break out of the infinite server
              // 'while' loop because I will get here after every timeout of
              // silence (no messages coming into the server for N secondsm and
              // N is currently set at 4 seconds).   
      goto listen_for_client_message;
  }
  return 0;
}

int sv_triad_set(char *tstring)
// Copies tstring into sv_Triad allocating appropriate memory to do so.
// Returns 0 on success and 1 on RAM allocation failure (serious - fatal)
{
 size_t len;
 
 len=strlen(tstring); 
 if(len<1) return 0; // Empty string.
  if(len>=TRIADMAX){ // Error: Size too big - programmer must ensure this
                     // doesn't happen
      show_message("Truncating sv_Triad.","Program Error: ",MT_ERR,1);
      len=TRIADMAX;
    } 
 // (Re-)Allocate memory for client sv_Triad
 if(sv_triad_resize(len+1)){
   show_message("Failed RAM allocation for sv_Triad.","Program Error: ",
                                                                      MT_ERR,0);
   return 1;
  }
 // Copy the (assumed string) into sv_Triad and terminate it.
 memcpy(sv_Triad,tstring,len);sv_Triad[len]='\0';
 return 0;
}

int sv_triad_argc(void)
// Returns the number of words in a sv_Triad string
{
 return num_words(sv_Triad," ");   
}

char *sv_triad_argv(size_t narg) 
// Returns a pointer to the narg-th argument in the sv_Triad string.
// The very first word in the sv_Triad will have narg = 1 (not 0).
// Returns NULL if memory allocation fails or sv_Triad is of zero length.
{
 char *currstrpos;  
 size_t tparts,len;
 
 len=strlen(sv_Triad);
 if(len==0) return NULL; else len++;
 
 if(sv_triadarg_resize(len)){
  show_message("Failed to allocate RAM for argument parser.","ERROR: ",
                                                                      MT_ERR,0);
  return NULL;
 }

 memcpy(sv_Triadarg,sv_Triad,len);
 tparts=0;
 currstrpos=strtok(sv_Triadarg," ");
 while(currstrpos!=NULL){
        len=strlen(currstrpos);
        if(len){
            tparts++;
            if(tparts==narg) break;
        }
        currstrpos=strtok(NULL," ");
      }

 return currstrpos;
}

int sv_triad_type(void)
// Reads the first word in a sv_Triad.
// Returns a TT_ constant or sv_TT_UNDEF
{
 char *argptr;
 
 argptr=sv_triad_argv(1);
 
      if(!strcmp(argptr,"OK")) return sv_TT_OK;
 else if(!strcmp(argptr,"ERROR")) return sv_TT_ERROR;

 // We shouldn't reach here other than by programmer error
 show_message("Unknown sv_Triad type.","Program Error: ",MT_ERR,0);
 return sv_TT_UNDEF;   
}

int sv_triad_state(void)
// Reads the last word in a sv_Triad and returns a TS_ constant or sv_TS_UNDEF
{
 int argnum;
 char *argptr;
 
 argnum= sv_triad_argc();
 if(argnum<3) goto state_undefined;
 
 argptr=sv_triad_argv(argnum);
 
      if(!strcmp(argptr,"Ready")) return sv_TS_READY;
 else if(!strcmp(argptr,"Bye"))   return sv_TS_BYE;
 
 state_undefined:
 return sv_TS_UNDEF;   
}

void sv_client_id_gen(char *unamestr, char *sv_Client_id)
// Generates a 'unique' id string for the current user.
// First it tries to make a string using the current time and date numbers.
// If that fails it resorts to generating a pseudo random number prefixed with
// an 'r' (so you know it failed to get time-date info if you see the 'r') 
{
 time_t tmt   = time(NULL);
 struct tm ct = *localtime(&tmt);
 int len;

 if(tmt==-1)
    sprintf(sv_Client_id,"%s_r%d",unamestr,
               (int)((float)(INT_MAX) * (float)rand()/((float)RAND_MAX + 1.0)));
   else {
    // First, put the username in the string
    len = sprintf(sv_Client_id, "%s_", unamestr);
    // Then, append the formatted time starting at the end of the username
    strftime(sv_Client_id + len, 20, "%Y%m%d_%H%M%S", &ct);
   }

  return;
}

int sv_send_pardcap_ins(uint32_t ciid, char *errmsg, char *insname, int preset)
// Sends the ps_Instruction to PARDcap for execution and receives the report
// back. If preset is 0 it forms a minimalist ps_Instruction based on the
// supplied ciid otherwise it assumes the caller has properly formed a complete
// ps_Instruction and just sends it.
// The reply from PARDcap is checked for errors.
// Returns CMD_OK if there is no error
// Returns CMD_ERR if an error occurred and prints an error message into errmsg
{
 int rc,timeo;
 int cmdstatus=CMD_OK;

 // If user did not set the required fields of the ps_Instruction struct it
 // it means this is a simple 'CIID-only' ps_Insruction, so set it here:
 if(!preset){
   // Reset the ps_Instruction struct to the unused state
   ps_ins_reset();
   // Form the ps_Instruction (set the members we need to use)
   ps_ins_set(PS_INS_CIID,&ciid,NULL); // Error can't occur with this use.
  }
 // Encapsulate and send the ps_Instruction to the PARDcap server
 if((Pard_Err=ps_ins_send())!=PS_CB_OK) {
   sprintf(errmsg,"%s",Pard_err_str());
   cmdstatus=CMD_ERR; goto end_sv_send_pardcap_ins;
  }
 // Wait for (then process) the response (10 minutes wait max)
 show_message(": PARDclient: Waiting for response ...",fnn,MT_INFO,0);
 for(timeo=0;timeo<PARDCAP_TIMEOUT;timeo++){
   rc=ps_receive_blob(1000); // Poll at 1s timeout intervals
   if(rc!=2) break;
   show_message("+","",MT_INFO,1);
  }
 if(rc){
   // Something went wrong receiving blob
   // Time out or other error occurred during receiving blob
   ps_print_recv_error(rc,errmsg);
   cmdstatus=CMD_ERR; goto end_sv_send_pardcap_ins;
  } 
 // If we get to here then we received an encapsulated blob OK and stripped it
 // of its header to leave an info blob - so check it:
 if((Pard_Err=ps_check_blob_from_server())!=PS_CB_OK) {
  sprintf(errmsg,"%s",Pard_err_str());
  cmdstatus=CMD_ERR; goto end_sv_send_pardcap_ins;
 }
 // If we get to here then the info blob contents check out as OK and the
 // ps_Instruction struct contains either our desired data (the result of the
 // PARDcap server doing what we asked) or an error code if it failed.
 // First let's deal with the failure scenario:
 if(ps_Instruction.ciid==PS_CIID_ERRC){
  // Translate the error codes we expect to get for this command into a
  // string for bloberr and reply
  Pard_Err=ps_Instruction.err[0];
  sprintf(errmsg,"%s",Pard_err_str());
  cmdstatus=CMD_ERR; goto end_sv_send_pardcap_ins;
 }
 // Another erroneous situation is if comms got mixed up so we are not
 // receiving the info blob for this request - check for it
 if(ps_Instruction.ciid!=ciid){
   sprintf(errmsg,"Wrong PARDcap report for %s",insname);
   cmdstatus=CMD_ERR; goto end_sv_send_pardcap_ins;
  }

 end_sv_send_pardcap_ins:
 return cmdstatus;
}

int sv_process_cuss(uint8_t siid, char *cmdnom, char *cmdval, char *errmsg,
                    char *cmdtriad, int *sending_bloberr,uint8_t valtype)
// Helper function to set, send and process an instruction of the CUSS type
// (custom framegrabber setting) where the sub-instruction only requires a
// single integer or double argument (constained in cmdval) or no argument (in
// which case cmdval will be ignored and you may pass it as NULL).
// siid is the SIID code for the sub-instruction (one of the PS_CIID_ #defines)
// cmdnom is a string with the name of the command (for making an error message)
// cmdval is a string containing the numerical value to set.
// valtype specified whether the value is an int32_t or a double or a uint8_t
// Returns CMD_OK if all went well, prints a success triad in cmdtriad
// Returns CMD_ERR if there was an error, prints a failure triad in cmdtriad,
//                 puts an error message in errmsg and sets *sending_bloberr=1
{
 int rv;
 uint32_t ui32;
 int32_t si32;
 uint8_t ui8;
 double doutmp;
 
 ps_ins_reset(); // Reset the ps_Instruction struct to the unused state
 // Form the ps_Instruction (set the members we need to use)
 ui32=PS_CIID_CUSS;  ps_ins_set(PS_INS_CIID,&ui32,NULL);
 ps_ins_set(PS_INS_SIID,&siid,NULL);
 // Value you want PARDcap to set this SIID parameter to
 switch(valtype){
  case CUSS_VAL_INT1:
    sscanf(cmdval,"%"SCNd32,&si32);  ps_ins_set(PS_INS_IVAL1,&si32,NULL);
  break;
  case CUSS_VAL_ONFF:
    si32=sv_evalonoff(cmdval);
    if(si32<0){
      sprintf(errmsg,"sv_process_cuss failed to covnvert ON/OFF string");
      rv=CMD_ERR;
      goto error_sv_process_cuss;
     } else ps_ins_set(PS_INS_IVAL1,&si32,NULL);
  break;
  case CUSS_VAL_ONFT:
    si32=sv_switch_string(cmdval);
    if(si32!=SWS_ON && si32!=SWS_OFF && si32!=SWS_FITS) {
      sprintf(errmsg,"sv_process_cuss failed to covnvert ON/OFF/FITS string");
      rv=CMD_ERR;
      goto error_sv_process_cuss;
     } else ps_ins_set(PS_INS_IVAL1,&si32,NULL);
  break;
  case CUSS_VAL_DOU1:
    doutmp=atof(cmdval);  ps_ins_set(PS_INS_DVAL1,&doutmp,NULL);
  break;
  case CUSS_VAL_FRMT:
    ui8=(uint8_t)ps_fmtchoice(cmdval);  ps_ins_set(PS_INS_SAFMT,&ui8,NULL);
  break;
  case CUSS_VAL_BYTE:
    sscanf(cmdval,"%"SCNu8,&ui8);  ps_ins_set(PS_INS_SAFMT,&ui8,NULL);
  break;
  case CUSS_VAL_NONE:
  break;
  default:
      sprintf(errmsg,"sv_process_cuss programming error");
      rv=CMD_ERR;
      goto error_sv_process_cuss;
  return rv;
 }
 // Send the command and process the result
 rv=sv_send_pardcap_ins(PS_CIID_CUSS,errmsg,cmdnom,1);

error_sv_process_cuss:
 if(rv==CMD_ERR){ 
    sprintf(cmdtriad,"ERROR FailedCmd Ready");
    *sending_bloberr=1;
  } else sprintf(cmdtriad,"OK Done Ready");

 return rv;
}

double sv_posdist_to_maxmsdenomsteps(uint8_t id, double posdist)
// Convert the posistion or distance value, posdist, into number of steps at the
// stepper motor's maximum microstep denominator step size using its calibration
// factor. This is why we need the id of the motor. It is assumed that the motor
// exists and is defined so the caller must ensure that is the case.
// The value posdist is expected to be supplied in units of either atomic steps
// or real world units depending on the value of sv_Unitype[ACT_STEPPER]:
//                            0 = number of maxmsdenom steps
//                            1 = calibrated units
// We use double as the return value to keep the code general for drivers that
// might have microstep sizes in both powers of 2 and non-powers of 2 but for
// the Trinamic drivers currently used in the PUMA CNC stage this could be done
// all in integer steps.
// For the case where the input is a simple or relative distance, the return
// value could be cast into a uint32_t (after adding 0.5). Where the input is an
// absolute destination position for the motor, a second stage of processing is
// required to compare this position to the motor's current position and find
// the distance between the two (and direction of travel to get to that
// destination position. In that case the return value should be retained as
// double for use as input to that second stage.
// Returns the number of microsteps
{
 // If in Unit Mode (e.g. microns), scale to Atomic.
 // maxcalibri is the number of atomic steps per real world unit.
 if(sv_Unitype[ACT_STEPPER]) 
     return posdist * Stepperlist[id]->maxcalibri;
 
 // If in Step Mode, assume the input is ALREADY the target unit.
 return posdist;
}

int sv_stepper_move_to(uint8_t id, double position, uint32_t duration,
                                                                   char *errmsg)
// Move stepper 'id' to 'position' where 'position' is expected to be an
// absolute coordinate position from the homed ( = 0) position in the motor's
// maxmsdenom microstep size number of steps (atomic step size = 1/maxmsdenom).
// This function assumes the caller has already checked that stepper 'id' has
// been defined (it exists) and that there is an acceleration profile for
// steppers (so p_acc, etc. are all set) but you supply the duration for the
// motion in microseconds in the duration argument (use 0 for auto-calculation).
// This function calculates the absolute number of steps to take and the
// direction to take them in in order to go to the desired position from
// wherever the motor happens to currently be. It then calls sv_stepper_move_by
// to actually do the motion after changing the direction of the motor if that
// is required.
// Returns CMD_OK if all went well.
// Returns CMD_ERR if something went wrong and errmsg contains the reason
{
 int cmdstatus;
 uint8_t infotype,reqdirn;
 uint32_t dirsteps;

 cmdstatus= CMD_OK; // We'll change this if things go wrong.

 // Initialise Order_ID.otype so we know for sure that if it becomes ORDER_SET
 // later then this was due to the need for a function change and not just a
 // 'left over' value from a previous function.
 Order_ID.otype  = ORDER_STEPPER_MOVE;

 // Ensure the local motor struct stepcount is up to date
 switch(id){
   case ACT_ID_STAGE_X:
    Stepperlist[ACT_ID_STAGE_X]->stepcount=PMRep1;
   break;
   case ACT_ID_STAGE_Y:
    Stepperlist[ACT_ID_STAGE_Y]->stepcount=PMRep2;
   break;
   case ACT_ID_STAGE_Z:
    Stepperlist[ACT_ID_STAGE_Z]->stepcount=PMRep3;
   break;
   default:
       sprintf(errmsg,"sv_stepper_move_to is only for use with stage steppers");
       cmdstatus=CMD_ERR; goto end_moveto;
   break;
  }

 // Convert position into dirsteps and direction by comparison with the
 // actuator's current position stepcount and set the Order_ID.otype to 
 // ORDER_SET if a change in direction is required:
 if(sv_moveto_to_moveby(id,&dirsteps,errmsg,&reqdirn,&infotype,position)){
    cmdstatus=CMD_ERR; goto end_moveto;
   }

 // Special case for dirsteps=0 - there is nothing to do.
 if(dirsteps==0) goto end_moveto;
              
 // If we need to change direction, do it now. The sv_moveto_to_moveby function
 // will have filled in the order form for this already:
 if(Order_ID.otype == ORDER_SET){
    Pard_Err=as_send_order();
    if(Pard_Err!=PARDU_ERR_NONE){
       sprintf(errmsg,"%s",Pard_err_str());
       cmdstatus=CMD_ERR; goto end_moveto;
      } 
    // If it worked we also need to set the equivalent local struct
    // members to keep things in sync.
    act_sync_to_local(ACT_STEPPER,id,1);
  }

 // If we get to here then the motor is pointing in the right direction and the
 // number of steps to move is > 0 and has been set into dirsteps. The rest is
 // just identical to a 'move by' command so use that function to do tha actual
 // motion:
 cmdstatus = sv_stepper_move_by(id, dirsteps, duration, errmsg);

end_moveto:
 return cmdstatus;
}

int sv_stepper_move_by(uint8_t id, uint32_t dirsteps, uint32_t duration,
                                                                   char *errmsg)
// Move stepper 'id' dirsteps number of steps in the direction the motor's is
// currently set to move in. The dirsteps value is interpreted as steps in the
// motor's current msdenom microstep size.
// This function assumes the caller has already checked that stepper 'id' has
// been defined (it exists) and that there is an acceleration profile for
// steppers (so p_acc, etc. are all set) but you supply the duration for the
// motion in microseconds in the duration argument (use 0 for auto-calculation).
// The reason for the above assumptions is that this is an efficient convenience
// function for server-initiated (as opposed to client initiated) single axis
// motion such as occurs during the af_auto_focus function. So those qualifying
// conditions are assumed to be checked once by the calling function and not
// repeated with every call to this function. Also, because this is a server
// side motion function, control will not be immediately passed back to the
// client to issue mandata interruptiva. Instead this function will wait for
// (actively look for) a PMR and only return when found (or if there is an error
// or timeout looking for the PMR). The PMR search timeout will be 5 x the
// calculated motion duration or 5 ms (whichever is the longer). This means the
// caller of this sv_stepper_move_by function (or the sv_stepper_move_to 
// function which uses this function to do the actual motion) will not have to
// implement a PMR search waiting for the motion to end - control will not be
// returned to the calling function until the motion ends or there is an error.
// sv_stepper_set_dirn first).
// Returns CMD_OK if all went well.
// Returns CMD_ERR if something went wrong and errmsg contains the reason
{
 int cmdstatus;
// uint8_t ui83; // DEBUG
 uint32_t ui32;
 double pmr_timeout_musecs;

 cmdstatus= CMD_OK; // We'll change this if things go wrong.
 // Special case for dirsteps=0 - there is nothing to do.
 if(dirsteps==0) goto end_moveby;

 // AUDIT (26.10.2026), advice 8c: only a MATURE stepper may move, as advice 8
 // requires in act_moveby and act_moveto.  This function makes the server's
 // own motions, which never pass through act_moveby or act_moveto: every
 // move of af_auto_focus() (MOVE_Z_BY, and MOVE_Z_TO through
 // sv_stepper_move_to(), which ends here).  A pin unset with 'act_set ... 255'
 // lowers the maturity (act_update_maturity()), but the firmware checks only
 // homed_to and ena_state before a move, and would pulse pin 255.
 if(Stepperlist[id]->maturity < adult_stepper(Stepperlist[id]->driver_type)){
    sprintf(errmsg,"Stepper %"PRIu8" is immature (a pin or a setting is"
                   " missing): it cannot move.",id);
    cmdstatus=CMD_ERR; goto end_moveby;
   }

 // Fill in the ID order form
 Order_ID.actype = ACT_STEPPER;
 Order_ID.actid  = id;
 Order_ID.otype  = ORDER_STEPPER_MOVE;

 // Complete the DirSteps order form
 Order_DirSteps.dirsteps_z=dirsteps;

 // Calculate duration if user requested auto-calc (i.e. duration=0)
 sv_Accparam[ACT_STEPPER].duration = duration;

 // Calculate the motion parameters & complete the Motion order form
 act_calc_motion(dirsteps,
                        sv_Accparam[ACT_STEPPER].duration,
                        sv_Accparam[ACT_STEPPER].p_acc,
                        sv_Accparam[ACT_STEPPER].delay_min,
                        sv_Accparam[ACT_STEPPER].delay_max_min);

 // Force  Order_Motion.delay_max to be sensible if it isn't.
 // Be sure to set id_a apporpriately over at act_moveto before
 // coming here from there.
 ui32 = (2*Stepperlist[id]->pulwidth);
 if(Order_Motion.delay_max < ui32) Order_Motion.delay_max=ui32;

 // Send the order to the Parduino and assess the report
 Pard_Err=as_send_order();
 if(Pard_Err!=PARDU_ERR_NONE){
    sprintf(errmsg,"%s",Pard_err_str());
    cmdstatus=CMD_ERR; goto end_moveby;
   }

 // Motion commands return from the BOB as soon as the motion starts so we must
 // look for the PARDUS Move Report (PMR) signal that is returned when the
 // motion ends before returning control to the caller of this motion function.
 // We start by waiting for 110% of the expected motion duration before checking
 // for the PMR. If it is not there then we wait in 10% (of expected motion
 // duration) chunks of time for it before giving up with an error if there is
 // no PMR after 5 times the expected motion duration elapses.
 pmr_timeout_musecs = 5.0 * (double)sv_Accparam[ACT_STEPPER].duration;
 if(pmr_timeout_musecs < 5000.0) pmr_timeout_musecs = 5000.0; // 5ms floor
 switch(sv_wait_for_pmr(pmr_timeout_musecs, errmsg)){
   case SV_PMR_FOUND: break; // All good - just return
   case SV_PMR_TIMEOUT:
    sprintf(errmsg,"Motion timed out waiting for PMR");
    cmdstatus=CMD_ERR;
   break;
   case SV_PMR_ERROR: // errmsg will have been set via sv_wait_for_pmr
    cmdstatus=CMD_ERR;
   break;
   default: // Should never happen unless there is a program bug
    sprintf(errmsg,"sv_stepper_move_by: sv_wait_for_pmr returned an"
                                                     " unexpected value (bug)");
    cmdstatus=CMD_ERR;
   break;
  }

 end_moveby:

 return cmdstatus;
}

int sv_stepper_set_dirn(uint8_t id, uint8_t dirn, char *errmsg)
// Ensure the motor is pointing in direction dirn. Change it's direction to dirn
// if that is not the case, otherwise just return without error.
// Return CMD_OK if the motor is pointing in direction dirn.
// Return CMD_ERR and set errmsg if the function failed to get the motor
// to point in directiob dirn.
{
 int cmdstatus;
 cmdstatus= CMD_OK; // We'll change this if things go wrong.

 if(dirn != Stepperlist[id]->currdirn){
   // Fill in the order form to set the direction
   Order_ID.actype = ACT_STEPPER;
   Order_ID.actid  = id;
   Order_ID.otype  = ORDER_SET; // Create a 'set' order
   Order_Info.infotype=INFO_DIRNC;
   Order_Info.value.ui8val=dirn;
   // Send the order to the BOB
   Pard_Err=as_send_order();
   if(Pard_Err!=PARDU_ERR_NONE){
      sprintf(errmsg,"%s",Pard_err_str());
      cmdstatus=CMD_ERR; goto end_setdirn;
     } 
   // If it worked we also need to set the equivalent local struct
   // members to keep things in sync.
   act_sync_to_local(ACT_STEPPER,id,1);
  }

 end_setdirn:
 return cmdstatus;
}


int sv_wait_for_pmr(double timeout_musecs, char *errmsg)
// Waits for a PMR (for use with server-initiated motion commands, as opposed to
// those that come over from a client).
// Returns SV_PMR_FOUND if a PMR was detected
// Returns SV_PMR_TIMEOUT if no PMR was read after timeout_musecs microseconds
//                        or 5ms (whichever is the longer)
// Returns SV_PMR_ERROR if the as_check_move_status function returned an error
//                      in which caseerrmsg will contain whatever that function
//                      set it to.
{
 struct timespec start, now;
 uint8_t result, actype, actid, pmerr; // Local variables to receive PMR data
 long elapsed_sec, elapsed_nsec;
 double total_elapsed;

 // Apply a floor to prevent false timeouts on ultra-short moves
 if (timeout_musecs < 5000.0) {
     timeout_musecs = 5000.0; 
 }

 clock_gettime(CLOCK_MONOTONIC, &start);

 while (1) {
     // 1. Check for the message
     // Pass addresses of local variables to satisfy the function signature
     result = as_check_move_status(&actype, &actid, &pmerr, errmsg);
     
     if (result == 1) return SV_PMR_FOUND;
     if (result == 2 || result == 3) return SV_PMR_ERROR; // AUDIT (26.10.2026): 3 is a valid PMR with a fatal code


     // 2. Check elapsed time
     clock_gettime(CLOCK_MONOTONIC, &now);
     elapsed_sec  = now.tv_sec  - start.tv_sec;
     elapsed_nsec = now.tv_nsec - start.tv_nsec;
     
     // Total_elapsed in microseconds
     total_elapsed = (double)(elapsed_sec * 1000000.0) +
                     (double)(elapsed_nsec / 1000.0);

     if(total_elapsed >= timeout_musecs) break; 

     // 3. Yield to the OS (100 microseconds)
     musleep(100);
 }

 // Final "Last Chance" check after timeout
 return (as_check_move_status(&actype, &actid, &pmerr, errmsg) == 1) ?
                                                  SV_PMR_FOUND : SV_PMR_TIMEOUT;
}

uint8_t sv_moveto_to_moveby(uint8_t id, uint32_t *dirsteps, char *errmsg,
                          uint8_t *reqdirn, uint8_t *infotype, double position)
// This takes the motor's current position (recorded in atomic steps in the 
// stepcount member of its struct) and the user's desired destination position
// for the motor ('position'- supplied in atomic steps) and calculates the
// distance between the two and direction of travel required to get there.
// That distance is then converted from atomic steps to steps in the current
// microstep size for the motor (the integer step currency used by the Arduino
// that actually moves the motor) and put that value into dirsteps.
// If, at the end of this function, Order_ID.otype == ORDER_SET it means a
// direction change is required and the order form has been filled in for this.
// Returns 0 on success
// Returns 1 on error and prints an error message into errmsg
{
 // 1. Position is already Atomic (maxmsdenom), so no scaling needed
 double target_atomic = position; 

 // 2. Pure Atomic subtraction
 double delta_atomic = target_atomic - Stepperlist[id]->stepcount;

 // 3. Convert Atomic delta to current microsteps for Arduino
 double steps_to_move = fabs(delta_atomic) * ((double)Stepperlist[id]->msdenom / 
                                          (double)Stepperlist[id]->maxmsdenom);

 // 4. Round to the nearest whole step
 *dirsteps = (uint32_t)(0.5 + steps_to_move);
 if(*dirsteps==0) return 0; // Don't waste time with direction logic if there is
                            // nowhere to go.

 // This will change to ORDER_SET later if a direction change is needed
 Order_ID.otype  = ORDER_STEPPER_MOVE;

 // 5. Set the direction based on the atomic delta
 *reqdirn = DIRN_UNS;
 if(delta_atomic < -0.00001) { // Floating point safe "Less than zero"
    // Move towards Home (Decreasing stepcount)
    switch(Stepperlist[id]->homed_to) {
       case LIM_SWTC1: case LIM_NULL1: *reqdirn = Stepperlist[id]->dirn1; break;
       case LIM_SWTC2: case LIM_NULL2: *reqdirn = Stepperlist[id]->dirn2; break;
       default: break;
    }
 } else if(delta_atomic > 0.00001) { // Floating point safe "Greater than zero"
    // Move away from Home (Increasing stepcount)
    switch(Stepperlist[id]->homed_to) {
       case LIM_SWTC1: case LIM_NULL1: *reqdirn = Stepperlist[id]->dirn2; break;
       case LIM_SWTC2: case LIM_NULL2: *reqdirn = Stepperlist[id]->dirn1; break;
       default: break;
    }
 }

 // 6. Final direction safety check & Order setup
 if(*reqdirn == DIRN_UNS) {
   sprintf(errmsg,"Couldn't set direction for motor %"PRIu8". Is it homed?",id);
   return 1;
 }

 // 7. Now we know the dirsteps and the required direction. If the required
 // direction is different to the current direction, we must send an order to
 // the Arduino to change this. First I make sure that the direction was set
 // correctly, else we can't go on.
 if(*reqdirn != Stepperlist[id]->currdirn) {
   Order_ID.actid = id;
   Order_ID.otype = ORDER_SET;
   *infotype = INFO_DIRNC;
   Order_Info.infotype = *infotype;
   Order_Info.value.ui8val = *reqdirn;
  }

 return 0;
}

void sv_init_accparam(uint8_t actuator)
// Initialse an actuator's acceleration parameters struct
{
 sv_Accparam[actuator].set=0;
 sv_Accparam[actuator].delay_max_min=0;
 sv_Accparam[actuator].delay_min=0;
 sv_Accparam[actuator].duration=0;
 sv_Accparam[actuator].p_acc=0.0;
 sv_Accparam[actuator].s_acc=0;
 return;
}

int sv_pard_server(int port)
// The main pard server loop
// Script command arguments are not all checked thoroughly for range and and
// other validity parameters because they would have been so checked at the
// client end. However, some checks are made in case client and server are
// different versions to each other (and so incompatible argument lists may be
// encountered in that case). 
{
 char msgtxt[TRIADMAX],cmdtriad[TRIADMAX],errmsg[ERRMSG_LEN];
 void *context=NULL,*responder=NULL;
 int rc,timeo,climdiff,have_blob,sending_blob,sending_bloberr,logerr,cmdstatus;
 size_t timeo_size = sizeof(timeo),lenerr,lenblob;
 time_t ts;
 char timedatestr[32];
 double doutmp;
 uint32_t ui32,ardutensteps1,ardutensteps2;
 int32_t  si32;
 uint8_t  ui81,ui82,ui83,gflag[5],reqdirn,from_moveto,id_a;
 int32_t ImHeight,ImWidth; // Local variables in this program but I keep the 
                           // same capitalised names as in pardclient & pardcap
 int local_frame_number=0;
 char nomstr[TXT_BUFMAX];
 uint8_t *ui8blob;

 uint32_t dirsteps,dirsteps_x,dirsteps_y,dirsteps_z;
 double userval1,userval2,userval3;
 double position,position_x,position_y,position_z=0;
 uint8_t stage_id,ptoggopt;
 uint32_t pvht,pvwd;
 uint8_t af_failure_choice;
 int is_curve_server = 1;
 int tmpverb;
 int loginstatus=2,useridx=0;

 

 start_again:
 
 // Temporarily ensure verbosity is high so any messages during startup are
 // emitted:
 tmpverb=Verbosity;
 Verbosity = 2;

 // Construct the listenting address
 sprintf(cmdtriad,"tcp://*:%d",port);


 // Socket to talk to clients
 context = zmq_ctx_new (); 
 if (context == NULL) {
   show_message("ERROR: Failed to establish a ZMQ context core.\n", "",
                                                                  MT_SESS_E, 1);
   return -1;
 }

 responder = zmq_socket (context, ZMQ_REP); 
 if (responder == NULL) {
   show_message("ERROR: Failed to establish responder socket widget.\n", "",
                                                                  MT_SESS_E, 1);
   zmq_ctx_destroy(context);
   return -1;
 }


 // 1. MANDATORY: Toggle this socket into explicit Curve Security Server Mode!
 rc = zmq_setsockopt(responder, ZMQ_CURVE_SERVER, &is_curve_server,
                                                       sizeof(is_curve_server));
 if(rc) {
   show_message("CRITICAL ERROR: Failed to configure socket into Curve Server"
                                        " Mode.\n", "SECURITY: ", MT_SESS_E, 1);
   goto end_server;
 }

 // 2. Inject the server's validated private (secret) key into the wire context
 rc = zmq_setsockopt(responder, ZMQ_CURVE_SECRETKEY, server_sec, 40);
 if(rc) {
   show_message("CRITICAL ERROR: Failed to bind secret encryption token to"
                               " server socket.\n", "SECURITY: ", MT_SESS_E, 1);
   goto end_server;
 }

 // 3. Inject the server's matching public key into the wire context
 rc = zmq_setsockopt(responder, ZMQ_CURVE_PUBLICKEY, server_pub, 40);
 if(rc) {
   show_message("CRITICAL ERROR: Failed to bind public encryption token to"
                               " server socket.\n", "SECURITY: ", MT_SESS_E, 1);
   goto end_server;
 } 


 rc = zmq_bind (responder, cmdtriad);

 if(rc){
   show_message("%s: Failed to bind socket to tcp port. Cannot run server.\n",
                                                         "ERROR: ",MT_SESS_E,1);
   goto end_server;
  }
       
 // Set a timeout on both the send and receive arms socket so we don't
 // 'hang' forever if the network goes down.
 timeo=100; // Set timeout to 100 ms.
 rc=zmq_setsockopt(responder,ZMQ_SNDTIMEO,&timeo,timeo_size);
 if(rc)
   show_message("Failed to set a network timeout - you may have to forcibly"
   " shut down the server if it 'hangs' too long due to a network outage.\n",
                                                         "ERROR: ",MT_SESS_E,0);
 rc=zmq_setsockopt(responder,ZMQ_RCVTIMEO,&timeo,timeo_size);
 if(rc) 
   show_message("Failed to set a network timeout - you may have to forcibly"
   " shut down the server if it 'hangs' too long due to a network outage.\n",
                                                         "ERROR: ",MT_SESS_E,0);
 
//// Do some inititialisation ////

 // Reset the memory for server sv_Triad and the argument pointer 
 if(sv_triad_resize(2)){
  show_message("Could not allocate RAM for the server triad.\n","ERROR: ",
                                                                   MT_SESS_E,0);
  goto end_server;
 }
 if(sv_triadarg_resize(2)){
  show_message("Could not allocate RAM for the server triad argument pointer.\n"
                                                        ,"ERROR: ",MT_SESS_E,0);
  goto end_server;
 }

 // Reset memory for sv_Blob
 if(sv_blob_resize(2)){
  show_message("Could not allocate RAM for the sv_Blob.\n","ERROR: ",
                                                                   MT_SESS_E,0);
  goto end_server;
 }
 
 // Set null user name
 sprintf(sv_Username,"Caller");
 sprintf(sv_Client_id,"%s","");

 // On first use, sv_Mdx must remain 0
 sv_Dont_update_Mdx=1;
 sv_Mdx=0;    // Receptive to all. Message index = 0.
 sv_Climdx=0; // Client respnse message index;
 
 // Start off with no-one logged in
 sv_Client_logged_in=0; 

 // No blobs (or anything) sent yet
 sv_Previous_blob=0;
 sending_blob=0;
 sending_bloberr=0;

 // Start off with a clean slate as far as comms errors are concerned
 sv_Merrdx=0;
 sv_Merrmax=10;
 sv_Resendidx=0;
 sv_Resendmax=10;

 // Autofocus
 af_failure_choice = AFFA_IGNORE;

 // The PMR null flag for when a motion request results in no motion - we set
 // this to tell the act_movereport to report back success without checking the
 // serial port because no PMR will ever appear there (so it will go on checking
 // forever creating a 'hang' at the server):
 sv_PMR_null = 0;
 sv_PMR_null_actype = ACT_STEPPER;
 sv_PMR_null_actid  = ACT_ID_STAGE_XYZ;

 ImHeight=0;
 ImWidth=0;
 
 Pard_Err_Checksum=0;

//// Initialisations complete ////

 
//// Start the Server ////
 sprintf(msgtxt,"Starting %s Server: Done. Use Ctrl+c to stop.\n",sv_Pardname);
 show_message(msgtxt,"FYI: ",MT_SESS_T,0);

 // Revert back to the user's choice of verbosity:
 Verbosity=tmpverb;

// Note: It would be nice to recover from send/receive errors but for now I just
// goto reboot_server in an attempt to at least keep the server running for
// other users (or the same user to re-log in).
// This is due to the lock-step limitation of the REQ-REP ZMQ pattern I am
// using. This could do with and upgrade to a more flexible pattern.        

 while(sv_Server_run){

   // As server, the first step is to listen for a call from clients ...
   if(sv_receive_message(responder,&have_blob)) goto reboot_server;
   
   // We'll change this if required.
   sending_blob=0;     // sv_Blob will contain an image or other non-error data
   sending_bloberr=0;  // sv_Blob will contain the errmsg string

   // A call from a potential or logged-in client (and addressed to our Pardus) 
   // has been received and is stored in sv_Msghdr[3],sv_Triad +/- sv_Blob.   
   if(sv_Client_logged_in==0){ 
      // No one is logged in - it means we are waiting for a call from a 'first-
      // come' user - i.e. a login request. If we got here the first part of the
      // message (sv_Msghdr[0]) was the name of this server instance (e.g.
      // "Pardus1") and this was checked in recv_multi(). The next thing to
      // check is that there are no more than 4 parts to the message (a valid
      // login request has exactly 4 parts) so first test for this:
      if((have_blob) || strcmp(sv_Msghdr[1],"login")){ 
        // This indicates there are more than 4 parts or the term 'login' is not
        // included in the right place. Update the reply message to an error -
        // sv_Mdx will show as -1 but actual sv_Mdx will not be affected:
        sv_update_message_header(sv_MES_ANON_ERR);
        if(sv_triad_set("ERROR NoLogin Bye")) goto reboot_server;
        if(sv_send_message(responder,0)) goto reboot_server;
        continue;
      }

      // OK, so this is a login request. The user name is supplied in
      // sv_Msghdr[2] and the password in sv_Triad. In practice this should be
      // sent encrypted but for basic demo we do not use encryption. Now check
      // if the supplied login credentials match a known user:
      loginstatus = sv_match_credentials(sv_Msghdr[2],sv_Triad,&useridx);
      switch(loginstatus){ 
          case 0: // Full match - user is authenticated
          case 5: // Full match - user is authenticated - but can only do a
                  // password change action
             // Now we have the name of our user we use it to generate the 
             // session client ID
             sv_client_id_gen(sv_Msghdr[2],sv_Client_id);
             if(sv_triad_set(sv_Client_id))goto reboot_server;
             // Set the new user name
             sprintf(sv_Username,"%s",sv_Msghdr[2]);
             // Send a response giving the good news and providing the client ID
             sv_update_message_header(sv_MES_NONE);
             if(loginstatus==5){
               if(sv_triad_set("OK ChangePWD Ready")) goto reboot_server;
              } else {
               if(sv_triad_set("OK Accepted Ready")) goto reboot_server;
              }
             if(sv_send_message(responder,0)) goto reboot_server;
             sprintf(timedatestr,"%s",((time(&ts)) == -1)?
                                         "[Time not available]\n" : ctime(&ts));
             timedatestr[strlen(timedatestr)-1]='\0'; // Remove terminal '\n'
             sprintf(msgtxt,"%s: User %s logged in at %s.\n",
                                           sv_Pardname,sv_Username,timedatestr);
             show_message(msgtxt,"FYI: ",MT_SESS_T,0);
             sv_Client_logged_in=1;

             // Initialise log file for this user session

             sprintf(Log_SessionFilename, "Log_%s.txt", sv_Client_id);

             open_session_logfile();
             if(FP_Sessionlog!=NULL) {
                fprintf(FP_Sessionlog, "Log for Client Session %s\n",
                                                                  sv_Msghdr[1]);
                fprintf(FP_Sessionlog, "Logged in at: %s\n\n",timedatestr);
                fflush(FP_Sessionlog);
             } else {
               show_message("Failed to initialise log file for this session.\n",
                                                         "ERROR: ",MT_SESS_E,0);
             }


             // Reset the sv_Unitype choices for this login session
             sv_Unitype[ACT_STEPPER]=0;
             sv_Unitype[ACT_BINARY]=0;
             sv_Unitype[ACT_MLEVEL]=0;
             sv_Unitype[ACT_SERVO]=0;
             sv_Unitype[ACT_SENSOR]=0;

             // Reset the acceleration parameter sets
             sv_init_accparam(ACT_STEPPER);
             sv_init_accparam(ACT_BINARY);
             sv_init_accparam(ACT_MLEVEL);
             sv_init_accparam(ACT_SERVO);
             sv_init_accparam(ACT_SENSOR);

             // Reset Autofocus parameters for this login
             af_vars_init();

          break;
          case 1: // The user is on the list but the password supplied does not
                  // match the password on record for this user.
             // Send a multipart response giving the bad news.
             sv_update_message_header(sv_MES_ANON_ERR); 
             if(sv_triad_set("ERROR WrongPassword Ready"))goto reboot_server;
             if(sv_send_message(responder,0)) goto reboot_server;
          break;
          case 2: // The user is not on the list.
             // Send a multipart response giving the bad news.
             sv_update_message_header(sv_MES_ANON_ERR); 
             if(sv_triad_set("ERROR UserUnknown Ready"))goto reboot_server;
             if(sv_send_message(responder,0)) goto reboot_server;
          break;
          case 3: // The user is on the list but made 3 wrong password attempts.
             // Send a multipart response giving the bad news.
             sv_update_message_header(sv_MES_ANON_ERR); 
             if(sv_triad_set("ERROR StrikeOut Ready"))goto reboot_server;
             if(sv_send_message(responder,0)) goto reboot_server;
          break;
          case 4: // Password expired
             // Send a multipart response giving the bad news.
             sv_update_message_header(sv_MES_ANON_ERR); 
             if(sv_triad_set("ERROR ExpiredPwd Ready"))goto reboot_server;
             if(sv_send_message(responder,0)) goto reboot_server;
          break;
          default: // This shouldn't happen unless the program is edited wrongly
            show_message("Unknown response from sv_match_credentials.\n",
                                                 "Program Error: ",MT_SESS_E,1); 
          continue;
      }
    } else { // A user is logged and sent a properly formatted message:
    
      rc=atoi(sv_Msghdr[2]); // Get the client message index
      sprintf(msgtxt,"%s[%d]: User %s[%d] wants me to: %s.\n",
                                    sv_Pardname,sv_Mdx,sv_Username,rc,sv_Triad);
      show_message(msgtxt,"FYI: ",MT_SESS_T,0);
      // Force logout if the index is wrong:
      climdiff=(rc-sv_Climdx);
      if(climdiff==1) sv_Climdx=rc; else goto logout;
      // Read the command in sv_Triad and take action accordingly:
      cmdtriad[0]='\0';  // 'Clear' any old command sv_Triad data.
      cmdstatus= CMD_OK; // We'll change this if things go wrong.

      // If the user is on a one-time password, only offer the change pass
      if(loginstatus==5){
        if(!strcmp(sv_Triad,"logout") || sv_UserDb[useridx].single_use == 2){
          // User want's to quit (or session ending due communication error or
          // or OTP abuse):
          goto logout;    
         } else if(!strcmp(sv_triad_argv(1),"pwd_change")){
           // pwd_change <current_pwd> <new_pwd>
           if(sv_change_password(sv_triad_argv(2), sv_triad_argv(3), useridx, errmsg)){
               sprintf(cmdtriad,"ERROR FailedCmd Ready");
               sending_bloberr=1;
           } else {
               sprintf(cmdtriad,"OK Done Ready");
               loginstatus=0; // Clear the probationary local variable flag
           }
         } else goto unknown_command;
        goto send_server_response;
       }

      // Othwise start the full server loop
      if(!strcmp(sv_Triad,"logout")){
        // User want's to quit (or session ending due communication error)
        logout:
          if(climdiff!=1){
            if(sv_triad_set("ERROR WrongIndex Bye")) goto reboot_server;
            logerr=1;
           } else if(sv_UserDb[useridx].single_use == 2){
            if(sv_triad_set("ERROR SecurityViolation Bye")) goto reboot_server;
            logerr=1;
           } else {
            if(sv_triad_set("OK Done Bye")) goto reboot_server;
            logerr=0;
           }
        sv_log_off_client(responder,sv_Triad,logerr);
        continue;     
       } else if(!strcmp(sv_triad_argv(1),"pwd_change")){
           // pwd_change <current_pwd> <new_pwd>
           if(sv_change_password(sv_triad_argv(2), sv_triad_argv(3), useridx, errmsg)){
               sprintf(cmdtriad,"ERROR FailedCmd Ready");
               sending_bloberr=1;
           } else {
               sprintf(cmdtriad,"OK Done Ready");
               loginstatus=0; // Clear the probationary local variable flag
           }
         } else if(!strcmp(sv_triad_argv(1),"af_setup")){  
        sscanf(sv_triad_argv(2),"%la",&userval1); // Current best focus Z posn.
        sscanf(sv_triad_argv(3),"%la",&userval2); // Initial movement distance.
        sscanf(sv_triad_argv(4),"%"SCNu32,&AF_max_retries);
        sscanf(sv_triad_argv(5),"%"SCNu8,&AF_dirn_up); // What way is up.
        sscanf(sv_triad_argv(6),"%"SCNu8,&AF_colchan); // Colour channel to use.
        sscanf(sv_triad_argv(7),"%"SCNu8,&af_failure_choice); // Failure action.

        // First check if the Z Stepper actuator exists. This is needed up front
        // because we use the stepper's calibration settings to calculate
        // distances and positions in tersm of number of (micro)steps):
        if(!actuator_exists(ACT_STEPPER,ACT_ID_STAGE_Z)){
           sprintf(errmsg,"Z stepper does not exist");
           cmdstatus=CMD_ERR; goto process_af_setup;
          }

        // All distances and coordinates in a script are provided in either
        // atomic units or calibrated real world units. However in this program
        // they must be atomic units - so convert as required:
        AF_centre=sv_posdist_to_maxmsdenomsteps(ACT_ID_STAGE_Z, userval1);
        AF_issz=sv_posdist_to_maxmsdenomsteps(ACT_ID_STAGE_Z, userval2);

        // Special case for initial step size = 0 so we don't waste time sending
        // trying to do a Z search where there can be no search motion:
        if(AF_issz<1.0){
           sprintf(errmsg,"Z search initial search delta is 0 steps");
           cmdstatus=CMD_ERR; 
           goto process_af_setup;
         }

        process_af_setup:
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else {
          sprintf(cmdtriad,"OK Done Ready");
         }

       } else if(!strcmp(sv_triad_argv(1),"af_ccrit")){  
        sscanf(sv_triad_argv(2),"%"SCNu32,&AF_max_itn); // Max. iterations.
        sscanf(sv_triad_argv(3),"%la",&userval1);    // Max. distance diff.
        sscanf(sv_triad_argv(4),"%la",&AF_min_peak); // Min true peak focus val.
        sscanf(sv_triad_argv(5),"%ld",&AF_timeout);  // Timeout (ms).

        // First check if the Z Stepper actuator exists. This is needed up front
        // because we use the stepper's calibration settings to calclate
        // distances and positions in tersm of number of steps):
        if(!actuator_exists(ACT_STEPPER,ACT_ID_STAGE_Z)){
           sprintf(errmsg,"Z stepper does not exist");
           cmdstatus=CMD_ERR; goto process_af_ccrit;
          }

        // Set the maximum acceptable distance a newly found best focus Z can be
        // from the previous best focus Z as number of atomic microsteps (if
        // this comes to <1.0 then this criterion will not be used).
        // All distances and coordinates in a script are provided in either
        // atomic units or calibrated real world units. However in this program
        // they must be atomic units - so convert as required:
        AF_maxdiff=sv_posdist_to_maxmsdenomsteps(ACT_ID_STAGE_Z, userval1);

        process_af_ccrit:
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");


       } else if(!strcmp(sv_triad_argv(1),"autofocus")){
        // autofocus
        int af_says;
        char aferr[128];
        uint32_t bu_af_max_itn,idx;
        double bu_af_max_diff;

        // First check if the actuator exists:
        if(!actuator_exists(ACT_STEPPER,ACT_ID_STAGE_Z)){
           sprintf(errmsg,"Stage Z motor does not exist (is not defined)");
           cmdstatus=CMD_ERR; goto process_af_triad;
          }

        // Now check we have an acceleration profile set for that actuator type
        if(sv_Accparam[ACT_STEPPER].set==0){
           sprintf(errmsg,"Stepper accel. profile not set (use act_accel)");
           cmdstatus=CMD_ERR; goto process_af_triad;
          }

        // Store some values that may get temporarily changed
        bu_af_max_diff=AF_maxdiff;
        bu_af_max_itn=AF_max_itn;
        // Attempt Autofocus
        af_says = af_auto_focus(aferr);
        switch(af_says){
             case AF_SUCCESS: 
               // Nothing more to do
             break;
             case AF_IMPOSSIBLE: 
               // User will get an interactive message - so let them decide 
               // whether to ignore or continue
               sprintf(errmsg,"AF 'impossible': %s",aferr);
               cmdstatus=CMD_ERR;
             break;
             case AF_CCRITTMO: 
               sprintf(errmsg,"AF timed out");
               cmdstatus=CMD_ERR;
               break;
             case AF_CCRITITN:  
               // Have a few more goes if that's what the user requested after
               // increasing af_max_itn 
             case AF_CCRITMDF:  
               // Have a few more goes if that's what the user requested after
               // increasing af_max_difference
             case AF_FALSEPEAK: 
               // Have a few more goes if that's what the user requested because
               // success is possible (failure might have been due to random
               // noise in the image or illumination or sample (if it is live or
               // otherwise changeable).
               cmdstatus=CMD_ERR; // Will change to OK if we succeed on re-tries
               for(idx=0;idx<AF_max_retries;idx++){
                   // Change values that might be hindering success 
                   // (increase each by 50%)
                   if(af_says==AF_CCRITITN)
                              AF_max_itn=(uint32_t)(0.5+1.5*(double)AF_max_itn);
                   if(af_says==AF_CCRITMDF)
                              AF_maxdiff*=1.5;
                   // Try AF again
                   af_says = af_auto_focus(aferr);
                   switch(af_says){
                     case AF_SUCCESS:  // Success at last!
                       // Restore values that might have been changed
                       AF_maxdiff=bu_af_max_diff;
                       AF_max_itn=bu_af_max_itn;
                       // Reset the comand success flag
                       cmdstatus= CMD_OK;
                       // Get right out of this nested place
                       goto process_af_triad;
                     break;
                     case AF_IMPOSSIBLE:
                     case AF_CCRITERR:
                     case AF_MOVEERR:
                     goto exit_af_retry_loop; // No point re-trying
                     default: break; // Otherwise continue with re-tries
                    }
                  } // end of retry for loop block
               exit_af_retry_loop:
               // If we have reached here then we have failed to get autofocus
               // despite multiple efforts so abide by the user's choice of
               // failure action.
               // First restore original CCRIT values in case these were changed
               AF_maxdiff=bu_af_max_diff;
               AF_max_itn=bu_af_max_itn;
               // Now get reason for failure to report back
               switch(af_says){ // 'Success' should never happen here but ...
                 case AF_SUCCESS:    sprintf(aferr,"Success bug!");       break; 
                 case AF_IMPOSSIBLE: break; // aferr will be set by auto_focus()
                 case AF_CCRITERR:   sprintf(aferr,"CCRIT error");        break;
                 case AF_CCRITITN:   sprintf(aferr,"Max Itns reached");   break;
                 case AF_CCRITTMO:   sprintf(aferr,"Timeout reached");    break;
                 case AF_CCRITMDF:   sprintf(aferr,"Max Diff exceeded"
                                                  " (%.9g)",AF_lk_maxdiff); break;
                 case AF_FALSEPEAK:  sprintf(aferr,"false peak (%.9g)",
                                                     AF_lk_peak);         break;
                 case AF_MOVEERR:    break; // aferr will be set by auto_focus()
                 default:            sprintf(aferr,"some reason");        break;
                }
               switch(af_failure_choice){
                  // For remote single commands 'exit' and 'ignore' have the
                  // same effect. The Z-stage will have defaulted to the last
                  // good Z position (or the default set in af_setup)
                  case AFFA_EXIT:  
                  case AFFA_IGNORE:
                   sprintf(errmsg,"Autofocus failed due to '%s'."
                                       " Set to last best Z or default.",aferr);
                  break;
                  default:// This should not happen unless there is a bug
                   sprintf(errmsg,"Autofocus FAILED due to a Program Error"
                    " (undef af_failure_choice = %"PRIu8").",af_failure_choice);
                  break;
                 }
            break;
            case AF_MOVEERR:
                   sprintf(errmsg,"AF 'movement error': %s",aferr);
                   cmdstatus=CMD_ERR;
            break;
            case AF_CCRITERR:
              // This error should never occur - it is tantamount to a bug.
            default: // This should not happen unless there is a bug.
                   sprintf(errmsg,"Autofocus FAILED due to a Program Error"
                                        " (undef af_auto_focus return value).");
                   cmdstatus=CMD_ERR;
            break;
           }  
        process_af_triad:                
        if(cmdstatus==CMD_ERR){
            sprintf(cmdtriad,"ERROR FailedCmd Ready"); 
            sending_bloberr=1;
           } else {
            sprintf(cmdtriad,"OK Done Ready");
            // Send the latest Z coordinate (stepcount) because it has likley
            // changed as a result of autofocus:
            if(sv_blob_resize(SCTBYTES)){
              sprintf(errmsg,"sv_blob resize failed for PMRep3 retrieval");
              cmdstatus=CMD_ERR; goto process_af_triad;
             }
            memcpy(sv_Blob,&PMRep3,SCTBYTES);
            sending_blob=1;
        }   
            
       } else if(!strcmp(sv_triad_argv(1),"def_actuator")){  
        sscanf(sv_triad_argv(2),"%"SCNu8,&ui81);  // Actuator type
        sscanf(sv_triad_argv(3),"%"SCNu8,&ui82);  // Actuator index
        sscanf(sv_triad_argv(4),"%s",nomstr);     // Actuator name
        // Fill in the request forms (only need an Order_ID for for this): 
        Order_ID.otype  = ORDER_CREATE;
        Order_ID.actype = ui81;
        Order_ID.actid  = ui82;
        // Send the order to the Parduino and assess the report
        Pard_Err=as_send_order();
        if(Pard_Err!=PARDU_ERR_NONE){
           sprintf(errmsg,"%s",Pard_err_str());
           sprintf(cmdtriad,"ERROR FailedCmd Ready");
           sending_bloberr=1;
          } else {
           // All has gone well on the PARDuino so let us register the actuator
           // here on the server:
           switch(Order_ID.actype){
            case ACT_STEPPER:
             Pard_Err=Act_Stepper_create(Order_ID.actid, nomstr);
            break;
            case ACT_BINARY:
             Pard_Err=Act_Binary_create(Order_ID.actid, nomstr);
            break;
            case ACT_MLEVEL:
             Pard_Err=Act_Mlevel_create(Order_ID.actid, nomstr);
            break;
            case ACT_SERVO:
             Pard_Err=Act_Servo_create(Order_ID.actid, nomstr);
            break;
            case ACT_SENSOR:
             Pard_Err=Act_Sensor_create(Order_ID.actid, nomstr);
            break;
           }          
           if(Pard_Err==PARDU_ERR_NONE) sprintf(cmdtriad,"OK Done Ready");
           else {
                 sprintf(errmsg,"%s",Pard_err_str());
                 sprintf(cmdtriad,"ERROR FailedCmd Ready");
                 sending_bloberr=1;
                }
          }

       } else if(!strcmp(sv_triad_argv(1),"undef_actuator")){  
        sscanf(sv_triad_argv(2),"%"SCNu8,&ui81);  // Actuator type
        sscanf(sv_triad_argv(3),"%"SCNu8,&ui82); // Actuator index
        // Fill in the request forms (only need an Order_ID for for this): 
        Order_ID.otype  = ORDER_DESTROY;
        Order_ID.actype = ui81;
        Order_ID.actid  = ui82;

        // Send the order to the Parduino and assess the report
        Pard_Err=as_send_order();
        if(Pard_Err!=PARDU_ERR_NONE){
           sprintf(errmsg,"%s",Pard_err_str());
           sprintf(cmdtriad,"ERROR FailedCmd Ready");
           sending_bloberr=1;
          } else {
           // All has gone well on the PARDuino so let us de-register the
           // actuator here on the server:
           switch(Order_ID.actype){
            case ACT_STEPPER:
             Pard_Err=Act_Stepper_destroy(Order_ID.actid);
            break;
            case ACT_BINARY: 
             Pard_Err=Act_Binary_destroy(Order_ID.actid);
            break;
            case ACT_MLEVEL: 
             Pard_Err=Act_Mlevel_destroy(Order_ID.actid);
            break;
            case ACT_SERVO:
             Pard_Err=Act_Servo_destroy(Order_ID.actid);
            break;
            case ACT_SENSOR:
             Pard_Err=Act_Sensor_destroy(Order_ID.actid);
            break;
           }          
           if(Pard_Err==PARDU_ERR_NONE) sprintf(cmdtriad,"OK Done Ready");
           else {
                 sprintf(errmsg,"%s",Pard_err_str());
                 sprintf(cmdtriad,"ERROR FailedCmd Ready");
                 sending_bloberr=1;
                }
          }

       } else if(!strcmp(sv_triad_argv(1),"act_get")){  
        sscanf(sv_triad_argv(2),"%"SCNu8,&ui81); // Actuator type
        sscanf(sv_triad_argv(3),"%"SCNu8,&ui82); // Actuator index
        sscanf(sv_triad_argv(4),"%"SCNu8,&ui83); // Info type

        // First check if the actuator exists:
        if(!actuator_exists(ui81,ui82)){
           sprintf(errmsg,"Actuator does not exist");
           cmdstatus=CMD_ERR; goto process_act_get;
          }

        // Special case check on sensor actuators - can't read until mature
        if(ui81 == ACT_SENSOR && ui83 == INFO_SEN_VAL){
          if(Sensorlist[ui82]->maturity < ADULT_SENSOR){
            snprintf(errmsg,ERRMSG_LEN,"You can't read the value of this sensor"
            " because it is immature.");
            cmdstatus=CMD_ERR; goto process_act_get;
           }
          }

        // Special case check for steppers to ensure the get request is allowed
        // according to the type of stepper driver used
        if(ui81 == ACT_STEPPER){
         switch(ui83){
             // Restrict reading DRV-specific settings when TMC is the type and vice versa
             case INFO_PIN_MS1:
             case INFO_PIN_MS2:
             case INFO_PIN_MS3:
              if(Stepperlist[ui82]->driver_type == SD_TMC2209){
                snprintf(errmsg,ERRMSG_LEN,"You can't read A4988 or DRV8825 "
                   "driver settings when the stepper is of TMC driver type.");
                cmdstatus=CMD_ERR; goto process_act_get;
               }
             break;
             case INFO_PIN_DIAG:
             case INFO_TMCSGVAL:
             case INFO_TMCSTATUS:
             case INFO_TMCCHCONF:
             case INFO_SG_BUFFPC:
             case INFO_SG_THRESH:
             case INFO_TURBO_BST:
             case INFO_TMCMSCNT:
             case INFO_MMODE:
              if(Stepperlist[ui82]->driver_type != SD_TMC2209){
                snprintf(errmsg,ERRMSG_LEN,"You can't read TMC driver settings"
                   " when the stepper is not of a TMC driver type.");
                cmdstatus=CMD_ERR; goto process_act_get;
               }
             break;
             default:
             break;
            }
          }
        

        // Fill in the 'infotype' part of the Order_Info form:
        Order_Info.infotype=ui83;

        // Now test the info item to see if it is valid for the type of actuator
        // and whether we need to call the PARDuino or just get this locally.
        // Accordingly fill in the appropriate part(s) of an Order_Info form.

        switch(act_get_source(ui81,errmsg)){
          case AIS_LOCAL:
            // Special case - Translation table request - read the row argument:
            if(ui81 == ACT_MLEVEL){
              if(ui83 == INFO_ML_TTABLE){
                 sscanf(sv_triad_argv(5),"%"SCNu32,&(Order_Info.value.triuinval[0]));
                }
             }
            act_get_from_local(ui81,ui82);
          break;
          case AIS_BOB:
           // We need to get the data from the BOB so fill in the order forms.
           // We did the Order_Info form above so only need an ID form now:
           Order_ID.otype  = ORDER_GET;
           Order_ID.actype = ui81;
           Order_ID.actid  = ui82;
           // Send the order to the Parduino and assess the report
           Pard_Err=as_send_order();
           if(Pard_Err!=PARDU_ERR_NONE){
              sprintf(errmsg,"%s",Pard_err_str());
              cmdstatus=CMD_ERR; goto process_act_get;
             } 
           // If it worked we also need to set the equivalent local struct
           // members to keep things in sync. The reason we went to the BOB to
           // read those values in the first place is because they might have
           // changed from what we currently have in out local copy.
           act_sync_to_local(ui81,ui82,0);
          break;
          default:
           cmdstatus=CMD_ERR; goto process_act_get;
          break;
         }
        // If we get to here it means we have successfully got the required
        // information in the value field of the Order_Info form. So now we
        // must package it into a blob for communication back to the client.
 
        lenblob = act_client_blobsz_from_infotype(ui81);
        if(lenblob<2){ // This indicates an error
           sprintf(errmsg,"Blob size calculation error for act_get");
           cmdstatus=CMD_ERR; goto process_act_get;
         }
 
        // Set the blob to hold that many bytes
        if(sv_blob_resize(lenblob)){
          sprintf(errmsg,"sv_blob resize failed for act_get");
          cmdstatus=CMD_ERR; goto process_act_get;
         }

        // Put the info value(s) into the blob
        // First, the byte that specifies what's coming next:

         ui8blob=(uint8_t *)sv_Blob;
         ui8blob[0]=Order_Info.infotype;
         ui8blob++; // Points to the start of the first data value

        // Now copy the gotten value into the blob according to its type:
        switch(Order_Info.infotype){
         case INVT_DOU8:   // 8-byte double
           memcpy(ui8blob,&(Order_Info.value.douval),8);        
         break; 
         case INVT_UI8 :   // 1 byte unsigned int
          ui8blob[0]=Order_Info.value.ui8val;
         break; 
         case INVT_UI32:   // 4 byte unsigned int
          memcpy(ui8blob,&(Order_Info.value.ui32val),4);
         break; 
         case INVT_3UIN:   // 3 x 4 byte unsigned int
          memcpy(ui8blob,Order_Info.value.triuinval,12);
         break; 
         case INVT_3DOU:   // 24 bytes (3x double)
          memcpy(ui8blob,Order_Info.value.tridouval,24);
         break; 
         case INVT_STR :  // 32 bytes (char string)
          memcpy(ui8blob,Order_Info.value.nomstr,32);
         break; 
         default: // If we get here its a program error (bug)
          sprintf(errmsg,"Invalid Order_Info return data type.");
          cmdstatus=CMD_ERR; goto process_act_get;
         break; 
        }

        // Let the send algorithm know we are sending a blob
        sending_blob=1;

        process_act_get:
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");

       } else if(!strcmp(sv_triad_argv(1),"act_set")){  
        sscanf(sv_triad_argv(2),"%"SCNu8,&ui81); // Actuator type
        sscanf(sv_triad_argv(3),"%"SCNu8,&ui82); // Actuator index
        sscanf(sv_triad_argv(4),"%"SCNu8,&ui83); // Info type

        // Fill in the ID order form in case we need it
        Order_ID.otype  = ORDER_SET;
        Order_ID.actype = ui81;
        Order_ID.actid  = ui82;

        // There will be 1 or 3 more arguments to read (supplying the data to be
        // set) but we need to find out a couple of things first:

        // 1. Does the actuator exist (no point doing any more if not):
        if(!actuator_exists(ui81,ui82)){
           sprintf(errmsg,"Actuator does not exist");
           cmdstatus=CMD_ERR; goto process_act_set;
          }

        // 2. Should we read 1, 2 or 3 more arguments? What data types are they?
        //    Read the outstanding arguments and fill in the Order_Info order
        //    form at the same time.
        Order_Info.infotype=ui83;
        switch(act_set_userinput_datatype(ui81,ui83)){
         case INVT_DOU8:   // 8-byte double
          sscanf(sv_triad_argv(5),"%la",&userval1);
         break; 
         case INVT_UI8 :   // 1 byte unsigned int
          sscanf(sv_triad_argv(5),"%"SCNu8,&(Order_Info.value.ui8val));
         break; 
         case INVT_UI32:   // 4 byte unsigned int
          sscanf(sv_triad_argv(5),"%"SCNu32,&(Order_Info.value.ui32val));
         break; 
         case INVT_3UIN:   // 3 x 4 byte unsigned int
          sscanf(sv_triad_argv(5),"%"SCNu32,&(Order_Info.value.triuinval[0]));
          sscanf(sv_triad_argv(6),"%"SCNu32,&(Order_Info.value.triuinval[1]));
          sscanf(sv_triad_argv(7),"%"SCNu32,&(Order_Info.value.triuinval[2]));
         break; 
         case INVT_3DOU:   // 24 bytes (3x double)
          sscanf(sv_triad_argv(5),"%la",&userval1);
          sscanf(sv_triad_argv(6),"%la",&userval2);
          sscanf(sv_triad_argv(7),"%la",&userval3);
         break; 
         case INVT_STR :  // 32 bytes (char string)
          sscanf(sv_triad_argv(5),"%31s",Order_Info.value.nomstr);
         break;
         default: // If we get here its a program error (bug)
          sprintf(errmsg,"Invalid Order_Info set data type.");
          cmdstatus=CMD_ERR; goto process_act_set;
         break; 
        }

        // 3. Do sanity checks on the proposed values. For example, if the
        // client wants to set msdenom, this must be denied unless
        // maxmsdenom > 1 and then only if the requested msdenom is <=
        // maxmsdenom. Also msdenom can't be 0.
        // Also, take this opportunity to convert steps to calibrated units if
        // this is the requested unit choice (e.g. for setting software limits
        // or stepcount.
        switch(ui81){
         case ACT_STEPPER:
           if(Order_Info.infotype != INFO_SD_TYPE){
              if(Stepperlist[ui82]->driver_type == SD_UNDEF){
                snprintf(errmsg,ERRMSG_LEN,"You must set the type of stepper"
                " actuator before you can set anything else.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
            }
           switch(Order_Info.infotype){
             case INFO_SD_TYPE:
              if(Stepperlist[ui82]->driver_type != SD_UNDEF){
                snprintf(errmsg,ERRMSG_LEN,"You can't change the type of a"
                                     " stepper actuator once it has been set.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             switch(Order_Info.value.ui8val){
                case SD_A4988   :
                case SD_DRV8825 :
                case SD_TMC2209 :
                break;
                default:
                snprintf(errmsg,ERRMSG_LEN,"You attempted to set an unknown or"
                  " forbidden driver type (%"PRIu8").",Order_Info.value.ui8val);
                cmdstatus=CMD_ERR;
                goto process_act_set;
               }
             break;
             case INFO_MSDEMAX:
              // Exclude zero
              if(Order_Info.value.ui32val < 1){
                snprintf(errmsg,ERRMSG_LEN,"Can't set maxmsdenom to be zero.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              // Ensure value is a valid power-of-two
              if((Order_Info.value.ui32val & (Order_Info.value.ui32val - 1)) != 0){
                snprintf(errmsg,ERRMSG_LEN,"Can't set maxmsdenom to a non-power-of-two value.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              // Driver-specific absolute ceilings
              switch(Stepperlist[ui82]->driver_type){
                case SD_TMC2209:
                 if(Order_Info.value.ui32val > 256){
                   snprintf(errmsg,ERRMSG_LEN,"Can't set maxmsdenom to"
                   " > 256 for TMC2209 drivers.");
                   cmdstatus=CMD_ERR; goto process_act_set;
                  }
                break;
                case SD_DRV8825:
                 if(Order_Info.value.ui32val > 32){
                   snprintf(errmsg,ERRMSG_LEN,"Can't set maxmsdenom to"
                   " > 32 for DRV8825 drivers.");
                   cmdstatus=CMD_ERR; goto process_act_set;
                  }
                break;
                case SD_A4988:
                 if(Order_Info.value.ui32val > 16){
                   snprintf(errmsg,ERRMSG_LEN,"Can't set maxmsdenom to"
                   " > 16 for A4988 drivers.");
                   cmdstatus=CMD_ERR; goto process_act_set;
                  }
                break;
                default: break;
               }

              if(Order_Info.value.ui32val < Stepperlist[ui82]->msdenom || 
                 Order_Info.value.ui32val < Stepperlist[ui82]->fastmsd) {
                snprintf(errmsg,ERRMSG_LEN,"Can't set maxmsdenom to be less than current msdenom or fastmsd settings.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;
             case INFO_CALIMAX:
              if(userval1 < 1.0e-3){
                snprintf(errmsg,ERRMSG_LEN,"Can't set maxcalibri to  < 1.0e-3.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              // maxcalibri is the only struct member that is actually a double
              // so, now that userval1 has passed the test, put it in the douval
              // union container (without calibration arithmetic):
              Order_Info.value.douval = userval1;
             break;
             case INFO_MSDENOM:
             case INFO_FASTMSD:
              // Guard 1: Ensure microstepping is structurally supported
              if(Stepperlist[ui82]->maxmsdenom <= 1){
                snprintf(errmsg,ERRMSG_LEN,
                      "Can't set msdenom or fastmsd unless maxmsdenom is > 1.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              
              // Guard 2: Catch zero early
              if(Order_Info.value.ui32val == 0){
                snprintf(errmsg,ERRMSG_LEN,"Can't set msdenom or fastmsd to be zero.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }

              // Guard 3: Ensure value is a valid power-of-two
              if((Order_Info.value.ui32val & (Order_Info.value.ui32val - 1)) != 0){
                snprintf(errmsg,ERRMSG_LEN,"Can't set msdenom or fastmsd to a non-power-of-two value.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }

              // Guard 4: Driver-specific absolute ceilings
              switch(Stepperlist[ui82]->driver_type){
                case SD_TMC2209:
                 if(Order_Info.value.ui32val > 256){
                   snprintf(errmsg,ERRMSG_LEN,"Can't set msdenom or fastmsd to"
                   " > 256 for TMC2209 drivers.");
                   cmdstatus=CMD_ERR; goto process_act_set;
                  }
                break;
                case SD_DRV8825:
                 if(Order_Info.value.ui32val > 32){
                   snprintf(errmsg,ERRMSG_LEN,"Can't set msdenom or fastmsd to"
                   " > 32 for DRV8825 drivers.");
                   cmdstatus=CMD_ERR; goto process_act_set;
                  }
                break;
                case SD_A4988:
                 if(Order_Info.value.ui32val > 16){
                   snprintf(errmsg,ERRMSG_LEN,"Can't set msdenom or fastmsd to"
                   " > 16 for A4988 drivers.");
                   cmdstatus=CMD_ERR; goto process_act_set;
                  }
                break;
                default: break;
               }

              // Guard 5: Ensure alignment with maximum hardware threshold constraints
              if(Order_Info.value.ui32val > Stepperlist[ui82]->maxmsdenom){
                snprintf(errmsg,ERRMSG_LEN,
                            "Can't set msdenom or fastmsd to be > maxmsdenom.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;
             case INFO_SW_LIM1:
             case INFO_SW_LIM2:
              if(sv_Unitype[ui81])
                   Order_Info.value.ui32val =
                      (uint32_t)round(userval1 * Stepperlist[ui82]->maxcalibri);
              else Order_Info.value.ui32val = (uint32_t)round(userval1);
              // Set to 'unset' (i.e. do not use) if this comes to 0
              if(Order_Info.value.ui32val<1)Order_Info.value.ui32val=SW_LIM_OFF;
             break;
             case INFO_STEPCOUNT:
              if(sv_Unitype[ui81])
                   Order_Info.value.ui32val =
                      (uint32_t)round(userval1 * Stepperlist[ui82]->maxcalibri);
              else Order_Info.value.ui32val = (uint32_t)round(userval1);
             break;
             case INFO_XYZCOORDS: 
             if(sv_Unitype[ui81]){ 
               Order_Info.value.triuinval[0] = 
            (uint32_t)round(userval1 * Stepperlist[ACT_ID_STAGE_X]->maxcalibri);
               Order_Info.value.triuinval[1] =
            (uint32_t)round(userval2 * Stepperlist[ACT_ID_STAGE_Y]->maxcalibri);
               Order_Info.value.triuinval[2] = 
            (uint32_t)round(userval3 * Stepperlist[ACT_ID_STAGE_Z]->maxcalibri);
              } else {
               Order_Info.value.triuinval[0] = (uint32_t)round(userval1);
               Order_Info.value.triuinval[1] = (uint32_t)round(userval2);
               Order_Info.value.triuinval[2] = (uint32_t)round(userval3);
              }
             break;
             case INFO_DIRN1:
             case INFO_DIRN2:
              if(Order_Info.value.ui8val>1){
                snprintf(errmsg,ERRMSG_LEN,"Only 0 or 1 is permitted for this setting "
                "(you chose %"PRIu8").",Order_Info.value.ui8val);
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;
             // Restrict setting DRV-specific settings when TMC is the type and vice versa
             case INFO_PIN_MS1:
             case INFO_PIN_MS2:
             case INFO_PIN_MS3:
              if(Stepperlist[ui82]->driver_type == SD_TMC2209){
                snprintf(errmsg,ERRMSG_LEN,"You can't set A4988 or DRV8825 "
                   "driver settings when the stepper is of TMC driver type.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;
             case INFO_PIN_DIAG:
             case INFO_TMCSGVAL:
             case INFO_TMCSTATUS:
             case INFO_TMCCHCONF:
             case INFO_SG_BUFFPC:
             case INFO_SG_THRESH:
             case INFO_TURBO_BST:
             case INFO_TMCMSCNT:
             case INFO_MMODE:
              if(Stepperlist[ui82]->driver_type != SD_TMC2209){
                snprintf(errmsg,ERRMSG_LEN,"You can't set TMC driver settings"
                   " when the stepper is not of a TMC driver type.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;
             default:
             break;
            }
         break;

         case ACT_BINARY:
           switch(Order_Info.infotype){
             case INFO_ON_VAL:
             case INFO_OFF_VAL:
              if(Order_Info.value.ui8val>1){
                snprintf(errmsg,ERRMSG_LEN,"Only 0 or 1 is permitted for this setting "
                "(you chose %"PRIu8").",Order_Info.value.ui8val);
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;
             case INFO_BIN_STATE:
              // User can't set the state on the Arduino until the actuator is
              // mature:
              if(Binarylist[ui82]->maturity < ADULT_BINARY){
                snprintf(errmsg,ERRMSG_LEN,"You can't set the state of this binary actuator"
                " because it is immature.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;
             default:
             break;
            }
         break;

         case ACT_MLEVEL:
           switch(Order_Info.infotype){
             case INFO_ML_TYPE:
              if(Mlevellist[ui82]->ml_type != MLT_UNSET){
                snprintf(errmsg,ERRMSG_LEN,"You can't change the type of an"
                                     " M-Level actuator once it has been set.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              switch(Order_Info.value.ui8val){
                case MLT_PARALLEL:
                case MLT_PWM:
                break;
                default: // (label added 14.11.26: without it the error branch
                         //  below was unreachable and any value was accepted)
                snprintf(errmsg,ERRMSG_LEN,"You can't change the type of an"
                 " M-Level actuator to anything other than MLT_PARALLEL or MLT_PWM.");
                cmdstatus=CMD_ERR;
                goto process_act_set;
               }
             break;
             case INFO_ML_PSC:
              if(Mlevellist[ui82]->ml_type == MLT_UNSET){
                snprintf(errmsg,ERRMSG_LEN,"You must set the type of an M-Level"
                " actuator before you set anything else.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              if(Mlevellist[ui82]->ml_pin[0] != PIN_IS_UNSET){
                snprintf(errmsg,ERRMSG_LEN,"You can only set the prescaler for"
                " pin 0 if it is unset.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              switch(Order_Info.value.ui8val){
                case PWM_PSC_1:     // Translates to:   1
                case PWM_PSC_8:     // Translates to:   8
                case PWM_PSC_32:    // Translates to:  32
                case PWM_PSC_64:    // Translates to:  64
                case PWM_PSC_128:   // Translates to: 128
                case PWM_PSC_256:   // Translates to: 256
                case PWM_PSC_1024:  // Translates to:1024
                break;
                default:
                snprintf(errmsg,ERRMSG_LEN,"You are attempting to set a pre-scaler "
                "to a forbidden value (you chose %"PRIu8").",Order_Info.value.ui8val);
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;
             case INFO_PIN_P0:
              if(Mlevellist[ui82]->ml_type == MLT_UNSET){
                snprintf(errmsg,ERRMSG_LEN,"You must set the type of an M-Level actuator"
                " before you set anything else.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;
             case INFO_PIN_P1:
             case INFO_PIN_P2:
             case INFO_PIN_P3:
             case INFO_PIN_P4:
             case INFO_PIN_P5:
             case INFO_PIN_P6:
             case INFO_PIN_P7:
              if(Mlevellist[ui82]->ml_type == MLT_UNSET){
                snprintf(errmsg,ERRMSG_LEN,"You must set the type of an M-Level actuator"
                " before you set anything else.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              if(Mlevellist[ui82]->ml_type == MLT_PWM){
                snprintf(errmsg,ERRMSG_LEN,"It is forbidden to set any pin other than P0 "
                "for a PWM M-Level actuator");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;
             case INFO_ML_LEVEL:
              if(Mlevellist[ui82]->ml_type == MLT_UNSET){
                snprintf(errmsg,ERRMSG_LEN,"You must set the type of an M-Level actuator"
                " before you set anything else.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              if(Mlevellist[ui82]->maturity <
                                       adult_mlevel(Mlevellist[ui82]->ml_type)){
                snprintf(errmsg,ERRMSG_LEN,"You can't set the level of this M-Level "
                "actuator because it is immature.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              if(Mlevellist[ui82]->pingaps){
                snprintf(errmsg,ERRMSG_LEN,"You can't set the level of this M-Level actuator"
                " because there are gaps in its pin definition sequence.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              // If we get here then there are no errors so we will interpret
              // the supplied mlevel value. However since this will change the
              // value sent by the user we should make a copy of it now so we
              // can sync it to the local shadow register if the command
              // succeeds:
              LK_mlevel = Order_Info.value.ui32val;

              // =====================================================================
              // FIX: VERIFY THE RESTRUCTURED ACTIVE HARWARE DATA PAYLOAD CACHE
              // =====================================================================
              if(Mlevellist[ui82]->ml_type == MLT_PWM){
                if(Order_Info.value.ui32val >= Mlevellist[ui82]->ntt){
                   snprintf(errmsg,ERRMSG_LEN,"Attempting to access a translation"
                   " table index that does not exist.");
                   cmdstatus=CMD_ERR; goto process_act_set;
                  } 

                // Stage content fields globally
                LK_freq = Mlevellist[ui82]->ml_tt[Order_Info.value.ui32val].freq;
                LK_duty = Mlevellist[ui82]->ml_tt[Order_Info.value.ui32val].duty;

                // Open the gate if the table data has changed, even if the index matches
                if(Mlevellist[ui82]->last_level == LK_mlevel &&
                   Mlevellist[ui82]->last.freq   == LK_freq &&
                   Mlevellist[ui82]->last.duty   == LK_duty) {
                    goto process_act_set; // Safe data packet filter abort
                }

                // Pack payload forms to ship across the serial bus
                Order_Info.value.biuinval[0] = LK_freq;
                Order_Info.value.biuinval[1] = LK_duty;

               } else { 
                // Parallel voice tracking type: Maintain your index block rule unchanged
                if(Order_Info.value.ui32val > Mlevellist[ui82]->max_level){
                   snprintf(errmsg,ERRMSG_LEN,"You can't set the level of this "
                   "actuator to more than %"PRIu32" because it only has %"PRIu8
                   " pins set.",Mlevellist[ui82]->max_level,Mlevellist[ui82]->npins);
                   cmdstatus=CMD_ERR; goto process_act_set;
                 } 
                 
                if(Mlevellist[ui82]->last_level == LK_mlevel){
                    goto process_act_set; 
                }

                uint8_t mlv8bit = (uint8_t)Order_Info.value.ui32val;
                Order_Info.value.ui8val = mlv8bit;
               }
             break;
             case INFO_ML_TTABLE:
              if(Mlevellist[ui82]->ml_type == MLT_UNSET){
                snprintf(errmsg,ERRMSG_LEN,"You must set the type of an M-Level actuator"
                " before you set anything else.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;
             default:
             break;
             }
         break;

         case ACT_SERVO:
           switch(Order_Info.infotype){
             case INFO_SRV_MIN:
               // Safety Guard: Force lower bounds to keep min strictly under max
               if(Order_Info.value.ui32val >= Servolist[ui82]->srv_max){
                snprintf(errmsg,ERRMSG_LEN,"You can't set a servo min value "
                "that is >= the current servo max value (%"PRIu32" us).", Servolist[ui82]->srv_max);
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;
             
             case INFO_SRV_MAX: {
              if(Order_Info.value.ui32val <= Servolist[ui82]->srv_min){
                snprintf(errmsg,ERRMSG_LEN,"You can't set a servo max value "
                "that is <= the current servo min value (%"PRIu32" us).", Servolist[ui82]->srv_min);
                cmdstatus=CMD_ERR; goto process_act_set;
               }
               
              // Dynamically resolve the absolute physical window limit based on the active band
              uint32_t max_frame_us = 20000; // Fallback default for SRV_FREQ_MID50 (50Hz)
              uint8_t current_band = Servolist[ui82]->freq;

              if      (current_band == SRV_FREQ_MID200) max_frame_us = 5000; // 200Hz ceiling
              else if (current_band == SRV_FREQ_MID333) max_frame_us = 3000; // 333Hz ceiling

              if(Order_Info.value.ui32val > max_frame_us){
                snprintf(errmsg, ERRMSG_LEN, "Servo max value (%"PRIu32" us) cannot exceed the physical "
                         "limit of your active frequency band (%"PRIu32" us).", 
                         Order_Info.value.ui32val, max_frame_us);
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             }
             break;
             case INFO_SRV_SPOS:
              if(Servolist[ui82]->maturity < ADULT_SERVO){
                snprintf(errmsg,ERRMSG_LEN,"You can't set the position of this "
                "servo because it is immature.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              if(userval1 < 0.0){
                snprintf(errmsg,ERRMSG_LEN,"Servo coordinates cannot be negative.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
               
              Order_Info.value.douval = userval1;
              // Convert user-supplied alignment (Degrees/Microseconds calibration pass)
              dirsteps = server_musecs(ui82, Order_Info.value.douval);
              Order_Info.value.ui32val = dirsteps;

              if(Order_Info.value.ui32val < Servolist[ui82]->srv_min){
                snprintf(errmsg,ERRMSG_LEN,"You can't set a servo position that"
                " is < the current servo min value.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              if(Order_Info.value.ui32val > Servolist[ui82]->srv_max){
                snprintf(errmsg,ERRMSG_LEN,"You can't set a servo position that"
                " is > the current servo max value.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;
             case INFO_SRV_QPOS:
              if(Servolist[ui82]->maturity < ADULT_SERVO){
                snprintf(errmsg,ERRMSG_LEN,"You can't queue the position of this "
                "servo because it is immature.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              if(userval1 < 0.0){
                snprintf(errmsg,ERRMSG_LEN,"Servo position values cannot be "
                "negative.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              // Now check we have an acceleration profile set for that actuator type
              if(sv_Accparam[ui81].set==0){
                 snprintf(errmsg,ERRMSG_LEN,"Actuator accel profile not set (use act_accel)");
                 cmdstatus=CMD_ERR; goto process_act_set;
                }
              Order_Info.value.douval = userval1;
              // Convert the user-supplied position value to microseconds if it
              // was supplied in degrees :
              dirsteps = server_musecs(ui82,Order_Info.value.douval);
              Order_Info.value.ui32val = dirsteps;  
              if(Order_Info.value.ui32val < Servolist[ui82]->srv_min){
                snprintf(errmsg,ERRMSG_LEN,"You can't queue a servo position that"
                " is < the current servo min value.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              if(dirsteps != SERVO_RESET){
                if(Order_Info.value.ui32val > Servolist[ui82]->srv_max){
                  snprintf(errmsg,ERRMSG_LEN,"You can't queue a servo position that"
                  " is > the current servo max value.");
                  cmdstatus=CMD_ERR; goto process_act_set;
                 } 
               }
             break;
             case INFO_SRV_THMIN:
              if(userval1 < 0.0){
                snprintf(errmsg,ERRMSG_LEN,"Servo minimum angle cannot be "
                "negative.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              Order_Info.value.douval = userval1;
             break;
             case INFO_SRV_THMAX:
              if(userval1 <= Servolist[ui82]->theta_min){
                snprintf(errmsg,ERRMSG_LEN,"Servo maximum angle cannot be "
                "<= servo minimum angle.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              Order_Info.value.douval = userval1;
             break;
             case INFO_SRV_FREQ:
              // Force frequency selection BEFORE pin allocation
              if(Servolist[ui82]->pin_srv != PIN_IS_UNSET){
                snprintf(errmsg, ERRMSG_LEN, "Forbidden: You cannot modify the"
                  " servo frequency band after a physical pin has been bound.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
               
              // Validate that the user passed a valid band token identifier (1, 2, or 3)
              if(Order_Info.value.ui8val < SRV_FREQ_MID50 ||
                 Order_Info.value.ui8val > SRV_FREQ_MID333){
                snprintf(errmsg, ERRMSG_LEN, "Invalid servo frequency band choice.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
              // ===================================================================
              // CROSS-VERIFICATION ENGINE FOR THE REVERSE INITIALIZATION LOOP
              // ===================================================================
              uint32_t target_frame_us = 20000;
              if      (Order_Info.value.ui8val == SRV_FREQ_MID200) target_frame_us = 5000;
              else if (Order_Info.value.ui8val == SRV_FREQ_MID333) target_frame_us = 3000;

              if(Servolist[ui82]->srv_max > target_frame_us){
                snprintf(errmsg, ERRMSG_LEN, "Forbidden: Your current servo max limit (%"PRIu32" us) "
                         "is too large for the requested %u Hz frequency band window (%"PRIu32" us). "
                         "Reduce your srv_max value first.", 
                         Servolist[ui82]->srv_max, 
                         (Order_Info.value.ui8val == SRV_FREQ_MID200) ? 200 : 333,
                         target_frame_us);
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;

             default: break;
             }
         break;

         case ACT_SENSOR:
           switch(Order_Info.infotype){
             case INFO_SEN_TYPE:
              if(Sensorlist[ui82]->type != SEN_UNSET){
                snprintf(errmsg,ERRMSG_LEN,"You can't change the type of a"
                                      " Sensor actuator once it has been set.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;
             case INFO_SEN_PIN:
              if(Sensorlist[ui82]->type == SEN_UNSET){
                snprintf(errmsg,ERRMSG_LEN,"You must set the type of a Sensor"
                " actuator before you set its pin.");
                cmdstatus=CMD_ERR; goto process_act_set;
               }
             break;
             default: break;
             }
         break;

         default:
           snprintf(errmsg,ERRMSG_LEN,"Actuator type does not exist");
           cmdstatus=CMD_ERR; goto process_act_set;
         break;
        }


        // Now test the info item to see if it is valid for the type of actuator
        // and whether we need to call the PARDuino or just set this locally.
        // Set the values accordingly or raise an error if there is a problem.

        from_moveto = 0; // act_moveto re-uses some code so we need this flag.
        moveto_changedir:

// DEBUG
// if(from_moveto){
//   printf("\n Changing Dirn: type:ID:action:newdirn = %"PRIu8":%"PRIu8":%"PRIu8":%"PRIu8,ui81,ui82,ui83,Order_Info.value.ui8val); 
//   fflush(stdout);
//  }
// DEBUG

        switch(act_set_source(ui81,errmsg)){
          case AIS_LOCAL:
            if(act_set_to_local(ui81,ui82,errmsg)){
               cmdstatus=CMD_ERR; goto process_act_set;
              }
          break;
          case AIS_BOB:
           // We need to set the data on the BOB. We filled the required order
           // forms above.
           // Send the order to the Parduino and assess the report
           Pard_Err=as_send_order();
           if(Pard_Err!=PARDU_ERR_NONE){
              snprintf(errmsg, ERRMSG_LEN, "%s", Pard_err_str());
              cmdstatus=CMD_ERR; goto process_act_set;
             } 
           // If it worked we also need to set the equivalent local struct
           // members to keep things in sync.
           act_sync_to_local(ui81,ui82,1);
          break;
          default:
           cmdstatus=CMD_ERR; goto process_act_set;
          break;
         }

        switch(from_moveto){
          case 1:
           from_moveto = 0; goto moveto_returnpoint;
          case 2:
           from_moveto = 0; goto moveto_x_returnpoint;
          case 3:
           from_moveto = 0; goto moveto_y_returnpoint;
          case 4:
           from_moveto = 0; goto moveto_z_returnpoint;
          default:
           from_moveto = 0; 
          break;            
        }
        // If we get to here it means we have successfully set the required
        // information. Some of these settings changes may require us to re-
        // calculate some other settings. So, before we finish let's see if any
        // such calculations need to be done, and do them.

        switch(ui81){
           case ACT_STEPPER:

          // 1. Has there been a change in MSDENOM, MSDEMAX, BLSTMAX
          //     or CALIMAX?
          //    If so, the calibration factor, and backlash steps, onestep and
          //    tensteps1 and tensteps2 need updating.
          // Reset a general flag, we'll need it below.
          gflag[0]=0; // This is for blsteps
          gflag[1]=0; // This is for onestep
          gflag[2]=0; // This is for tensteps1
          gflag[3]=0; // This is for tensteps2
          switch(Order_Info.infotype){
            case INFO_MSDEMAX:      // Server only  // LG
            case INFO_MSDENOM:                      // LG
            // Note: we don't include a case INFO_FASTMSD because that is only
            // used for homing where uncalibrated steps are used and where the
            // stepcount is set to 0 just after homing (so onestep is
            // irrelevant) and the microstep denominator is immediately and
            // automatically set to msdenom after homing so it is the msdenom
            // value that should be used in these calibration calcs,
            // not fastmsd.
            // A change to any of these can change the ratio by which the
            // read-only values of onestep and blsteps are calculated- so
            // update them.
  
            // onestep =  maxmsdenom / msdenom
            Stepperlist[ui82]->onestep = Stepperlist[ui82]->maxmsdenom / 
                                         Stepperlist[ui82]->msdenom;
            gflag[1]=1; // Note to send the updated onestep to Arduino
            // Only bother with blsteps if maxblsteps > 0
            if(Stepperlist[ui82]->maxblsteps){
              // blsteps =  maxblsteps * (msdenom / maxmsdenom)
              //         =  maxblsteps / onestep
              Stepperlist[ui82]->blsteps = Stepperlist[ui82]->maxblsteps /
                                           Stepperlist[ui82]->onestep;
              gflag[0]=1; // Note to send the updated blsteps to Arduino
             }
            // Only bother with tensteps1 if it is > 0
            if(Stepperlist[ui82]->tensteps1){
              // Ardu tensteps1 =  tensteps1 / onestep
              ardutensteps1 = Stepperlist[ui82]->tensteps1 /
                                           Stepperlist[ui82]->onestep;
              gflag[2]=1; // Note to send the updated ardutensteps1 to Arduino
             }
            // Only bother with tensteps2 if it is > 0
            if(Stepperlist[ui82]->tensteps2){
              // Ardu tensteps2 =  tensteps2 / onestep
              ardutensteps2 = Stepperlist[ui82]->tensteps2 /
                                           Stepperlist[ui82]->onestep;
              gflag[3]=1; // Note to send the updated ardutensteps2 to Arduino
             }
            break;
            case INFO_BLSTMAX:      // Server only  // LG
            // A change to this requires us to update the read-only blsteps.
              Stepperlist[ui82]->blsteps = Stepperlist[ui82]->maxblsteps /
                                           Stepperlist[ui82]->onestep;
            gflag[0]=1; // Note to send the updated blsteps to Arduino
            break;
            case INFO_TENSTEPS1:                    // LG
            // A change to this requires us to update the Arduino value.
              ardutensteps1 = Stepperlist[ui82]->tensteps1 /
                                           Stepperlist[ui82]->onestep;
              gflag[2]=1; // Note to send the updated ardutensteps1 to Arduino
            break;
            case INFO_TENSTEPS2:                    // LG
            // A change to this requires us to update the Arduino value.
              ardutensteps2 = Stepperlist[ui82]->tensteps2 /
                                           Stepperlist[ui82]->onestep;
              gflag[3]=1; // Note to send the updated ardutensteps2 to Arduino
            break;
            default: break;
           }
          // If the backlash steps were updated, send them to the Arduino
          if(gflag[0]){ 
             // Fill in the SET order form for backlash steps
             Order_Info.infotype=INFO_BLSTEPS;
             Order_Info.value.ui32val=Stepperlist[ui82]->blsteps;
             // Send the order to the Parduino and assess the report
             Pard_Err=as_send_order();
             if(Pard_Err!=PARDU_ERR_NONE){
                sprintf(errmsg,"%s",Pard_err_str());
                cmdstatus=CMD_ERR; goto process_act_set;
               } 
            }
          if(gflag[1]){ 
             // If the onestep was updated, send it to the Arduino
             // Fill in the SET order form for onestep
             Order_Info.infotype=INFO_ONESTEP;
             Order_Info.value.ui32val=Stepperlist[ui82]->onestep;
             // Send the order to the Parduino and assess the report
             Pard_Err=as_send_order();
             if(Pard_Err!=PARDU_ERR_NONE){
                sprintf(errmsg,"%s",Pard_err_str());
                cmdstatus=CMD_ERR; goto process_act_set;
               } 
            }
          if(gflag[2]){ 
             // If the ardutensteps1 was updated, send it to the Arduino
             // Fill in the SET order form for onestep
             Order_Info.infotype=INFO_TENSTEPS1;
             Order_Info.value.ui32val=ardutensteps1;
             // Send the order to the Parduino and assess the report
             Pard_Err=as_send_order();
             if(Pard_Err!=PARDU_ERR_NONE){
                sprintf(errmsg,"%s",Pard_err_str());
                cmdstatus=CMD_ERR; goto process_act_set;
               } 
            }
          if(gflag[3]){ 
             // If the ardutensteps2 was updated, send it to the Arduino
             // Fill in the SET order form for onestep
             Order_Info.infotype=INFO_TENSTEPS2;
             Order_Info.value.ui32val=ardutensteps2;
             // Send the order to the Parduino and assess the report
             Pard_Err=as_send_order();
             if(Pard_Err!=PARDU_ERR_NONE){
                sprintf(errmsg,"%s",Pard_err_str());
                cmdstatus=CMD_ERR; goto process_act_set;
               } 
            }

          // 2. Have there been any limit switch alterations that require the
          // limitstatus and/or homed_to status to be updated locally and/or
          // on Arduino? If a user wanted to change a limit switch pin
          // on-the-fly they would have to call act_set twice: first to unset
          // the currently used pin by passing a value of 255, then to set the
          // new pin with the required value. So check to see if the request is
          // to set a limit switch pin to 255 and act accordingly.
          // set homed_to and to LIM_UNSET if the pin being unset is the one the
          // stepper is homed to. While this is a client read-only value, the
          // server is allowed to set it. Likewise, if the limstate is set to 
          // the limit switch whose pin has just been unset then limstate must
          // also be set to LIM_UNSET and, again, even though INFO_LIMSTATE is
          // client read-only, the server can set it.

          // Reset two general flags - we'll need them below.
          gflag[0]=0; // This is for if to do a homed_to update
          gflag[1]=0; // This is for if to do a limstate update 
          if(Stepperlist[ui82]->pin_lim1 == PIN_IS_UNSET){
             if(Stepperlist[ui82]->homed_to == LIM_SWTC1){
               Stepperlist[ui82]->homed_to = LIM_UNSET;
               gflag[0]++;
              }
             if(Stepperlist[ui82]->limstate == LIM_SWTC1){
               Stepperlist[ui82]->limstate = LIM_UNSET;
               gflag[1]++;
              }
            }
          if(Stepperlist[ui82]->pin_lim2 == PIN_IS_UNSET){
             if(Stepperlist[ui82]->homed_to == LIM_SWTC2){
               Stepperlist[ui82]->homed_to = LIM_UNSET;
               gflag[0]++;
              }
             if(Stepperlist[ui82]->limstate == LIM_SWTC2){
               Stepperlist[ui82]->limstate = LIM_UNSET;
               gflag[1]++;
              }
            }
          if(act_nhs_invalid(ui81,ui82)){
             if( (Stepperlist[ui82]->homed_to == LIM_NULL1) ||
                 (Stepperlist[ui82]->homed_to == LIM_NULL2) ){
               Stepperlist[ui82]->homed_to = LIM_UNSET;
               gflag[0]++;
              }
           }
          // There is no equivalent to the '== PIN_IS_UNSET' for software limits
          // and software limits cannot be 'homed_to'. Also software limits do
          // not work the same way as hardware limits in that it is not what the
          // axis is homed to that counts but what direction the axis is
          // currently travelling in. In the Arduino, limstate is always set to
          // LIM_UNSET before any motion and then triggered to something else if
          // a limit is exceeded during motion (or right before the first step
          // is taken. For these reasons I do not make any adjustments here -
          // the user will find out as soon as they try to move if the new
          // software limits they just set forbid motion in any direction (and
          // really they should be on top of that before changing any software
          // limits) so we take no action here if either a software limit was
          // set or a stepcount value was set.

          // If indicated, update the homed_to member on the Arduino and locally
          if(gflag[0]){
            // Fill in the SET info order form for a homed_to info item
            Order_Info.infotype=INFO_HOMED_TO;
            Order_Info.value.ui8val=Stepperlist[ui82]->homed_to;
            // Send the order to the Parduino and assess the report
            Pard_Err=as_send_order();
            if(Pard_Err!=PARDU_ERR_NONE){
               sprintf(errmsg,"%s",Pard_err_str());
               cmdstatus=CMD_ERR; goto process_act_set;
              } 
           }
          // If indicated, update the limstate member on the Arduino and locally
          // for the specific stepper
          if(gflag[1]){
            // Fill in the SET info order form for a limstate info item
            Order_Info.infotype=INFO_LIMSTATE;
            Order_Info.value.ui8val=Stepperlist[ui82]->limstate;
            // Send the order to the Parduino and assess the report
            Pard_Err=as_send_order();
            if(Pard_Err!=PARDU_ERR_NONE){
               sprintf(errmsg,"%s",Pard_err_str());
               cmdstatus=CMD_ERR; goto process_act_set;
              } 
           }

         break;
         case ACT_BINARY:
         break;
         case ACT_MLEVEL:
         break;
         case ACT_SERVO:
         break;
         case ACT_SENSOR:
         break;
         default: break;
        }
        
        // Update the maturity factor (act_set may have changed this)
        act_update_maturity(ui81,ui82);

        // All done. Send result back to client.
        process_act_set:
        if(from_moveto){from_moveto = 0; goto moveto_returnpoint;}
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");

       } else if(!strcmp(sv_triad_argv(1),"act_home")){  
        sscanf(sv_triad_argv(2),"%"SCNu8,&ui81); // Actuator type
        sscanf(sv_triad_argv(3),"%"SCNu8,&ui82); // Actuator index
        sscanf(sv_triad_argv(4),"%"SCNu8,&ui83); // Home type

        // First check if the actuator exists:
        if(!actuator_exists(ui81,ui82)){
           sprintf(errmsg,"Actuator does not exist");
           cmdstatus=CMD_ERR; goto process_act_home;
          }

        // Fill in the order forms (ID and Home):
        Pard_Err=act_fillforms_home(ui81,ui82,ui83);
        if(Pard_Err!=PARD_OK){
           sprintf(errmsg,"%s",Pard_err_str());
           cmdstatus=CMD_ERR; goto process_act_home;
          } 

        // Send the order to the Parduino and assess the report
        Pard_Err=as_send_order();
        if(Pard_Err!=PARDU_ERR_NONE){
           sprintf(errmsg,"%s",Pard_err_str());
           cmdstatus=CMD_ERR; goto process_act_home;
          } 

        process_act_home:
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");


       } else if(!strcmp(sv_triad_argv(1),"act_halt")){  
        sscanf(sv_triad_argv(2),"%"SCNu8,&ui81); // Actuator type
        sscanf(sv_triad_argv(3),"%"SCNu8,&ui82); // Actuator index

        // First check if the actuator exists:
        if(!actuator_exists(ui81,ui82)){
           sprintf(errmsg,"Actuator does not exist");
           cmdstatus=CMD_ERR; goto process_act_halt;
          }

        // Fill in the ID order form
        Order_ID.otype  = ORDER_HALT;
        Order_ID.actype = ui81;
        Order_ID.actid  = ui82;

        if(ui81==ACT_MLEVEL){
          // Reset the repeat inhibitor. Even if the command fails, the user
          // intended a hard reset so we can do this prior to sending:
          Mlevellist[Order_ID.actid]->last_level = MLEVEL_RESET; 
         }

        // Send the order to the Parduino and assess the report
        Pard_Err=as_send_order();
        if(Pard_Err!=PARDU_ERR_NONE){
           sprintf(errmsg,"%s",Pard_err_str());
           cmdstatus=CMD_ERR; goto process_act_halt;
          } 

        process_act_halt:
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");

       } else if(!strcmp(sv_triad_argv(1),"act_unit")){  
        sscanf(sv_triad_argv(2),"%"SCNu8,&ui81); // Actuator type
        sscanf(sv_triad_argv(3),"%"SCNu8,&ui82); // Unit choice
        // Set the sv_Unitype
        sv_Unitype[ui81]=ui82;
        sprintf(cmdtriad,"OK Done Ready");

       } else if(!strcmp(sv_triad_argv(1),"act_accel")){  
        sscanf(sv_triad_argv(2),"%"SCNu8,&ui81); // Actuator type

        // Read addition arguments based on the actuator type
        switch(ui81){
           case ACT_STEPPER:
            sscanf(sv_triad_argv(3),"%la",&sv_Accparam[ui81].p_acc);
            sscanf(sv_triad_argv(4),"%"SCNu32,&sv_Accparam[ui81].delay_min);
            sscanf(sv_triad_argv(5),"%"SCNu32,&sv_Accparam[ui81].delay_max_min);
            // Mark the stepper sv_Accparam as being set
            sv_Accparam[ui81].set=1;
           break;
           case ACT_BINARY:
           sprintf(errmsg,"Command not applicable for a binary actuator type");
           cmdstatus=CMD_ERR;
           break;
           case ACT_MLEVEL:
           sprintf(errmsg,"Command not applicable for an M-level actuator type");
           cmdstatus=CMD_ERR;
           break;
           case ACT_SERVO:
            sscanf(sv_triad_argv(3),"%"SCNu8,&sv_Accparam[ui81].s_acc);
            sscanf(sv_triad_argv(4),"%"SCNu32,&sv_Accparam[ui81].duration);
            // Mark the servo sv_Accparam as being set
            sv_Accparam[ui81].set=1;
            // Unlike steppers, the servo acceleration values are held on the
            // Arduino so send the information there:

            // Fill in the ID order form
            Order_ID.otype  = ORDER_ACCEL;
            Order_ID.actype = ui81;
            Order_ID.actid  = 0; // This is a type-specific setting (not
                                 // actuator-specific) so we just use 0.

           // Send the order to the Parduino and assess the report
           Pard_Err=as_send_order();
           if(Pard_Err!=PARDU_ERR_NONE){
              sprintf(errmsg,"%s",Pard_err_str());
              cmdstatus=CMD_ERR; goto process_act_accel;
             } 

           break;
           case ACT_SENSOR:
           sprintf(errmsg,"Command not applicable for a Sensor actuator type");
           cmdstatus=CMD_ERR;
           break;
           default:
             sprintf(errmsg,"Actuator type does not exist");
             cmdstatus=CMD_ERR;
           break;
          }

        process_act_accel:
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");


       } else if(!strcmp(sv_triad_argv(1),"act_moveby")){  
        sscanf(sv_triad_argv(2),"%"SCNu8,&ui81); // Actuator type
        sscanf(sv_triad_argv(3),"%"SCNu8,&ui82); // Actuator index
        sscanf(sv_triad_argv(4),"%"SCNu32,&sv_Accparam[ui81].duration);
        sscanf(sv_triad_argv(5),"%la",&userval1);

        // First check if the actuator exists:
        if(!actuator_exists(ui81,ui82)){
           sprintf(errmsg,"Actuator does not exist");
           cmdstatus=CMD_ERR; goto process_act_moveby;
          }

        // AUDIT (26.10.2026): only a MATURE stepper, or a stage whose axes are all
        // mature, may move. A pin unset with 'act_set ... 255' (the way to change
        // a pin) lowers the maturity (act_update_maturity()), but the firmware
        // checks only homed_to and ena_state before a move, and would pulse pin
        // 255 (port_buffer_loadpin(255): a write to a stray address).
        if(ui81 == ACT_STEPPER){
            uint8_t ax, ax_first = ui82, ax_last = ui82;
            if(ui82 == ACT_ID_STAGE_XY){ ax_first = ACT_ID_STAGE_X; ax_last = ACT_ID_STAGE_Y; }
            if(ui82 == ACT_ID_STAGE_XYZ){ ax_first = ACT_ID_STAGE_X; ax_last = ACT_ID_STAGE_Z; }
            for(ax = ax_first; ax <= ax_last; ax++){
               if(Stepperlist[ax]->maturity < adult_stepper(Stepperlist[ax]->driver_type)){
                   snprintf(errmsg,ERRMSG_LEN,"Stepper %"PRIu8" is immature (a pin or a"
                                    " setting is missing): it cannot move.",ax);
                   cmdstatus=CMD_ERR; goto process_act_moveby;
                 }
              }
          }

        // Now check we have an acceleration profile set for that actuator type
        if(sv_Accparam[ui81].set==0){
           sprintf(errmsg,"Actuator accel profile not set (use act_accel)");
           cmdstatus=CMD_ERR; goto process_act_moveby;
          }

        // Fill in the parts of the ID order form common to all actuators
        // Fill in the ID order form
        Order_ID.actype = ui81;
        Order_ID.actid  = ui82;

        // For steppers:
        // ui82 is used as an individual motor ID so first lets offload it
        // into stage_id so we can use ui82 the way it was intended. This
        // is particularly important when branching off to other code that
        // expect ui82 to be an individual motor ID like 'act_set':
        stage_id=ui82;

        // Calculate the motion parameters and complete the required forms:
        // Order_ID, Order_DirSteps and Order_Motion:
        switch(ui81){
           case ACT_STEPPER:
             // Complete the ID order form
             Order_ID.otype  = ORDER_STEPPER_MOVE;
             // Set dirsteps according to the unit type choice so it ends up as
             // an integer number of steps in the current msdenom step size.
             // Also set id_a to the ID of the index motor
             if(stage_id < ACT_ID_STAGE_XY){
               // The distance to move by (userval1) will have been supplied in
               // either atomic steps or real world units. The first step is to
               // convert it to atomic steps if real worl units are in effect:
               position=sv_posdist_to_maxmsdenomsteps(ui82, userval1);
               // We only send integer steps to the Arduino and these will be
               // performed at the motor's current msdenom setting. So the next
               // step is to convert the atomic steps distance ('position') into
               // steps at the motor's current msdenom microstep size:
               dirsteps=(uint32_t)(0.5 + (position/Stepperlist[ui82]->onestep));
               id_a=ui82;
              } else {
              // I will use rc as a counter to know if >1 axis has a dirsteps>0
              // because, if only one axis has dirsteps>0, then we switch over
              // to the more efficient single axis act_moveby instead of
              // continuing with the stage version of act_moveby (which has more
              // overheads on the Arduino) so initialise this marker to 0:
              rc=0;


              // Dirsteps for X
               position=sv_posdist_to_maxmsdenomsteps(ACT_ID_STAGE_X, userval1);
               dirsteps_x = (uint32_t)
                      (0.5 + (position / Stepperlist[ACT_ID_STAGE_X]->onestep));
               if(dirsteps_x) rc++; 

              // Dirsteps for Y
               sscanf(sv_triad_argv(6),"%la",&userval1);
               position=sv_posdist_to_maxmsdenomsteps(ACT_ID_STAGE_Y, userval1);
               dirsteps_y = (uint32_t)
                      (0.5 + (position / Stepperlist[ACT_ID_STAGE_Y]->onestep));
               if(dirsteps_y) rc++; 

              // Dirsteps for Z
              if(stage_id == ACT_ID_STAGE_XYZ){
                sscanf(sv_triad_argv(7),"%la",&userval1);
                position=sv_posdist_to_maxmsdenomsteps(ACT_ID_STAGE_Z,userval1);
                dirsteps_z = (uint32_t)
                      (0.5 + (position / Stepperlist[ACT_ID_STAGE_Z]->onestep));
               } else dirsteps_z = 0;
               if(dirsteps_z) rc++; 

              // Select the maximum to use as index motor for calculating the
              // accelaration profile:
              dirsteps=dirsteps_z; id_a=ACT_ID_STAGE_Z;
              if(dirsteps_x > dirsteps){
                 dirsteps = dirsteps_x; id_a=ACT_ID_STAGE_X;
                }
              if(dirsteps_y > dirsteps){
                 dirsteps = dirsteps_y; id_a=ACT_ID_STAGE_Y;
                }

               // Now test if >1 axis needs moving and convert the order to
               // STAGE_MOVE (instead of STEPPER_MOVE) if true but revert to
               // STEPPER_MOVE and set the stepper ID to id_a if false.
               if(dirsteps){ // If dirsteps is 0 we do nothing anyway.
                  if(rc<=1){ // Only one axis, so switch over to STEPPER_MOVE:
                    ui82=id_a;
                    Order_ID.otype  = ORDER_STEPPER_MOVE;
                    Order_ID.actid  = ui82;
                   } else Order_ID.otype = ORDER_STAGE_MOVE;
                 }

              }

             moveto_moveby_stepper: // Entry point for the act_moveto command

             // Special case for dirsteps=0 so we don't waste time sending a
             // a move order to the Arduino for this.
             if(dirsteps==0){
               sv_PMR_null=1; // Prevent calls to act_movereport from hanging.
               sv_PMR_null_actype = ACT_STEPPER;
               // The ID must be for a single actuator because we access its
               // maxcalibri member from its struct and compound stage IDs
               // do not relate to any single struct so I default to X if
               // a compount ID is given in stasge_id:
               sv_PMR_null_actid  = 
                            (stage_id>=ACT_ID_STAGE_XY)?ACT_ID_STAGE_X:stage_id;

               goto process_act_moveby;
              } else sv_PMR_null=0;

             // Complete the DirSteps order form
             if(stage_id < ACT_ID_STAGE_XY){
               Order_DirSteps.dirsteps_z=dirsteps;
              } else {
               Order_ID.actid=stage_id;
               Order_DirSteps.dirsteps_x=dirsteps_x;
               Order_DirSteps.dirsteps_y=dirsteps_y;
               if(stage_id == ACT_ID_STAGE_XYZ)
                Order_DirSteps.dirsteps_z=dirsteps_z;
               else Order_DirSteps.dirsteps_z=0;
              }

             // Calculate the motion parameters & complete the Motion order form
             ui83 = act_calc_motion(dirsteps,
                                    sv_Accparam[ACT_STEPPER].duration,
                                    sv_Accparam[ACT_STEPPER].p_acc,
                                    sv_Accparam[ACT_STEPPER].delay_min,
                                    sv_Accparam[ACT_STEPPER].delay_max_min);

             // Force  Order_Motion.delay_max to be sensible if it isn't.
             // Be sure to set id_a apporpriately over at act_moveto before
             // coming here from there.
             ui32 = (2*Stepperlist[id_a]->pulwidth);
             if(Order_Motion.delay_max < ui32) Order_Motion.delay_max=ui32;

           break;
           case ACT_BINARY:
             sprintf(errmsg,"Command not applicable for a binary actuator");
             cmdstatus=CMD_ERR; goto process_act_moveby;
           break;
           case ACT_MLEVEL:
             sprintf(errmsg,"Command not applicable for an M-Level actuator");
             cmdstatus=CMD_ERR; goto process_act_moveby;
           break;
           case ACT_SERVO:
             sprintf(errmsg,"Command not applicable for a servo actuator");
             cmdstatus=CMD_ERR; goto process_act_moveby;
           break;
           case ACT_SENSOR:
             sprintf(errmsg,"Command not applicable for a sensor actuator");
             cmdstatus=CMD_ERR; goto process_act_moveby;
           break;
           default:
             sprintf(errmsg,"Actuator type does not exist");
             cmdstatus=CMD_ERR; goto process_act_moveby;
           break;
          }

        // Send the order to the Parduino and assess the report
        Pard_Err=as_send_order();
        if(Pard_Err!=PARDU_ERR_NONE){
           sprintf(errmsg,"%s",Pard_err_str());
           cmdstatus=CMD_ERR; goto process_act_moveby;
          } 

        process_act_moveby:
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else {
          if(sv_blob_resize(1)){
            sprintf(errmsg,"sv_blob resize failed for motion retrurn value");
            cmdstatus=CMD_ERR; goto process_act_moveby;
           }
          memcpy(sv_Blob,&ui83,1);
          sending_blob=1;
          sprintf(cmdtriad,"OK Done Ready");
         }

       } else if(!strcmp(sv_triad_argv(1),"act_moveto")){  
        sscanf(sv_triad_argv(2),"%"SCNu8,&ui81);  // Actuator type
        sscanf(sv_triad_argv(3),"%"SCNu8,&ui82);  // Actuator index
        sscanf(sv_triad_argv(4),"%"SCNu32,&si32); // Actuator-specific value

        // First check if the actuator exists:
        if(!actuator_exists(ui81,ui82)){
           sprintf(errmsg,"Actuator does not exist");
           cmdstatus=CMD_ERR; goto process_act_moveto;
          }

        // AUDIT (26.10.2026): only a MATURE stepper, or a stage whose axes are all
        // mature, may move.  A pin unset with 'act_set ... 255' (the way to change
        // a pin) lowers the maturity (act_update_maturity()), but the firmware
        // checks only homed_to and ena_state before a move, and would pulse pin
        // 255 (port_buffer_loadpin(255): a write to a stray address).
        if(ui81 == ACT_STEPPER){
          uint8_t ax, ax_first = ui82, ax_last = ui82;
          if(ui82 == ACT_ID_STAGE_XY){ ax_first = ACT_ID_STAGE_X; ax_last = ACT_ID_STAGE_Y; }
          if(ui82 == ACT_ID_STAGE_XYZ){ ax_first = ACT_ID_STAGE_X; ax_last = ACT_ID_STAGE_Z; }
          for(ax = ax_first; ax <= ax_last; ax++){
            if(Stepperlist[ax]->maturity < adult_stepper(Stepperlist[ax]->driver_type)){
              snprintf(errmsg,ERRMSG_LEN,"Stepper %"PRIu8" is immature (a pin or a"
                       " setting is missing): it cannot move.",ax);
              cmdstatus=CMD_ERR; goto process_act_moveto;
             }
           }
         }

        // Now check we have an acceleration profile set for that actuator
        if(sv_Accparam[ui81].set==0){
           sprintf(errmsg,"Actuator accel profile not set (use act_accel)");
           cmdstatus=CMD_ERR; goto process_act_moveto;
          }

        // Fill in the parts of the ID order form common to all actuators
        // Fill in the ID order form
        Order_ID.actype = ui81;
        Order_ID.actid  = ui82;


        // Calculate the motion parameters and complete the required forms:
        // Order_ID, Order_DirSteps and Order_Motion:
        switch(ui81){
           case ACT_STEPPER:

             // The third parameter for steppers is duration - so set it
             sv_Accparam[ui81].duration = si32;

             // Get the position to move to:
             sscanf(sv_triad_argv(5),"%la",&userval1);

             // ui82 is used as an individual motor ID so first lets offload it
             // into stage_id so we can use ui82 the way it was intended. This
             // is particularly important when branching off to other code that
             // expect ui82 to be an individual motor ID like 'act_set':
             stage_id=ui82;

             // Set the position the user wants us to move to according to the
             // unit type choice so it ends up as number of atomic microsteps:
             if(stage_id < ACT_ID_STAGE_XY){
               position=sv_posdist_to_maxmsdenomsteps(ui82,userval1);
              } else {

              // I will use rc as a counter to know if >1 axis has a dirsteps>0
              // because, if only one axis has dirsteps>0, then we switch over
              // to the more efficient single axis act_moveby instead of
              // continuing with the stage version of act_moveby (which has more
              // overheads on the Arduino) so initialise this marker to 0:
              rc=0;

              // Position for X
               position_x=
                        sv_posdist_to_maxmsdenomsteps(ACT_ID_STAGE_X,userval1);

              // Position for Y
               sscanf(sv_triad_argv(6),"%la",&userval1);
               position_y=
                        sv_posdist_to_maxmsdenomsteps(ACT_ID_STAGE_Y,userval1);

              // Position for Z
              if(stage_id == ACT_ID_STAGE_XYZ){
                 sscanf(sv_triad_argv(7),"%la",&userval1);
                 position_z=
                        sv_posdist_to_maxmsdenomsteps(ACT_ID_STAGE_Z,userval1);
                }

              }

             // Convert position from atomic microsteps into number of steps at
             // the motor's current msdenom (dirsteps - which is the integer
             // step currency used by the Arduino) and direction by comparison 
             // with the actuator's current position stepcount and set the id of 
             // the index motor into id_a:

             if(stage_id < ACT_ID_STAGE_XY){

                  if(sv_moveto_to_moveby(ui82,&dirsteps,errmsg,&reqdirn,
                                                                &ui83,position))
                    { cmdstatus=CMD_ERR; goto process_act_moveto; }

               // Now find the index motor and put it's id into id_a
               id_a = ui82; // There's only one motor, so that is the index.
              
               // If we're there already, just return success
               if(dirsteps==0){goto moveto_moveby_stepper;}
                
               // If we need to change direction, do it now
               if(Order_ID.otype == ORDER_SET){
                   from_moveto = 1;
                   goto moveto_changedir; // Re-use the act_set code
                   moveto_returnpoint:
                   if(cmdstatus==CMD_ERR) goto process_act_moveto;
                }

               // If we get to here then the motor is pointing in the right
               // direction and the number of steps to move is > 0 and has been
               // set into dirsteps. The rest is just identical to act_moveby,
               // so re-use that code:
               Order_ID.otype  = ORDER_STEPPER_MOVE; // Set the order type

              } else { // End of single motor block, now for STAGE block ...

               // Process X Axis
               // --------------
               // First we must convert the motor's stepcount from its native
               // value  measured in 1/maxmsdenom microsteps) into steps of size
               // 1/msdenom so as to be comparable to 'position':
               ui82=ACT_ID_STAGE_X;

               if(sv_moveto_to_moveby(ui82,&dirsteps_x,errmsg,&reqdirn,
                                                              &ui83,position_x))
                 { cmdstatus=CMD_ERR; goto process_act_moveto; }

               // Note if there any X steps to take or a direction change
               if(dirsteps_x){
                  rc++;
                
                 // If we need to change direction, do it now
                 if(Order_ID.otype == ORDER_SET){
                   from_moveto = 2;
                   goto moveto_changedir; // Re-use the act_set code
                   moveto_x_returnpoint:
                   if(cmdstatus==CMD_ERR) goto process_act_moveto;
                  }

                }

               // Process Y Axis
               // --------------
               // First we must convert the motor's stepcount from its native
               // value  measured in 1/maxmsdenom microsteps) into steps of size
               // 1/msdenom so as to be comparable to 'position':
               ui82=ACT_ID_STAGE_Y;
               if(sv_moveto_to_moveby(ui82,&dirsteps_y,errmsg,&reqdirn,
                                                              &ui83,position_y))
                 { cmdstatus=CMD_ERR; goto process_act_moveto; }

               // Note if there any Y steps to take or a direction change
               if(dirsteps_y){
                  rc++;
                
                 // If we need to change direction, do it now
                 if(Order_ID.otype == ORDER_SET){
                   from_moveto = 3;
                   goto moveto_changedir; // Re-use the act_set code
                   moveto_y_returnpoint:
                   if(cmdstatus==CMD_ERR) goto process_act_moveto;
                  }

                }


              if(stage_id == ACT_ID_STAGE_XYZ){

               // Process Z Axis
               // --------------
               // First we must convert the motor's stepcount from its native
               // value  measured in 1/maxmsdenom microsteps) into steps of size
               // 1/msdenom so as to be comparable to 'position':
               ui82=ACT_ID_STAGE_Z;
               if(sv_moveto_to_moveby(ui82,&dirsteps_z,errmsg,&reqdirn,
                                                              &ui83,position_z))
                 { cmdstatus=CMD_ERR; goto process_act_moveto; }

               // Note if there any Z steps to take or a direction change
               if(dirsteps_z){
                  rc++;
                
                 // If we need to change direction, do it now
                 if(Order_ID.otype == ORDER_SET){
                   from_moveto = 4;
                   goto moveto_changedir; // Re-use the act_set code
                   moveto_z_returnpoint:
                   if(cmdstatus==CMD_ERR) goto process_act_moveto;
                  }

                }


               } else dirsteps_z=0;


              // Select the maximum to use as index motor for calculating the
              // accelaration profile:
              dirsteps=dirsteps_z; id_a=ACT_ID_STAGE_Z;
              if(dirsteps_x > dirsteps){
                 dirsteps = dirsteps_x; id_a=ACT_ID_STAGE_X;
                }
              if(dirsteps_y > dirsteps){
                 dirsteps = dirsteps_y; id_a=ACT_ID_STAGE_Y;
                }

               // Now test if >1 axis needs moving and convert the order to
               // STAGE_MOVE (instead of STEPPER_MOVE) if true but revert to
               // STEPPER_MOVE and set the stepper ID to id_a if false.
               if(dirsteps){ // If dirsteps is 0 we do nothing anyway.
                  if(rc<=1){ // Only one axis, so switch over to STEPPER_MOVE:
                    stage_id=ui82=id_a;
                    Order_ID.otype  = ORDER_STEPPER_MOVE;
                    Order_ID.actid  = ui82;
                   } else {
                    Order_ID.otype = ORDER_STAGE_MOVE;
                    Order_ID.actid = stage_id;
                   }
                 }

              }

             goto moveto_moveby_stepper;
           break;
           case ACT_BINARY:
             sprintf(errmsg,"Command not applicable for a binary actuator");
             cmdstatus=CMD_ERR; goto process_act_moveto;
           break;
           case ACT_MLEVEL:
             sprintf(errmsg,"Command not applicable for an M-Level actuator");
             cmdstatus=CMD_ERR; goto process_act_moveto;
           break;
           case ACT_SERVO:
              // Initialise the PMR flag:
              sv_Accparam[ACT_SERVO].pmr_pending = 0;
              // Complete the order form
              Order_ID.otype  = ORDER_SERVO_MOVE;
              // Prepare the parameter to send to the Arduino:
              Order_Info.value.ui8val = (uint8_t)si32;              
              // Send the order to the Parduino and assess the report
              Pard_Err=as_send_order();
              if(Pard_Err!=PARDU_ERR_NONE){
                 snprintf(errmsg, ERRMSG_LEN, "%s", Pard_err_str());
                 cmdstatus=CMD_ERR; goto process_act_moveto;
                } 
           break;
           case ACT_SENSOR:
             sprintf(errmsg,"Command not applicable for a Sensor actuator");
             cmdstatus=CMD_ERR; goto process_act_moveto;
           break;
           default:
             sprintf(errmsg,"Actuator type does not exist");
             cmdstatus=CMD_ERR; goto process_act_moveto;
           break;
          }

        process_act_moveto:
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");

       // Post successful command processing:
        switch(ui81){
           case ACT_STEPPER:
           break;
           case ACT_BINARY:
           break;
           case ACT_MLEVEL:
           break;
           case ACT_SERVO:
            if(sv_Accparam[ACT_SERVO].pmr_pending == 0){
               sv_PMR_null=1; // Prevent calls to act_movereport from hanging.
               sv_PMR_null_actype = ACT_SERVO;
               sv_PMR_null_actid  = ui82;
             } else sv_PMR_null=0;
           // Wipe the queue
            for(ui82=0;ui82<SERVOSMAX;ui82++){
                if(actuator_exists(ACT_SERVO,ui82)){
                  Servolist[ui82]->destn_us = SERVO_RESET; 
                }
             }
           break;
           case ACT_SENSOR:
           break;
           default: break;
          }



       } else if(!strcmp(sv_triad_argv(1),"act_movereport")){  
        // If no movement was done at the last move command because the dirsteps
        // happened to equal 0 for a stepper or if no positional change was
        // made for a servo or the servo executed a set-and-forget move as
        // indicated by feedback from the Arduino, then no PMR will never be
        // issued on the serial port. However, the PCS script writer may be
        // checking for one before allowing the script to continue so we
        // manufacture a PMR for this eventuality right here:
        if(sv_PMR_null){
           if(sv_blob_resize(CLI_PMRBYTES)){
             sprintf(errmsg,"sv_blob resize failed for PMR");
             cmdstatus=CMD_ERR;
             goto process_act_movereport;
            }
           ui81=sv_PMR_null_actype;
           ui82=sv_PMR_null_actid;
           ui83=PARDU_ERR_NONE;
           sv_PMR_null=0;
         } else {

           // See if a PMR is on the inbound serial buffer, read it if so:
           switch(as_check_move_status(&ui81,&ui82,&ui83,errmsg)){
             case 0: // No PMR, return success but without a blob
              sprintf(cmdtriad,"OK Done Ready");
              goto end_act_movereport;
             break;
             case 1: // Got a PMR, process it into a blob and return success

              if(sv_blob_resize(CLI_PMRBYTES)){
                sprintf(errmsg,"sv_blob resize failed for PMR");
                goto process_act_movereport;
               }

              Pard_Err = (int)ui83;

              // AUDIT (22.10.2026): a move cut short by a limit switch, a step-
              // count limit or a HALT (the non-fatal codes, 61 to 63) still
              // reports where the motors are.  Keep those positions, as for a
              // move that ended normally, before the code is reported.
              if(Pard_Err >= PARDERR_NON_FATAL && ui81 == ACT_STEPPER){
                 if(StageStatus >= ACT_ID_STAGE_XY){
                   Stepperlist[ACT_ID_STAGE_X]->stepcount=PMRep1;
                   Stepperlist[ACT_ID_STAGE_Y]->stepcount=PMRep2;
                  }
                 if(StageStatus == ACT_ID_STAGE_XYZ)
                   Stepperlist[ACT_ID_STAGE_Z]->stepcount=PMRep3;
                 if(ui82 < ACT_ID_STAGE_XY) Stepperlist[ui82]->stepcount=PMRep4;
                }

              if(Pard_Err!=PARDU_ERR_NONE){
                 sprintf(errmsg,"%s (PMR)",Pard_err_str());
                 cmdstatus=CMD_ERR;
                 goto process_act_movereport;
                } 

              // Actuator-type specific code:
              switch(ui81){
                case ACT_STEPPER:
                  // Update local copies of the stepcounts
                  if(StageStatus >= ACT_ID_STAGE_XY){
                    Stepperlist[ACT_ID_STAGE_X]->stepcount=PMRep1;
                    Stepperlist[ACT_ID_STAGE_Y]->stepcount=PMRep2;
                   }
                  if(StageStatus == ACT_ID_STAGE_XYZ)
                    Stepperlist[ACT_ID_STAGE_Z]->stepcount=PMRep3;
                  // Only perform individual stepcount assignment if ui82 represents 
                  // a genuine single motor index (< 254), preventing out-of-bounds
                  // pointer crashes
                  if(ui82 < ACT_ID_STAGE_XY){
                     Stepperlist[ui82]->stepcount=PMRep4;
                  }
                  // Return success
                break;
                case ACT_SERVO:
                break;
                default: break;
              }

             break;
             case 3:   // AUDIT (26.10.2026): a valid PMR with a FATAL code (for
                       // example the kill switch, 27): keep the positions it
                       // reports, as for the non-fatal codes above, then fail.
              if(ui81 == ACT_STEPPER){
                    if(StageStatus >= ACT_ID_STAGE_XY){
                       Stepperlist[ACT_ID_STAGE_X]->stepcount=PMRep1;
                       Stepperlist[ACT_ID_STAGE_Y]->stepcount=PMRep2;
                      }
                    if(StageStatus == ACT_ID_STAGE_XYZ)
                       Stepperlist[ACT_ID_STAGE_Z]->stepcount=PMRep3;
                    if(ui82 < ACT_ID_STAGE_XY) Stepperlist[ui82]->stepcount=PMRep4;
                  }
              cmdstatus=CMD_ERR; // errmsg set by as_check_move_status
             break;
             case 2: // Error occurred, return failure with an error blob.
               cmdstatus=CMD_ERR; // errmsg set by as_check_move_status
             break;
             default:
              sprintf(errmsg,"as_check_move_status returned an unknown value "
                                                               "(program bug)");
              cmdstatus=CMD_ERR;
             break;
            }
          }
         process_act_movereport:
         if(cmdstatus==CMD_ERR){
           sprintf(cmdtriad,"ERROR FailedCmd Ready");
           sending_bloberr=1;
          } else {
            sprintf(cmdtriad,"OK Done Ready");
            memcpy(sv_Blob   ,&ui81,1); // Actuator type
            memcpy(sv_Blob+1 ,&ui82,1); // Actuator ID
            memcpy(sv_Blob+2 ,&ui83,1); // Error status
            // Send atomic uint32_t PMRep values directly to the client:
            memcpy(sv_Blob+3 ,&PMRep1,4);
            memcpy(sv_Blob+7 ,&PMRep2,4);
            memcpy(sv_Blob+11,&PMRep3,4);
            memcpy(sv_Blob+15,&PMRep4,4);
            sending_blob=1;
          }
        end_act_movereport:
          ; // Null statement to satisfy goto label rules.
       } else if(!strcmp(sv_triad_argv(1),"select_camera")){
        sscanf(sv_triad_argv(2),"%"SCNu32,&ui32); // Camera
        // Sanity checks on the range
        if(ui32<1){
           sprintf(errmsg,"Camera number can't be less than 1");
           cmdstatus=CMD_ERR; goto process_select_camera;
          }
        if(ui32>NPSs){
           sprintf(errmsg,"You only have %"PRIu32" cameras to choose from",
                                                                          NPSs);
           cmdstatus=CMD_ERR; goto process_select_camera;
          }
        if(ui32==CurrPS) goto process_select_camera; // Nothing to do
        ui32--; // Convert camera number into array index
        // Is the wanted camera connected to the server?
        if(PSlist[ui32].fd<1){
           sprintf(errmsg,"Camera %"PRIu32" is diconnected from the server",
                                                                          ui32);
           cmdstatus=CMD_ERR; goto process_select_camera;
          }
        // Looks like we're good to go, so make the connection:
        ps_Comms_fd = PSlist[ui32].fd;
        CurrPS = ui32+1;

        process_select_camera:
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");

       } else if(!strcmp(sv_triad_argv(1),"prv_hgm_scales")){
        // prv_hgm_scales <R/Y> <G> <B>
        ps_ins_reset(); // Reset the ps_Instruction struct to the unused state
        // Form the ps_Instruction (set the members we need to use)
        ui32=PS_CIID_CUSS;  ps_ins_set(PS_INS_CIID,&ui32,NULL); // Custom f.g.
        ui81=PS_CUSS_PHSSF; ps_ins_set(PS_INS_SIID,&ui81,NULL); // Hgm scales
        sscanf(sv_triad_argv(2),"%la",&doutmp); // R/Y value
        ps_ins_set(PS_INS_DVAL1,&doutmp,NULL);
        sscanf(sv_triad_argv(3),"%la",&doutmp); // G value
        ps_ins_set(PS_INS_DVAL2,&doutmp,NULL);
        sscanf(sv_triad_argv(4),"%la",&doutmp); // B value
        ps_ins_set(PS_INS_DVAL3,&doutmp,NULL);
        // ps_Instruction is now fully populated so send it to PARDcap
        cmdstatus=sv_send_pardcap_ins(PS_CIID_CUSS,errmsg,sv_triad_argv(1),1);
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");
       } else if(!strcmp(sv_triad_argv(1),"prv_hgm_satlim_u")){
        // prv_hgm_satlim_u <R/Y> <G> <B>
        ps_ins_reset(); // Reset the ps_Instruction struct to the unused state
        // Form the ps_Instruction (set the members we need to use)
        ui32=PS_CIID_CUSS; ps_ins_set(PS_INS_CIID,&ui32,NULL);// Custom f.g.
        ui81=PS_CUSS_PHSSU; ps_ins_set(PS_INS_SIID,&ui81,NULL); // Hgm sat lims U
        sscanf(sv_triad_argv(2),"%"SCNd32,&si32); // R/Y value
        ps_ins_set(PS_INS_IVAL1,&si32,NULL);
        sscanf(sv_triad_argv(3),"%"SCNd32,&si32); // G value
        ps_ins_set(PS_INS_IVAL2,&si32,NULL);
        sscanf(sv_triad_argv(4),"%"SCNd32,&si32); // B value
        ps_ins_set(PS_INS_IVAL3,&si32,NULL);
        // ps_Instruction is now fully populated so send it to PARDcap
        cmdstatus=sv_send_pardcap_ins(PS_CIID_CUSS,errmsg,sv_triad_argv(1),1);
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");
       } else if(!strcmp(sv_triad_argv(1),"prv_hgm_satlim_l")){
        // prv_hgm_satlim_l <R/Y> <G> <B>
        ps_ins_reset(); // Reset the ps_Instruction struct to the unused state
        // Form the ps_Instruction (set the members we need to use)
        ui32=PS_CIID_CUSS; ps_ins_set(PS_INS_CIID,&ui32,NULL);// Custom f.g.
        ui81=PS_CUSS_PHSSL; ps_ins_set(PS_INS_SIID,&ui81,NULL); // Hgm sat lims L
        sscanf(sv_triad_argv(2),"%"SCNd32,&si32); // R/Y value
        ps_ins_set(PS_INS_IVAL1,&si32,NULL);
        sscanf(sv_triad_argv(3),"%"SCNd32,&si32); // G value
        ps_ins_set(PS_INS_IVAL2,&si32,NULL);
        sscanf(sv_triad_argv(4),"%"SCNd32,&si32); // B value
        ps_ins_set(PS_INS_IVAL3,&si32,NULL);
        // ps_Instruction is now fully populated so send it to PARDcap
        cmdstatus=sv_send_pardcap_ins(PS_CIID_CUSS,errmsg,sv_triad_argv(1),1);
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");
       } else if(!strcmp(sv_triad_argv(1),"prv_hgm_posn")){
        // prv_hgm_posn <x> <y>
        ps_ins_reset(); // Reset the ps_Instruction struct to the unused state
        // Form the ps_Instruction (set the members we need to use)
        ui32=PS_CIID_CUSS; ps_ins_set(PS_INS_CIID,&ui32,NULL);// Custom f.g.
        ui81=PS_CUSS_PHSXY; ps_ins_set(PS_INS_SIID,&ui81,NULL); // Hgm x y posn
        sscanf(sv_triad_argv(2),"%"SCNd32,&si32); // X position
        ps_ins_set(PS_INS_IVAL1,&si32,NULL);
        sscanf(sv_triad_argv(3),"%"SCNd32,&si32); // Y position
        ps_ins_set(PS_INS_IVAL2,&si32,NULL);
        // ps_Instruction is now fully populated so send it to PARDcap
        cmdstatus=sv_send_pardcap_ins(PS_CIID_CUSS,errmsg,sv_triad_argv(1),1);
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");
       } else if(!strcmp(sv_triad_argv(1),"prv_fc_posn")){
        // Process the command with the cuss helper function
        sv_process_cuss(PS_CUSS_PFCPY,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_INT1);
       } else if(!strcmp(sv_triad_argv(1),"prv_fc_show")){
        // Process the command with the cuss helper function
        sv_process_cuss(PS_CUSS_PFCSH,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_INT1);
       } else if(!strcmp(sv_triad_argv(1),"prv_m_integral")){
        // Process the command with the cuss helper function
        sv_process_cuss(PS_CUSS_PMINT,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_INT1);
       } else if(!strcmp(sv_triad_argv(1),"prv_m_bias")){
        // Process the command with the cuss helper function
        sv_process_cuss(PS_CUSS_PMBIA,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_INT1);
       } else if(!strcmp(sv_triad_argv(1),"prv_lut")){
        // Process the command with the cuss helper function
        sv_process_cuss(PS_CUSS_PVLUT,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_BYTE);
       } else if(!strcmp(sv_triad_argv(1),"prv_size")){
        // Process the command with the cuss helper function
        sv_process_cuss(PS_CUSS_PVSZ,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_BYTE);
       } else if(!strcmp(sv_triad_argv(1),"prv_cdf_use")){
        // prv_cdf_use <ON|OFF|image_file> <prevht> <prevwd>

        // First check if this is an ON/OFF command or if the client is sending
        // an image file:
        switch(sv_switch_string(sv_triad_argv(2))){
          case SWS_ON:
          case SWS_OFF:
          // This is a simple binary request with no image blob invovled.
          // Process the command with the cuss helper function
          sv_process_cuss(PS_CUSS_PCDFC,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_ONFF);
          goto end_cdf_use;
          case SWS_PPM:
          case SWS_BMP:
          // This means client has sent a preview colour dark image sv_Blob.
          ps_ins_reset(); // Reset the ps_Instruction struct to the unused state
          // Form the ps_Instruction (set the members we need to use)
          ui32=PS_CIID_IMGS; ps_ins_set(PS_INS_CIID,&ui32,NULL); // Set an image
          ui81=PS_IMGT_DFCDP; ps_ins_set(PS_INS_SIID,&ui81,NULL);  // Prev. colour
          ui81=sv_file_to_safmt(sv_triad_argv(2)); // This will be BMP
          ps_ins_set(PS_INS_SAFMT,&ui81,NULL);  // Image format
          // Dimensions - it is assumed the pardclient has checked these have
          // have been correctly set before allowing the user to send a flat
          // field command:
          pvht = (uint32_t)atoi(sv_triad_argv(3));
          pvwd = (uint32_t)atoi(sv_triad_argv(4));
          ps_ins_set(PS_INS_IVAL1,&pvht,NULL);  // Preview height for this camera
          ps_ins_set(PS_INS_IVAL2,&pvwd,NULL);  // Preview width for this camera
          // Image file name for the blob to be saved as when it gets to pardcap
          si32=strlen(sv_triad_argv(2));
          if(ps_ins_set(PS_INS_STR,&si32,sv_triad_argv(2))){
            sprintf(errmsg,"Program error: ps_ins_set memory failed for preview"
                           "colour dark field image name");
            cmdstatus=CMD_ERR;
            break;
           }
          // Set the image blob and its length in bytes
          if(!have_blob){
             sprintf(errmsg,"No preview colour dark image sv_Blob received."); 
             cmdstatus=CMD_ERR;
             break;
            }
          si32=sv_Blobsize;
          if(ps_ins_set(PS_INS_BLOB,&si32,sv_Blob)){
            sprintf(errmsg,"Program error: ps_ins_set memory failed for preview"
                           "colour dark field image blob");
            cmdstatus=CMD_ERR;
            break;
           } 
          // ps_Instruction is now fully populated so send it to PARDcap
          cmdstatus=sv_send_pardcap_ins(PS_CIID_IMGS,errmsg,sv_triad_argv(1),1);
          break;
          default: // A programming error
             sprintf(errmsg,"Program error: Unexpected return value from"
                                                        " sv_switch_string().");
             cmdstatus=CMD_ERR;
          break;
         }

        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");
        end_cdf_use:
       } else if(!strcmp(sv_triad_argv(1),"prv_mask_use")){
        // prv_mask_use <ON|OFF|image_file>

        // First check if this is an ON/OFF command or if the client is sending
        // an image file:
        switch(sv_switch_string(sv_triad_argv(2))){
          case SWS_ON:
          case SWS_OFF:
          // This is a simple binary request with no image blob invovled.
          // Process the command with the cuss helper function
          sv_process_cuss(PS_CUSS_PMAST,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_ONFF);
          goto end_cmask_use;
          case SWS_PGM:
          case SWS_PPM:
          case SWS_BMP:
          // This means client has sent a preview mask image sv_Blob.
          ps_ins_reset(); // Reset the ps_Instruction struct to the unused state
          // Form the ps_Instruction (set the members we need to use)
          ui32=PS_CIID_IMGS; ps_ins_set(PS_INS_CIID,&ui32,NULL); // Set an image
          ui81=PS_IMGT_MASKP; ps_ins_set(PS_INS_SIID,&ui81,NULL);  // Prev. mask
          ui81=sv_file_to_safmt(sv_triad_argv(2)); // This will be BMP
          ps_ins_set(PS_INS_SAFMT,&ui81,NULL);  // Image format
          // Dimensions - it is assumed the pardclient has checked these have
          // have been correctly set before allowing the user to send a flat
          // field command:
          pvht = (uint32_t)atoi(sv_triad_argv(3));
          pvwd = (uint32_t)atoi(sv_triad_argv(4));
          ps_ins_set(PS_INS_IVAL1,&pvht,NULL);  // Preview height for this camera
          ps_ins_set(PS_INS_IVAL2,&pvwd,NULL);  // Preview width for this camera
          // Image file name for the blob to be saved as when it gets to pardcap
          si32=strlen(sv_triad_argv(2));
          if(ps_ins_set(PS_INS_STR,&si32,sv_triad_argv(2))){
            sprintf(errmsg,"Program error: ps_ins_set memory failed for preview"
                           "mask image name");
            cmdstatus=CMD_ERR;
            break;
           }
          // Set the image blob and its length in bytes
          if(!have_blob){
             sprintf(errmsg,"No preview mask image sv_Blob received."); 
             cmdstatus=CMD_ERR;
             break;
            }
          si32=sv_Blobsize;
          if(ps_ins_set(PS_INS_BLOB,&si32,sv_Blob)){
            sprintf(errmsg,"Program error: ps_ins_set memory failed for preview"
                           "mask image blob");
            cmdstatus=CMD_ERR;
            break;
           } 
          // ps_Instruction is now fully populated so send it to PARDcap
          cmdstatus=sv_send_pardcap_ins(PS_CIID_IMGS,errmsg,sv_triad_argv(1),1);
          break;
          default: // A programming error
             sprintf(errmsg,"Program error: Unexpected return value from"
                                                        " sv_switch_string().");
             cmdstatus=CMD_ERR;
          break;
         }

        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");
        end_cmask_use:
       } else if(!strcmp(sv_triad_argv(1),"prv_mdf_set")){
        // prv_mdf_set <image_file>
        switch(sv_switch_string(sv_triad_argv(2))){
          case SWS_DOU:
          ps_ins_reset(); // Reset the ps_Instruction struct to the unused state
          // Form the ps_Instruction (set the members we need to use)
          ui32=PS_CIID_IMGS; ps_ins_set(PS_INS_CIID,&ui32,NULL);// Set an image
          ui81=PS_IMGT_DFIMP; ps_ins_set(PS_INS_SIID,&ui81,NULL); // PM Dark field
          ui81=sv_file_to_safmt(sv_triad_argv(2));
          ps_ins_set(PS_INS_SAFMT,&ui81,NULL);  // Image format
          // Dimensions - it is assumed the pardclient has checked these have
          // have been correctly set before allowing the user to send a dark
          // field command:
          pvht = (uint32_t)atoi(sv_triad_argv(3));
          pvwd = (uint32_t)atoi(sv_triad_argv(4));
          ps_ins_set(PS_INS_IVAL1,&pvht,NULL);  // Preview height for this camera
          ps_ins_set(PS_INS_IVAL2,&pvwd,NULL);  // Preview width for this camera
          // Image file name for the blob to be saved as when it gets to pardcap
          si32=strlen(sv_triad_argv(2));
          if(ps_ins_set(PS_INS_STR,&si32,sv_triad_argv(2))){
             sprintf(errmsg,"Program error: ps_ins_set memory failed for dark"
                            "field (preview mono) image name");
             cmdstatus=CMD_ERR;
             break;
            }
          // Set the image blob and its length in bytes
          if(!have_blob){
             sprintf(errmsg,"No dark field image sv_Blob received."); 
             cmdstatus=CMD_ERR;
             break;
            }
          si32=sv_Blobsize;
          if(ps_ins_set(PS_INS_BLOB,&si32,sv_Blob)){
             sprintf(errmsg,"Program error: ps_ins_set memory failed for dark"
                            "field (preview mono) image blob");
             cmdstatus=CMD_ERR;
             break;
            } 
          // ps_Instruction is now fully populated so send it to PARDcap
          cmdstatus=sv_send_pardcap_ins(PS_CIID_IMGS,errmsg,sv_triad_argv(1),1);
          break;
          default: // A programming error
             sprintf(errmsg,"Program error: Unexpected return value from"
                                                        " sv_switch_string().");
             cmdstatus=CMD_ERR;
          break;
         }

        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");
       } else if(!strcmp(sv_triad_argv(1),"prv_mff_set")){
        // prv_mff_set <image_file>
        switch(sv_switch_string(sv_triad_argv(2))){
          case SWS_DOU:
          ps_ins_reset(); // Reset the ps_Instruction struct to the unused state
          // Form the ps_Instruction (set the members we need to use)
          ui32=PS_CIID_IMGS; ps_ins_set(PS_INS_CIID,&ui32,NULL);// Set an image
          ui81=PS_IMGT_FFIMP; ps_ins_set(PS_INS_SIID,&ui81,NULL); // PM Flat field
          ui81=sv_file_to_safmt(sv_triad_argv(2));
          ps_ins_set(PS_INS_SAFMT,&ui81,NULL);  // Image format
          // Dimensions - it is assumed the pardclient has checked these have
          // have been correctly set before allowing the user to send a dark
          // field command:
          pvht = (uint32_t)atoi(sv_triad_argv(3));
          pvwd = (uint32_t)atoi(sv_triad_argv(4));
          ps_ins_set(PS_INS_IVAL1,&pvht,NULL);  // Preview height for this camera
          ps_ins_set(PS_INS_IVAL2,&pvwd,NULL);  // Preview width for this camera
          // Image file name for the blob to be saved as when it gets to pardcap
          si32=strlen(sv_triad_argv(2));
          if(ps_ins_set(PS_INS_STR,&si32,sv_triad_argv(2))){
             sprintf(errmsg,"Program error: ps_ins_set memory failed for flat"
                            "field (preview mono) image name");
             cmdstatus=CMD_ERR;
             break;
            }
          // Set the image blob and its length in bytes
          if(!have_blob){
             sprintf(errmsg,"No dark field image sv_Blob received."); 
             cmdstatus=CMD_ERR;
             break;
            }
          si32=sv_Blobsize;
          if(ps_ins_set(PS_INS_BLOB,&si32,sv_Blob)){
             sprintf(errmsg,"Program error: ps_ins_set memory failed for flat"
                            "field (preview mono) image blob");
             cmdstatus=CMD_ERR;
             break;
            } 
          // ps_Instruction is now fully populated so send it to PARDcap
          cmdstatus=sv_send_pardcap_ins(PS_CIID_IMGS,errmsg,sv_triad_argv(1),1);
          break;
          default: // A programming error
             sprintf(errmsg,"Program error: Unexpected return value from"
                                                        " sv_switch_string().");
             cmdstatus=CMD_ERR;
          break;
         }

        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");
       } else if(!strcmp(sv_triad_argv(1),"prv_mcor_eject")){
        // Process the command with the cuss helper function
        cmdstatus= sv_process_cuss(PS_CUSS_PMCEJ,sv_triad_argv(1),
               NULL,errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_NONE);
       } else if(!strcmp(sv_triad_argv(1),"prv_toggle")){
        // Set one of the preview on/off options.
        // Set the PS_CUSS_ value according to the option:
                if(!strcmp(sv_triad_argv(2),"mono")){
          ptoggopt=PS_CUSS_PMONO;
         } else if(!strcmp(sv_triad_argv(2),"invert")){
          ptoggopt=PS_CUSS_PINVT;
         } else if(!strcmp(sv_triad_argv(2),"flip_h")){
          ptoggopt=PS_CUSS_PFLPH;
         } else if(!strcmp(sv_triad_argv(2),"flip_v")){
          ptoggopt=PS_CUSS_PFLPV;
         } else if(!strcmp(sv_triad_argv(2),"laplacian")){
          ptoggopt=PS_CUSS_PLAPL;
         } else if(!strcmp(sv_triad_argv(2),"premedian")){
          ptoggopt=PS_CUSS_PREMDN;
         } else if(!strcmp(sv_triad_argv(2),"postmedian")){
          ptoggopt=PS_CUSS_POSTMDN;
         } else if(!strcmp(sv_triad_argv(2),"dctpostlut")){
          ptoggopt=PS_CUSS_DCTAL;
         } else if(!strcmp(sv_triad_argv(2),"fc_dct")){
          ptoggopt=PS_CUSS_PFDCT;
         } else if(!strcmp(sv_triad_argv(2),"fc_mal")){
          ptoggopt=PS_CUSS_PFMAL;
         } else if(!strcmp(sv_triad_argv(2),"fc_vmul")){
          ptoggopt=PS_CUSS_PFVAR;
         } else if(!strcmp(sv_triad_argv(2),"mask_show")){
          ptoggopt=PS_CUSS_PMASH;
         } else if(!strcmp(sv_triad_argv(2),"hgm_show")){
          ptoggopt=PS_CUSS_PHSSH;
         } else if(!strcmp(sv_triad_argv(2),"hgm_cum_r")){
          ptoggopt=PS_CUSS_PHSCR;
         } else if(!strcmp(sv_triad_argv(2),"hgm_cum_g")){
          ptoggopt=PS_CUSS_PHSCG;
         } else if(!strcmp(sv_triad_argv(2),"hgm_cum_b")){
          ptoggopt=PS_CUSS_PHSCB;
         } else if(!strcmp(sv_triad_argv(2),"hgm_nofit")){
          ptoggopt=PS_CUSS_PHSDF;
         } else if(!strcmp(sv_triad_argv(2),"hgm_restrictvar")){
          ptoggopt=PS_CUSS_PHSVM;
         } else {
          // Error, unknown toggle option
           sprintf(errmsg,"[PARDUS]: Unknown prv_toggle option");
           sprintf(cmdtriad,"ERROR FailedCmd Ready");
           sending_bloberr=1;
           goto process_prvtoggle_result;
         }
        // Now rocess the command with the cuss helper function
        sv_process_cuss(ptoggopt,sv_triad_argv(2),
               sv_triad_argv(3),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_ONFF);
        process_prvtoggle_result:
        ;
       } else if(!strcmp(sv_triad_argv(1),"save_format")){
        // Process the command with the cuss helper function
        sv_process_cuss(PS_CUSS_SAFMT,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_FRMT);
       } else if(!strcmp(sv_triad_argv(1),"save_doubles")){
        // save_doubles <OFF|ON|FITS>
        // Process the command with the cuss helper function
        sv_process_cuss(PS_CUSS_SADOU,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_ONFT);
       } else if(!strcmp(sv_triad_argv(1),"frame_number")){
        // Process the command with the cuss helper function
        cmdstatus=sv_process_cuss(PS_CUSS_FRMNO,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_INT1);
        if(cmdstatus==CMD_OK) local_frame_number=atoi(sv_triad_argv(2));

       } else if(!strcmp(sv_triad_argv(1),"frames_to_average")){
        // Process the command with the cuss helper function
        sv_process_cuss(PS_CUSS_FTOAV,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_INT1);

       } else if(!strcmp(sv_triad_argv(1),"frames_to_skip")){
        // Process the command with the cuss helper function
        sv_process_cuss(PS_CUSS_FTOSK,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_INT1);

       } else if(!strcmp(sv_triad_argv(1),"skip_frames")){
        // Process the command with the cuss helper function
        sv_process_cuss(PS_CUSS_SKFRM,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_INT1);

       } else if(!strcmp(sv_triad_argv(1),"grabber_timeout")){
        // Process the command with the cuss helper function
        sv_process_cuss(PS_CUSS_GBTMO,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_INT1);

       } else if(!strcmp(sv_triad_argv(1),"grabber_predelay")){
        // Process the command with the cuss helper function
        sv_process_cuss(PS_CUSS_GBDLY,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_INT1);

       } else if(!strcmp(sv_triad_argv(1),"grabber_retries")){
        // Process the command with the cuss helper function
        sv_process_cuss(PS_CUSS_GBRTY,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_INT1);

       } else if(!strcmp(sv_triad_argv(1),"average_scale_means")){
        // Process the command with the cuss helper function
        sv_process_cuss(PS_CUSS_AVSCA,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_ONFF);

       } else if(!strcmp(sv_triad_argv(1),"camera_get")){
        // Set the required fields of the ps_Instruction to send:
        // Reset the ps_Instruction struct to the unused state
        ps_ins_reset();
        // Form the ps_Instruction (set the members we need to use)
        ui32=PS_CIID_CAMG;  ps_ins_set(PS_INS_CIID,&ui32,NULL);
        // Camera setting ID number
        sscanf(sv_triad_argv(2),"%"SCNd32,&si32);
        ps_ins_set(PS_INS_IVAL1,&si32,NULL);
        cmdstatus=sv_send_pardcap_ins(PS_CIID_CAMG,errmsg,sv_triad_argv(1),1);

        if(cmdstatus!=CMD_ERR){
           // Does the blob have CAMGETBYTES bytes (as it should)?
           if(ps_Instruction.blen!=CAMGETBYTES){
             sprintf(errmsg,"Received blob is the wrong size (%"PRId32
                             ") for a camera get package",ps_Instruction.blen);
             cmdstatus=CMD_ERR; goto process_cam_get_triad;
            }    
           if(sv_blob_resize(ps_Instruction.blen)){
             sprintf(errmsg,"sv_blob resize failed for camera get retrieval");
             cmdstatus=CMD_ERR; goto process_cam_get_triad;
            }
           memcpy(sv_Blob,ps_Instruction.blob,ps_Instruction.blen);
           sending_blob=1;
         }
        process_cam_get_triad:
        if(cmdstatus==CMD_ERR){ 
           sprintf(cmdtriad,"ERROR FailedCmd Ready");
           sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");
        

       } else if(!strcmp(sv_triad_argv(1),"camera_set")){
        // Set the required fields of the ps_Instruction to send:
        // Reset the ps_Instruction struct to the unused state
        ps_ins_reset();
        // Form the ps_Instruction (set the members we need to use)
        ui32=PS_CIID_CAMS;  ps_ins_set(PS_INS_CIID,&ui32,NULL);
        // Camera setting ID number
        sscanf(sv_triad_argv(2),"%"SCNd32,&si32);
        ps_ins_set(PS_INS_IVAL1,&si32,NULL);
        // Value to set it so
        sscanf(sv_triad_argv(3),"%"SCNd32,&si32);
        ps_ins_set(PS_INS_IVAL2,&si32,NULL);
        cmdstatus=sv_send_pardcap_ins(PS_CIID_CAMS,errmsg,sv_triad_argv(1),1);
        if(cmdstatus==CMD_ERR){ 
           sprintf(cmdtriad,"ERROR FailedCmd Ready");
           sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");

       } else if(!strcmp(sv_triad_argv(1),"save_resolution")){
         ps_ins_reset(); // Reset the ps_Instruction struct to the unused state
         // Form the ps_Instruction (set the members we need to use)
         ui32=PS_CIID_SRES;  ps_ins_set(PS_INS_CIID,&ui32,NULL);
         // Height into ival1
         sscanf(sv_triad_argv(3),"%"SCNd32,&si32);  
         ps_ins_set(PS_INS_IVAL1,&si32,NULL);
         ImHeight=si32; // Needed for commands like ff_correction image setting
         // Width into ival2
         sscanf(sv_triad_argv(2),"%"SCNd32,&si32);  
         ps_ins_set(PS_INS_IVAL2,&si32,NULL);
         ImWidth=si32; // Needed for commands like ff_correction image setting
        // Send the command and process the result
        cmdstatus=sv_send_pardcap_ins(PS_CIID_SRES,errmsg,sv_triad_argv(1),1);
        if(cmdstatus==CMD_ERR){ 
           sprintf(cmdtriad,"ERROR FailedCmd Ready");
           sending_bloberr=1;
           // Unset the local image dimensions variables:
           ImHeight=0; ImWidth=0;
         } else sprintf(cmdtriad,"OK Done Ready");

       } else if(!strcmp(sv_triad_argv(1),"ff_correction")){
        // ff_correction <ON|OFF|image_file>

        // First check if this is an ON/OFF command or if the client is sending
        // an image file:
        switch(sv_switch_string(sv_triad_argv(2))){
          case SWS_ON:
          case SWS_OFF:
          // This is a simple binary request with no image blob invovled.
          // Process the command with the cuss helper function
          sv_process_cuss(PS_CUSS_FFCOR,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_ONFF);
          goto end_ff_correction;
          case SWS_PGM:
          case SWS_PPM:
          case SWS_BMP:
          case SWS_DOU:
          case SWS_DO3:
          // This means client has sent a flat field image sv_Blob.
          ps_ins_reset(); // Reset the ps_Instruction struct to the unused state
          // Form the ps_Instruction (set the members we need to use)
          ui32=PS_CIID_IMGS; ps_ins_set(PS_INS_CIID,&ui32,NULL); // Set an image
          ui81=PS_IMGT_FFIMF; ps_ins_set(PS_INS_SIID,&ui81,NULL);  // Flat field
          ui81=sv_file_to_safmt(sv_triad_argv(2));
          ps_ins_set(PS_INS_SAFMT,&ui81,NULL);  // Image format
          // Dimensions - it is assumed the pardclient has checked these have
          // have been correctly set before allowing the user to send a flat
          // field command:
          ps_ins_set(PS_INS_IVAL1,&ImHeight,NULL);  // Image height
          ps_ins_set(PS_INS_IVAL2,&ImWidth,NULL);   // Image width
          // Image file name for the blob to be saved as when it gets to pardcap
          si32=strlen(sv_triad_argv(2));
          if(ps_ins_set(PS_INS_STR,&si32,sv_triad_argv(2))){
             sprintf(errmsg,"Program error: ps_ins_set memory failed for flat"
                            "field image name");
             cmdstatus=CMD_ERR;
             break;
            }
          // Set the image blob and its length in bytes
          if(!have_blob){
             sprintf(errmsg,"No flat field image sv_Blob received."); 
             cmdstatus=CMD_ERR;
             break;
            }
          si32=sv_Blobsize;
          if(ps_ins_set(PS_INS_BLOB,&si32,sv_Blob)){
             sprintf(errmsg,"Program error: ps_ins_set memory failed for flat"
                            "field image blob");
             cmdstatus=CMD_ERR;
             break;
            } 
          // ps_Instruction is now fully populated so send it to PARDcap
          cmdstatus=sv_send_pardcap_ins(PS_CIID_IMGS,errmsg,sv_triad_argv(1),1);
          break;
          default: // A programming error
             sprintf(errmsg,"Program error: Unexpected return value from"
                                                        " sv_switch_string().");
             cmdstatus=CMD_ERR;
          break;
         }

        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");
        end_ff_correction:
       } else if(!strcmp(sv_triad_argv(1),"df_correction")){
        // df_correction <ON|OFF|image_file>

        // First check if this is an ON/OFF command or if the client is sending
        // an image file:
        switch(sv_switch_string(sv_triad_argv(2))){
          case SWS_ON:
          case SWS_OFF:
          // This is a simple binary request with no image blob invovled.
          // Process the command with the cuss helper function
          sv_process_cuss(PS_CUSS_DFCOR,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_ONFF);
          goto end_df_correction;
          case SWS_DOU:
          case SWS_DO3:
          // This means client has sent a dark field image sv_Blob.
          ps_ins_reset(); // Reset the ps_Instruction struct to the unused state
          // Form the ps_Instruction (set the members we need to use)
          ui32=PS_CIID_IMGS; ps_ins_set(PS_INS_CIID,&ui32,NULL); // Set an image
          ui81=PS_IMGT_DFIMF; ps_ins_set(PS_INS_SIID,&ui81,NULL);  // Dark field
          ui81=sv_file_to_safmt(sv_triad_argv(2));
          ps_ins_set(PS_INS_SAFMT,&ui81,NULL);  // Image format
          // Dimensions - it is assumed the pardclient has checked these have
          // have been correctly set before allowing the user to send a dark
          // field command:
          ps_ins_set(PS_INS_IVAL1,&ImHeight,NULL);  // Image height
          ps_ins_set(PS_INS_IVAL2,&ImWidth,NULL);   // Image width
          // Image file name for the blob to be saved as when it gets to pardcap
          si32=strlen(sv_triad_argv(2));
          if(ps_ins_set(PS_INS_STR,&si32,sv_triad_argv(2))){
             sprintf(errmsg,"Program error: ps_ins_set memory failed for dark"
                            "field image name");
             cmdstatus=CMD_ERR;
             break;
            }
          // Set the image blob and its length in bytes
          if(!have_blob){
             sprintf(errmsg,"No dark field image sv_Blob received."); 
             cmdstatus=CMD_ERR;
             break;
            }
          si32=sv_Blobsize;
          if(ps_ins_set(PS_INS_BLOB,&si32,sv_Blob)){
             sprintf(errmsg,"Program error: ps_ins_set memory failed for dark"
                            "field image blob");
             cmdstatus=CMD_ERR;
             break;
            } 
          // ps_Instruction is now fully populated so send it to PARDcap
          cmdstatus=sv_send_pardcap_ins(PS_CIID_IMGS,errmsg,sv_triad_argv(1),1);
          break;
          default: // A programming error
             sprintf(errmsg,"Program error: Unexpected return value from"
                                                        " sv_switch_string().");
             cmdstatus=CMD_ERR;
          break;
         }

        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");
        end_df_correction:
       } else if(!strcmp(sv_triad_argv(1),"corrections_mask")){
        // corrections_mask <ON|OFF|image_file>

        // First check if this is an ON/OFF command or if the client is sending
        // an image file:
        switch(sv_switch_string(sv_triad_argv(2))){
          case SWS_ON:
          case SWS_OFF:
          // This is a simple binary request with no image blob invovled.
          // Process the command with the cuss helper function
          sv_process_cuss(PS_CUSS_MACOR,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_ONFF);
          goto end_mask_correction;
          case SWS_PGM:
          case SWS_PPM:
          case SWS_BMP:
          // This means client has sent a mask image sv_Blob.
          ps_ins_reset(); // Reset the ps_Instruction struct to the unused state
          // Form the ps_Instruction (set the members we need to use)
          ui32=PS_CIID_IMGS; ps_ins_set(PS_INS_CIID,&ui32,NULL); // Set an image
          ui81=PS_IMGT_MASKF; ps_ins_set(PS_INS_SIID,&ui81,NULL);  // Mask (full)
          ui81=sv_file_to_safmt(sv_triad_argv(2));
          ps_ins_set(PS_INS_SAFMT,&ui81,NULL);  // Image format
          // Dimensions - it is assumed the pardclient has checked these have
          // have been correctly set before allowing the user to send a mask
          // command:
          ps_ins_set(PS_INS_IVAL1,&ImHeight,NULL);  // Image height
          ps_ins_set(PS_INS_IVAL2,&ImWidth,NULL);   // Image width
          // Image file name for the blob to be saved as when it gets to pardcap
          si32=strlen(sv_triad_argv(2));
          if(ps_ins_set(PS_INS_STR,&si32,sv_triad_argv(2))){
             sprintf(errmsg,"Program error: ps_ins_set memory failed for a mask"
                            "image name");
             cmdstatus=CMD_ERR;
             break;
            }
          // Set the image blob and its length in bytes
          if(!have_blob){
             sprintf(errmsg,"No mask image sv_Blob received."); 
             cmdstatus=CMD_ERR;
             break;
            }
          si32=sv_Blobsize;
          if(ps_ins_set(PS_INS_BLOB,&si32,sv_Blob)){
             sprintf(errmsg,"Program error: ps_ins_set memory failed for a mask"
                            "(full size) image blob");
             cmdstatus=CMD_ERR;
             break;
            } 
          // ps_Instruction is now fully populated so send it to PARDcap
          cmdstatus=sv_send_pardcap_ins(PS_CIID_IMGS,errmsg,sv_triad_argv(1),1);
          break;
          default: // A programming error
             sprintf(errmsg,"Program error: Unexpected return value from"
                                                        " sv_switch_string().");
             cmdstatus=CMD_ERR;
          break;
         }

        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");
        end_mask_correction:
       } else if(!strcmp(sv_triad_argv(1),"grab_image")){
        char imtypestr[10],fmtstring[5];
        int32_t expsz;

        // Set the required fields of the ps_Instruction to send:
        // Reset the ps_Instruction struct to the unused state
        ps_ins_reset();
        // Form the ps_Instruction (set the members we need to use)
        ui32=PS_CIID_IMGR;  ps_ins_set(PS_INS_CIID,&ui32,NULL);
        ui81=PS_IMGT_MAINF;  ps_ins_set(PS_INS_SIID,&ui81,NULL);
        cmdstatus=sv_send_pardcap_ins(PS_CIID_IMGR,errmsg,sv_triad_argv(1),1);
        if(cmdstatus==CMD_ERR) goto process_grab_image_triad;

        // If the save as raw doubles or fits option is selected the the return
        // value in ps_Instruction.safmt may be SAF_DOU or SAF_DO3 in addition
        // to the standard format options.
        sv_fmtstr(ps_Instruction.safmt,fmtstring);

        // Check if the blob is the correct expected size then copy it over to
        // to the sv_Blob for sending over to the client.
        // The expected size depends on the format and dimensions. For jpg
        // compressed stream there is no way to calulate the expected size here
        // so just trust whatever is sent:
        expsz=ps_Instruction.ival1 * ps_Instruction.ival2;
        switch(ps_Instruction.safmt){
          case SAF_YUYV: expsz*=2; break;
          case SAF_YP5: break;
          case SAF_BM8: break;
          case SAF_RGB:
          case SAF_BMP:
          case SAF_PNG: expsz*=3; break;
          case SAF_DOU: 
          case SAF_INT: expsz*=sizeof(double); break;
          case SAF_DO3: expsz*=(3*sizeof(double)); break;
          case SAF_JPG: expsz=ps_Instruction.blen; break; // Trust blen
          default: // Programming error
            sprintf(errmsg,"Image format not recognised (program bug).");
            cmdstatus=CMD_ERR; goto process_grab_image_triad;
          }
        // Does the blob have the expected size?
        if(ps_Instruction.blen!=expsz){
          sprintf(errmsg,"Image blob wrong size for its format and dims");
          cmdstatus=CMD_ERR; goto process_grab_image_triad;
         }    
        if(sv_blob_resize(ps_Instruction.blen)){
          sprintf(errmsg,"Image sv_blob resize failed for full image");
          cmdstatus=CMD_ERR; goto process_grab_image_triad;
         }
        memcpy(sv_Blob,ps_Instruction.blob,ps_Instruction.blen);
        sending_blob=1;


        // I need to check ps_Instruction.err[0]  to set imtypestr as either
        // Full_OK (if err[0]==PS_CB_OK), Full_NB (if err[0]==PS_CB_E_NBC) or
        // Full_NT (if err[0]==PS_CB_E_NTC) - if it were anything else we would
        // not get to here because the command would have returned with an error
        // and if(cmdstatus==CMD_ERR) goto process_grab_image_triad; activated.
        switch(ps_Instruction.err[0]){
          case PS_CB_OK:    sprintf(imtypestr,"Full_OK"); break;
          case PS_CB_E_NTC: sprintf(imtypestr,"Full_NT"); break;
          case PS_CB_E_NBC: sprintf(imtypestr,"Full_NB"); break;
          default: // Programming error
            sprintf(errmsg,"Image type not recognised (program bug).");
            cmdstatus=CMD_ERR; goto process_grab_image_triad;
          break; 
         }

        // ImRoot is ignored by the current PARDclient. I am not sure why I 
        // implemented it / return it here (the pardserver does not contain a
        // 'image_root' command because it is set and generated at the client
        // end) other than this string is required to maintain standard command
        // argument numbers. I can just return the string constant "ImRoot" for
        // now. 


        process_grab_image_triad:
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Image %s %d %d %s %d %s Ready",
                                    imtypestr,
                                    ps_Instruction.ival2, // Image width
                                    ps_Instruction.ival1, // Image height
                                    fmtstring,
                                    local_frame_number++,
                                    "ImRoot");

       } else if(!strcmp(sv_triad_argv(1),"yuyv_gain")){
        // Process the command with the cuss helper function
        cmdstatus=sv_process_cuss(PS_CUSS_GAIYU,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_DOU1);

       } else if(!strcmp(sv_triad_argv(1),"yuyv_bias")){
        // Process the command with the cuss helper function
        cmdstatus=sv_process_cuss(PS_CUSS_BIAYU,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_DOU1);

       } else if(!strcmp(sv_triad_argv(1),"get_preview")){
        // Set the required fields of the ps_Instruction to send:
        // Reset the ps_Instruction struct to the unused state
        ps_ins_reset();
        // Form the ps_Instruction (set the members we need to use)
        ui32=PS_CIID_IMGR;  ps_ins_set(PS_INS_CIID,&ui32,NULL);
        ui81=PS_IMGT_PREVW;  ps_ins_set(PS_INS_SIID,&ui81,NULL);
        si32=sv_evalonoff(sv_triad_argv(2));
        if(si32<0){
          sprintf(errmsg,"<zoomed> argument not resolved as on or off");
          cmdstatus=CMD_ERR;
          goto process_get_preview_triad;
         } else ui81=(si32)?SAF_PZO:SAF_PUZ;
        ps_ins_set(PS_INS_SAFMT,&ui81,NULL);

        sscanf(sv_triad_argv(3),"%"SCNd32,&si32);
        ps_ins_set(PS_INS_IVAL1,&si32,NULL);
        sscanf(sv_triad_argv(4),"%"SCNd32,&si32);
        ps_ins_set(PS_INS_IVAL2,&si32,NULL);
        cmdstatus=sv_send_pardcap_ins(PS_CIID_IMGR,errmsg,sv_triad_argv(1),1);
        if(cmdstatus==CMD_ERR) goto process_get_preview_triad;

        // Does the blob have at least 160x120x3=57600 bytes (as it should)?
        if(ps_Instruction.blen<57600){
          sprintf(errmsg,"Image blob too small for a preview image");
          cmdstatus=CMD_ERR; goto process_get_preview_triad;
         }    
        if(sv_blob_resize(ps_Instruction.blen)){
          sprintf(errmsg,"Image sv_blob resize failed for preview image");
          cmdstatus=CMD_ERR; goto process_get_preview_triad;
         }
        memcpy(sv_Blob,ps_Instruction.blob,ps_Instruction.blen);
        sending_blob=1;
        process_get_preview_triad:
        if(cmdstatus==CMD_ERR){ 
           // Append the actual error codes if this was a pardcapture error so
           // the client can decide whether it is fatal or not and how to set
           // the PreviewReport system variable:
           if (strncmp(errmsg, "[PARDCAP]", 9) == 0) {
             // 1. Calculate the current filled length of the message buffer
             size_t current_len = strlen(errmsg);
             
             // 2. Only attempt to append if we haven't already overflowed it
             if (current_len < ERRMSG_LEN) {
                 // snprintf dynamically calculates the remaining space 
                 // (ERRMSG_LEN - current_len) and will truncate if it runs out
                 // of room.
                 snprintf(errmsg + current_len, ERRMSG_LEN - current_len, 
                          " %"PRId32" %"PRId32" ", 
                          ps_Instruction.err[0], ps_Instruction.err[1]);
             }
           }

           sprintf(cmdtriad,"ERROR FailedCmd Ready");
           sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Image Preview %d %d ppm 0 0 Ready",
                                                           PreviewWd,PreviewHt);
       } else if(!strcmp(sv_triad_argv(1),"get_preview_stats")){
        // Process the command with the cuss helper function
        cmdstatus= sv_process_cuss(PS_CUSS_GPVST,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_ONFF);

        if(cmdstatus!=CMD_ERR){
        // Does the blob have STATSBYTES bytes (as it should)?
        if(ps_Instruction.blen!=STATSBYTES){
          sprintf(errmsg,"Received blob is the wrong size (%"PRId32
                                   ") for a stats package",ps_Instruction.blen);
          cmdstatus=CMD_ERR; goto process_get_stats_triad;
         }    
        if(sv_blob_resize(ps_Instruction.blen)){
          sprintf(errmsg,"sv_blob resize failed for stats retrieval");
          cmdstatus=CMD_ERR; goto process_get_stats_triad;
         }
        memcpy(sv_Blob,ps_Instruction.blob,ps_Instruction.blen);
        sending_blob=1;
        process_get_stats_triad:
        if(cmdstatus==CMD_ERR){ 
           sprintf(cmdtriad,"ERROR FailedCmd Ready");
           sending_bloberr=1;
         }
        }
       } else if(!strcmp(sv_triad_argv(1),"get_preview_focus")){
        // Process the command with the cuss helper function
        cmdstatus= sv_process_cuss(PS_CUSS_GPFOC,sv_triad_argv(1),
               NULL,errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_NONE);

        if(cmdstatus!=CMD_ERR){
        // Does the blob have FOCUSBYTES bytes (as it should)?
        if(ps_Instruction.blen!=FOCUSBYTES){
          sprintf(errmsg,"Received blob is the wrong size (%"PRId32
                          ") for a focus package",ps_Instruction.blen);
          cmdstatus=CMD_ERR; goto process_get_focus_triad;
         }    
        if(sv_blob_resize(ps_Instruction.blen)){
          sprintf(errmsg,"sv_blob resize failed for focus retrieval");
          cmdstatus=CMD_ERR; goto process_get_focus_triad;
         }
        memcpy(sv_Blob,ps_Instruction.blob,ps_Instruction.blen);
        sending_blob=1;
        process_get_focus_triad:
        if(cmdstatus==CMD_ERR){ 
           sprintf(cmdtriad,"ERROR FailedCmd Ready");
           sending_bloberr=1;
         }
        }
       } else if(!strcmp(sv_triad_argv(1),"get_preview_tc")){
        // First check if this is a reference call or a measurement call
        si32=sv_evalonoff(sv_triad_argv(2));
        if(si32<0){
          sprintf(errmsg,"argument not resolved as on or off");
          cmdstatus=CMD_ERR;
          goto process_get_tc_triad;
         } 
        // Process the command with the cuss helper function
        cmdstatus= sv_process_cuss(PS_CUSS_GTCVL,sv_triad_argv(1),
               sv_triad_argv(2),errmsg,cmdtriad,&sending_bloberr,CUSS_VAL_ONFF);

        if(cmdstatus!=CMD_ERR){
        // Does the blob have TCBYTES bytes (as it should)?
        if(ps_Instruction.blen!=TCBYTES){
          sprintf(errmsg,"Received blob is the wrong size (%"PRId32
                          ") for a tissue content package",ps_Instruction.blen);
          cmdstatus=CMD_ERR; goto process_get_tc_triad;
         }    
        if(sv_blob_resize(ps_Instruction.blen)){
          sprintf(errmsg,"sv_blob resize failed for tc retrieval");
          cmdstatus=CMD_ERR; goto process_get_tc_triad;
         }
        memcpy(sv_Blob,ps_Instruction.blob,ps_Instruction.blen);
        sending_blob=1;
        // If this was a reference call, set the autofocus 'tissue content
        // reference has been set' flag
        if(si32==0) AF_ref_set = 1;

        process_get_tc_triad:
        if(cmdstatus==CMD_ERR){ 
           sprintf(cmdtriad,"ERROR FailedCmd Ready");
           sending_bloberr=1;
         }
        }
       } else if(!strcmp(sv_triad_argv(1),"tc_threshold")){
        // tc_threshold <lower> <upper>
        ps_ins_reset(); // Reset the ps_Instruction struct to the unused state
        // Form the ps_Instruction (set the members we need to use)
        ui32=PS_CIID_CUSS; ps_ins_set(PS_INS_CIID,&ui32,NULL);  // Custom f.g.
        ui81=PS_CUSS_TCTHR; ps_ins_set(PS_INS_SIID,&ui81,NULL); // tc thresholds
        sscanf(sv_triad_argv(2),"%la",&userval1); // Lower value
        ps_ins_set(PS_INS_DVAL1,&userval1,NULL);
        sscanf(sv_triad_argv(3),"%la",&userval2); // Upper value
        ps_ins_set(PS_INS_DVAL2,&userval2,NULL);
        // ps_Instruction is now fully populated so send it to PARDcap
        cmdstatus=sv_send_pardcap_ins(PS_CIID_CUSS,errmsg,sv_triad_argv(1),1);
        if(cmdstatus==CMD_ERR){
          sprintf(cmdtriad,"ERROR FailedCmd Ready");
          sending_bloberr=1;
         } else {
          sprintf(cmdtriad,"OK Done Ready");
          // Update the values used for autofocus
          AF_tc_threshlow=userval1;  // Tissue content lower threshold value
          AF_tc_threshupp=userval2;  // Tissue content upper threshold value
         }
       } else if(!strcmp(sv_triad_argv(1),"get_resolutions")){
        size_t listsz;
        cmdstatus=sv_send_pardcap_ins(PS_CIID_LRES,errmsg,sv_triad_argv(1),0);
        if(cmdstatus==CMD_ERR) goto process_get_resolutions_triad;
                                            
        // If we get to here then it looks like the correct command was executed
        // correctly by PARDcap - so get the results back to the client:
        listsz=ps_Instruction.slen+1;
        if(sv_blob_resize(listsz)){
         sprintf(errmsg,"Failed to resize sv_blob"); 
         cmdstatus=CMD_ERR; goto process_get_resolutions_triad; 
         }
        if(listsz>1){
          memcpy(sv_Blob,ps_Instruction.str,listsz);
          sending_blob=1;
          show_message((char *)ps_Instruction.str,"Res list: ",MT_INFO,0);
         } else {
          sprintf(errmsg,"Resolution list size too small");
          cmdstatus=CMD_ERR;
         }
        process_get_resolutions_triad:
        if(cmdstatus==CMD_ERR){ 
           sprintf(cmdtriad,"ERROR FailedCmd Ready");
           sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");
       } else if(!strcmp(sv_triad_argv(1),"get_actuators")){
        // Calculate the size of the required blob
        // For each actuator we need (bytes):
        // 5     For the 'N' values
        // For each member of the 'N' values we need:
        // 1     For the index
        // 32    For the name
        // Thus:
        lenblob = 5 + 33*(NSteppers+NBinaries+NMlevels+NServos+NSensors);

        // Set the blob to hold that many bytes
        if(sv_blob_resize(lenblob)){
          sprintf(errmsg,"sv_blob resize failed for get_actuators");
          cmdstatus=CMD_ERR; goto process_get_stats_triad;
         }
        // Put the current values of the numbers of actuators into the blob
        ui32=0; // We'll use this as the blob position index
        ((uint8_t *)sv_Blob)[ui32]=(uint8_t)NSteppers; ui32++;
        ((uint8_t *)sv_Blob)[ui32]=(uint8_t)NBinaries; ui32++;
        ((uint8_t *)sv_Blob)[ui32]=(uint8_t)NMlevels;  ui32++;
        ((uint8_t *)sv_Blob)[ui32]=(uint8_t)NServos;   ui32++;
        ((uint8_t *)sv_Blob)[ui32]=(uint8_t)NSensors;  ui32++;
        // Now loop through all actuators of each type in turn and supply the
        // Name and index number of each so the client can maintain their own
        // list:
        if(NSteppers){
          for(ui81=0;ui81<STEPPERSMAX;ui81++){
             if(StepperID[ui81]!=STEPPER_ID_FREE){
                ((uint8_t *)sv_Blob)[ui32]=ui81; ui32++;
                memcpy(sv_Blob+ui32,Stepperlist[ui81]->Name,32);
                ui32+=32;
               }
            }
         }

        if(NBinaries){
          for(ui81=0;ui81<BINARIESMAX;ui81++){
             if(BinaryID[ui81]!=BINARY_ID_FREE){
                ((uint8_t *)sv_Blob)[ui32]=ui81; ui32++;
                memcpy((uint8_t *)sv_Blob+ui32,Binarylist[ui81]->Name,32);
                ui32+=32;
               }
            }
         }

        if(NMlevels){
          for(ui81=0;ui81<MLEVELSMAX;ui81++){
             if(MlevelID[ui81]!=MLEVEL_ID_FREE){
                ((uint8_t *)sv_Blob)[ui32]=ui81; ui32++;
                memcpy((uint8_t *)sv_Blob+ui32,Mlevellist[ui81]->Name,32);
                ui32+=32;
               }
            }
         }

        if(NServos){
          for(ui81=0;ui81<SERVOSMAX;ui81++){
             if(ServoID[ui81]!=SERVO_ID_FREE){
                ((uint8_t *)sv_Blob)[ui32]=ui81; ui32++;
                memcpy((uint8_t *)sv_Blob+ui32,Servolist[ui81]->Name,32);
                ui32+=32;
               }
            }
         }

        if(NSensors){
          for(ui81=0;ui81<SENSORSMAX;ui81++){
             if(SensorID[ui81]!=SENSOR_ID_FREE){
                ((uint8_t *)sv_Blob)[ui32]=ui81; ui32++;
                memcpy((uint8_t *)sv_Blob+ui32,Sensorlist[ui81]->Name,32);
                ui32+=32;
               }
            }
         }

        sending_blob=1;
        sprintf(cmdtriad,"OK Done Ready");
       } else if(!strcmp(sv_triad_argv(1),"get_cameras")){
        // Calculate the size of the required blob
        // One uint32_t for the current camera and one for the number of cams:
        lenblob = 4 + 4;

        // Set the blob to hold that many bytes
        if(sv_blob_resize(lenblob)){
          sprintf(errmsg,"sv_blob resize failed for get_cameras");
          cmdstatus=CMD_ERR; goto process_get_stats_triad;
         }
        // Put the current values into the blob
        ui32=0; // We'll use this as the blob position index
        ((uint32_t *)sv_Blob)[ui32]=(uint32_t)CurrPS; ui32++;
        ((uint32_t *)sv_Blob)[ui32]=(uint32_t)NPSs;
        // Now send it.
        sending_blob=1;
        sprintf(cmdtriad,"OK Done Ready");
       } else if(!strcmp(sv_triad_argv(1),"get_camsettings")){
        size_t listsz;
        cmdstatus=sv_send_pardcap_ins(PS_CIID_LSET,errmsg,sv_triad_argv(1),0);
        if(cmdstatus==CMD_ERR) goto process_get_camsettings_triad;

        // If we get to here then it looks like the correct command was executed
        // correctly by PARDcap - so get the results back to the client:
        listsz=ps_Instruction.slen+1;
        if(sv_blob_resize(listsz)){
         sprintf(errmsg,"Failed to resize sv_blob"); 
         cmdstatus=CMD_ERR; goto process_get_camsettings_triad;
         }
        if(listsz>1){
          memcpy(sv_Blob,ps_Instruction.str,listsz);
          sending_blob=1;
         } else {
          sprintf(errmsg,"Cam settings list size too small");
          cmdstatus=CMD_ERR;
         }
        process_get_camsettings_triad:
        if(cmdstatus==CMD_ERR){ 
           sprintf(cmdtriad,"ERROR FailedCmd Ready");
           sending_bloberr=1;
         } else sprintf(cmdtriad,"OK Done Ready");

       } else {
         unknown_command:
          if(loginstatus==5){
             // A user attempting anything other than a password change on a OTP
             // may be a sign of a hacker - so log this on the security channel
             snprintf(msgtxt, sizeof(msgtxt), "User '%s' (IP: %s) is attempting"
                         " to run general commands.",sv_Username, sv_Client_IP);
             show_message(msgtxt,"Security notice: ",MT_SEC,0);

             // Block the account permanently until a sysadmin resets it
             // Marks it as an compromised/abused
             sv_UserDb[useridx].single_use = 2; 
             sv_save_user_file(sv_Userfile);
             // Send an 'UnpermittedCommand' response:
             if(sv_triad_set("ERROR UnpermittedCommand Ready"))
                                                             goto reboot_server;
            } else {
             // Send an 'UnknownCommand' response:
             if(sv_triad_set("ERROR UnknownCommand Ready")) goto reboot_server;
            }
          sv_update_message_header(sv_MES_USER_ERR);
          if(sv_send_message(responder,0)) goto reboot_server;
          sprintf(msgtxt,"%s[%d]: User %s[%d] was not understood.\n",
                                      sv_Pardname,sv_Mdx,sv_Username,sv_Climdx);
          show_message(msgtxt,"FYI: ",MT_SESS_T,0);
          // To prevent continual pre-occupation with an incompatible client
          // (these misunderstood commands may be due to version incompatibility
          // between server and client) the resend limit is incremented with
          // each misunderstood command. When the limit is reached the user is
          // automatically logged out.
          sv_Resendidx++;
          if(sv_Resendidx==sv_Resendmax){
              if(sv_triad_set("ERROR TooManyResends Bye"))goto reboot_server;
              sv_log_off_client(responder,sv_Triad,1);
          }
          continue;   
       }

      send_server_response:
      // Pardus received a known command, tried to execute it, prepared a
      // sv_Triad in cmdtriad and set any required image sv_Blob data. However,
      // the error message sv_Blob has not been dealt with so deal with that now
      if(sending_bloberr){
          lenerr=strlen(errmsg)+1; // +1 to include the string's null terminator
          if(sv_blob_resize(lenerr)){
            show_message("Failed to allocate memory for error message blob.\n",
                                                         "ERROR: ",MT_SESS_E,1);
            goto send_anyway;
          }
          memcpy(sv_Blob,errmsg,lenerr);
          sending_blob=1;
      }
      // Now send a response with that information
      // Send a response:
      send_anyway:
      if(sv_triad_set(cmdtriad))goto reboot_server;
      sv_update_message_header(sv_MES_NONE);
      if(sv_send_message(responder,sending_blob)) goto reboot_server;
      sprintf(msgtxt,"%s[%d]: User %s[%d] command done and reply sent.\n",
                                      sv_Pardname,sv_Mdx,sv_Username,sv_Climdx);    
      show_message(msgtxt,"FYI: ",MT_SESS_T,0);         
    }

 } // End of: while(sv_Server_run)
 
 end_server:   
    show_message("Shutting down Server. Done\n","FYI: ",MT_SESS_T,0);
    if(sv_Client_logged_in){
       sv_triad_set("ERROR ServerDown Bye");
       sv_log_off_client(responder,sv_Triad,1);
     } else sv_set_log_off_flags();
    // Anti-hang protection for Ctrl+C interrupts
    int linger = 0;
    zmq_setsockopt(responder, ZMQ_LINGER, &linger, sizeof(linger));

    zmq_close (responder);
    zmq_ctx_destroy (context);
 return 0;

 // Attempt to reboot the server - do this if an unrecoverable error occurs such
 // as a malloc/calloc failure or an unrecoverable zmq failure such as getting
 // the REQ-REP lock-step out of sync.
  reboot_server:
    if(sv_Server_run==0) goto end_server;
    show_message("Rebooting Server.\n","FYI: ",MT_SESS_T,0);
    sv_set_log_off_flags();
    zmq_close (responder);
    zmq_ctx_destroy (context);
    goto start_again;

}

//---------------------------------------------------------------------------//.
 //                             The PARD Server                             //.
  //-----------------------------------------------------------------------//.
#endif

#endif
//                                                                            //
///////                   END OF:  FUNCTION DEFINITIONS                /////////
//////////                                                            //////////
////////////////////////////////////////////////////////////////////////////////

// _.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._.-._. //


