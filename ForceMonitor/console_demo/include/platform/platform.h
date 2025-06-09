#ifndef kw_platform_h
#define kw_platform_h

#if defined(KW_OS_WINDOWS) || defined(WIN32) || defined(_WIN32) || defined(__WIN32__)

#ifndef KW_OS_WINDWS
#define KW_OS_WINDWS
#endif

#include <WinSock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

#elif defined(KW_OS_LINUX) || defined(__linux__)

#ifndef KW_OS_LINUX
#define KW_OS_LINUX
#endif

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/serial.h>



typedef sockaddr SOCKADDR;

#endif

#endif