// Portable types

#ifndef __TYPE_H_
#define __TYPE_H_

#include <stdint.h>

#ifdef WIN32_BUILD
typedef char s8;
typedef unsigned char u8;

typedef short s16;
typedef unsigned short u16;

typedef int s32;
typedef unsigned int u32;

typedef long s64;
typedef unsigned long u64;
#else
typedef char s8;
typedef unsigned char u8;

typedef short s16;
typedef unsigned short u16;

typedef long s32;
typedef unsigned long u32;

typedef long long s64;
typedef unsigned long long u64;
#endif

// Define serial port "handle" type for each platform
// [TODO] for now, only UNIX is supported
#ifdef WIN32_BUILD
#include <windows.h>
typedef HANDLE ser_handler;
#else // assume POSIX here
typedef int ser_handler;
#endif


#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE 
#define FALSE 0
#endif

#endif

