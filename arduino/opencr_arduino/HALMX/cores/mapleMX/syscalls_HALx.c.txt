/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/**
  * \file syscalls_sam3.c
  *
  * Implementation of newlib syscall.
  *
  */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/


/* placeholders left over from Arduino sam implementation */

#include "syscalls.h"

#include <stdio.h>
#include <stdarg.h>

#if defined (  __GNUC__  ) /* GCC CS3 */
  #include <sys/types.h>
  #include <sys/stat.h>
#endif

// Helper macro to mark unused parameters and prevent compiler warnings.
// Appends _UNUSED to the variable name to prevent accidentally using them.
#ifdef __GNUC__
#  define UNUSED(x) x ## _UNUSED __attribute__((__unused__))
#else
#  define UNUSED(x) x ## _UNUSED
#endif

/*----------------------------------------------------------------------------
 *        Exported variables
 *----------------------------------------------------------------------------*/

#undef errno
extern int errno ;
extern int  _end ;

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
extern void _exit( int status ) ;
extern void _kill( int pid, int sig ) ;
extern int _getpid ( void ) ;

extern caddr_t _sbrk ( int incr )
{
	return -1;
}

extern int link( UNUSED(char *cOld), UNUSED(char *cNew) )
{
    return -1 ;
}

extern int _close( UNUSED(int file) )
{
    return -1 ;
}

extern int _fstat( UNUSED(int file), struct stat *st )
{
 
    return 0 ;
}

extern int _isatty( UNUSED(int file) )
{
    return 1 ;
}

extern int _lseek( UNUSED(int file), UNUSED(int ptr), UNUSED(int dir) )
{
    return 0 ;
}

extern int _read(UNUSED(int file), UNUSED(char *ptr), UNUSED(int len) )
{
    return 0 ;
}

extern int _write( UNUSED(int file), char *ptr, int len )
{

    return 0 ;
}

extern void _exit( int status )
{
/* not till we have something working 
    printf( "Exiting with status %d.\n", status ) ;
*/
    for ( ; ; ) ;

}

extern void _kill( UNUSED(int pid), UNUSED(int sig) )
{
    return ;
}

extern int _getpid ( void )
{
    return -1 ;
}
