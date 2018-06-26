/*
 * OpenCR Loader
 *
 * by Baram
 * by PBPH
 * by http://oroca.org
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include "opencr_ld.h"






/*---------------------------------------------------------------------------
     TITLE   : main
     WORK    :
---------------------------------------------------------------------------*/
int main( int argc, const char **argv )
{
  u8 not_flashing=0;
  u8 send_go_command=0;
  u8 boot_mode = 0;
  u8 minor, major;
  u16 version;
  long baud;
 

  printf("opencr_ld ver 1.0.4\n");

  if( argc < 4 )
  {
    fprintf( stderr, "Usage: opencl_ld <port> <baud> <binary image name> [<0|1 to send Go command to new flashed app>]\n" );
    exit( 1 );
  }

  errno = 0;
  baud = strtol( argv[ 2 ], NULL, 10 );
  if( ( errno == ERANGE && ( baud == LONG_MAX || baud == LONG_MIN ) ) || ( errno != 0 && baud == 0 ) || ( baud < 0 ) ) 
  {
    fprintf( stderr, "Invalid baud '%s'\n", argv[ 2 ] );
    exit( 1 );
  }
  

  opencr_ld_main( argc, argv );
  
  return 0;
}
           
