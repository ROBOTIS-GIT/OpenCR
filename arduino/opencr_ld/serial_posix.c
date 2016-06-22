// Serial inteface implementation for POSIX-compliant systems

#include "serial.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>

static u32 ser_timeout = SER_INF_TIMEOUT;

// Open the serial port
ser_handler ser_open( const char* sername )
{
  int fd;

  if( ( fd = open( sername, O_RDWR | O_NOCTTY | O_NDELAY ) ) == -1 )
    perror( "ser_open: unable to open port" );
  else
    fcntl( fd, F_SETFL, 0 );
  return ( ser_handler )fd;
}

// Close the serial port
void ser_close( ser_handler id )
{
  close( ( int )id );
}

// Helper function: get baud ID from actual baud rate
#define BAUDCASE(x)  case x: return B##x
static u32 ser_baud_to_id( u32 baud )
{
  switch( baud )
  {
    BAUDCASE( 1200 );
    BAUDCASE( 1800 );
    BAUDCASE( 2400 );
    BAUDCASE( 4800 );
    BAUDCASE( 9600 );
    BAUDCASE( 19200 );
    BAUDCASE( 38400 );
    BAUDCASE( 57600 );
    BAUDCASE( 115200 );
    BAUDCASE( 230400 );
  }
  return 0;
}

// Helper function: get number of bits ID from actual number of bits
#define NBCASE(x) case x: return CS##x
static int ser_number_of_bits_to_id( int nb )
{
  switch( nb )
  {
    NBCASE( 5 );
    NBCASE( 6 );
    NBCASE( 7 );
    NBCASE( 8 );
  }
  return 0;
}

int ser_setup( ser_handler id, u32 baud, int databits, int parity, int stopbits )
{
  struct termios termdata;
  int hnd = ( int )id;

  usleep( 200000 );
  tcgetattr( hnd, &termdata );

  // Baud rate
  cfsetispeed( &termdata, ser_baud_to_id( baud ) );
  cfsetospeed( &termdata, ser_baud_to_id( baud ) );

  // Parity / stop bits
  termdata.c_cflag &= ~CSTOPB;
  if( parity == SER_PARITY_NONE ) // no parity
  {
    termdata.c_cflag &= ~PARENB;
  }
  else if( parity == SER_PARITY_EVEN ) // even parity
  {
    termdata.c_cflag |= PARENB;
    termdata.c_cflag &= ~PARODD;
  }
  else if( parity == SER_PARITY_ODD ) // odd parity
  {
    termdata.c_cflag |= PARENB;
    termdata.c_cflag |= PARODD;
  }

   // Data bits
  termdata.c_cflag |= ( CLOCAL | CREAD );
  termdata.c_cflag &= ~CSIZE;
  termdata.c_cflag |= ser_number_of_bits_to_id( databits );

  // Disable HW and SW flow control
  termdata.c_cflag &= ~CRTSCTS;
  termdata.c_iflag &= ~( IXON | IXOFF | IXANY );

  // Raw input
  termdata.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG );

  // Raw output
  termdata.c_oflag &= ~OPOST;

  // Check and strip parity bit
  if( parity == SER_PARITY_NONE )
    termdata.c_iflag &= ~( INPCK | ISTRIP );
  else
    termdata.c_iflag |= ( INPCK | ISTRIP );

  // Setup timeouts
  termdata.c_cc[ VMIN ]  = 1;
  termdata.c_cc[ VTIME ] = 0;

  // Set the attibutes now
  tcsetattr( hnd, TCSANOW, &termdata );

  // Flush everything
  tcflush( hnd, TCIOFLUSH );

  // And set blocking mode by default
  fcntl( id, F_SETFL, 0 );

  return 0;
}


int ser_setupEx( ser_handler id, u32 baud, int databits, int parity, int stopbits, int Mode )
{
  struct termios termdata;
  int hnd = ( int )id;

  usleep( 200000 );
  tcgetattr( hnd, &termdata );

  // Baud rate
  cfsetispeed( &termdata, ser_baud_to_id( baud ) );
  cfsetospeed( &termdata, ser_baud_to_id( baud ) );

  // Parity / stop bits
  termdata.c_cflag &= ~CSTOPB;
  if( parity == SER_PARITY_NONE ) // no parity
  {
    termdata.c_cflag &= ~PARENB;
  }
  else if( parity == SER_PARITY_EVEN ) // even parity
  {
    termdata.c_cflag |= PARENB;
    termdata.c_cflag &= ~PARODD;
  }
  else if( parity == SER_PARITY_ODD ) // odd parity
  {
    termdata.c_cflag |= PARENB;
    termdata.c_cflag |= PARODD;
  }

   // Data bits
  termdata.c_cflag |= ( CLOCAL | CREAD );
  termdata.c_cflag &= ~CSIZE;
  termdata.c_cflag |= ser_number_of_bits_to_id( databits );



  // Disable HW and SW flow control
  termdata.c_cflag &= ~CRTSCTS;
  termdata.c_iflag &= ~( IXON | IXOFF | IXANY );


  if( Mode == 0 )
    termdata.c_cflag &= ~CRTSCTS;
  else
    termdata.c_cflag |= CRTSCTS;    /* Also called CRTSCTS */

    

  // Raw input
  termdata.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG );

  // Raw output
  termdata.c_oflag &= ~OPOST;


  // Check and strip parity bit
  if( parity == SER_PARITY_NONE )
    termdata.c_iflag &= ~( INPCK | ISTRIP );
  else
    termdata.c_iflag |= ( INPCK | ISTRIP );

  // Setup timeouts
  termdata.c_cc[ VMIN ]  = 1;
  termdata.c_cc[ VTIME ] = 0;

  // Set the attibutes now
  tcsetattr( hnd, TCSANOW, &termdata );

  // Flush everything
  tcflush( hnd, TCIOFLUSH );

  // And set blocking mode by default
  fcntl( id, F_SETFL, 0 );

  return 0;
}



// Read up to the specified number of bytes, return bytes actually read
u32 ser_read( ser_handler id, u8* dest, u32 maxsize )
{
  if( ser_timeout == SER_INF_TIMEOUT )
    return ( u32 )read( ( int )id, dest, maxsize );
  else
  {
    fd_set readfs;
    struct timeval tv;
    int retval;

    FD_ZERO( &readfs );
    FD_SET( ( int )id, &readfs );
    tv.tv_sec = ser_timeout / 1000000;
    tv.tv_usec = ( ser_timeout % 1000000 ) * 1000;
    retval = select( ( int )id + 1, &readfs, NULL, NULL, &tv );
    if( retval == -1 || retval == 0 )
      return 0;
    else 
      return ( u32 )read( ( int )id, dest, maxsize );
  }
}

// Read a single byte and return it (or -1 for error)
int ser_read_byte( ser_handler id )
{
  u8 data;
  int res = ser_read( id, &data, 1 );

  return res == 1 ? data : -1;
}

// Write up to the specified number of bytes, return bytes actually written
u32 ser_write( ser_handler id, const u8 *src, u32 size )
{
  u32 res;
  
  res = ( u32 )write( ( int )id, src, size );
  return res;
}

// Write a byte to the serial port
u32 ser_write_byte( ser_handler id, u8 data )
{
  return ( u32 )write( id, &data, 1 );
}

// Set communication timeout
void ser_set_timeout_ms( ser_handler id, u32 timeout )
{
  ser_timeout = timeout;
}

