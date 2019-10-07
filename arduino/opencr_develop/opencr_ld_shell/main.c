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


/*
  opencr_ld_shell make fw.bin fw_name fw_ver
  opencr_ld_shell view fw_name
*/





void make_firmware( int argc, const char **argv );
void view_firmware( int argc, const char **argv );



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


  printf("opencr_ld_shell ver 1.0.0\n");

  if (argc == 5 && strcmp(argv[ 1 ], "make") == 0)
  {
    make_firmware(argc, argv);
    return 0;
  }

  if (argc == 3 && strcmp(argv[ 1 ], "view") == 0)
  {
    view_firmware(argc, argv);
    return 0;
  }

  if( argc < 4 )
  {
    fprintf( stderr, "Usage: opencl_ld <port> <baud> <binary image name> [<0|1 to send Go command to new flashed app>]\n" );
    fprintf( stderr, "       opencr_ld_shell make fw.bin burger V171017R1\n" );
    fprintf( stderr, "       opencr_ld_shell view fw_name\n" );

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

void make_firmware( int argc, const char **argv )
{
  FILE    *fp;
  uint32_t fpsize;
  int      fw_size;

  char     *fw_ver_str;
  char     *fw_input_str;
  int      i;

  opencr_fw_header_t fw_header;

  printf("make firmware...\n");


  if( ( fp = fopen( argv[2], "rb" ) ) == NULL )
  {
    fprintf( stderr, "[NG] Unable to open \t: %s\n", argv[ 2 ] );
    exit( 1 );
  }
  else
  {
    fseek( fp, 0, SEEK_END );
    fpsize = ftell( fp );
    fseek(fp, 0, SEEK_SET);

    printf("[  ] file name   \t: %s \r\n", argv[2]);
    printf("[  ] file size   \t: %d bytes\r\n", fpsize);
  }

  fw_input_str = (char *)argv[2];
  fw_ver_str = (char *)argv[4];
  fw_size = fpsize;


  memset(&fw_header, 0, sizeof(opencr_fw_header_t));

  fw_header.magic_number = MAGIC_NUMBER;
  fw_header.fw_size = fw_size;

  strcpy(fw_header.fw_name_str, argv[3]);
  strcpy(fw_header.fw_ver_str, argv[4]);

  printf("[  ] fw_name     \t: %s \n", fw_header.fw_name_str);
  printf("[  ] fw_ver      \t: %s \n", fw_header.fw_ver_str);


  FILE    *fp_write;

  char fw_write_name[128];

  sprintf(fw_write_name, "%s.opencr", fw_header.fw_name_str);

  if( ( fp_write = fopen( fw_write_name, "wb" ) ) == NULL )
  {
    fclose(fp);
    fprintf( stderr, "[NG] Unable to open \t: %s\n", argv[ 2 ] );
    exit( 1 );
  }

  fwrite(&fw_header, 1, sizeof(opencr_fw_header_t), fp_write);

  for (i=0; i<fw_size; i++)
  {
    uint8_t data;

    fread(&data, 1, 1, fp);
    fwrite(&data, 1, 1, fp_write);
  }

  printf("[OK] finished    \t: %d bytes\n", (int)(fw_size + sizeof(opencr_fw_header_t)));

  fclose(fp_write);
  fclose(fp);
}

void view_firmware( int argc, const char **argv )
{
  FILE    *fp;
  uint32_t fpsize;
  int      fw_size;

  char     *fw_ver_str;
  char     *fw_input_str;

  opencr_fw_header_t fw_header;

  printf("view firmware...\n");


  if( ( fp = fopen( argv[2], "rb" ) ) == NULL )
  {
    fprintf( stderr, "[NG] Unable to open \t: %s\n", argv[ 2 ] );
    exit( 1 );
  }
  else
  {
    fseek( fp, 0, SEEK_END );
    fpsize = ftell( fp );
    fseek(fp, 0, SEEK_SET);

    printf("[  ] file name   \t: %s \r\n", argv[2]);
    printf("[  ] file size   \t: %d KB\r\n", fpsize/1024);
  }

  fw_size = fpsize;


  fread(&fw_header, 1, sizeof(opencr_fw_header_t), fp);

  if (fw_header.magic_number == MAGIC_NUMBER)
  {
    printf("[  ] fw_name     \t: %s \n", fw_header.fw_name_str);
    printf("[  ] fw_ver      \t: %s \n", fw_header.fw_ver_str);
  }
  else
  {
    printf("[NG] not opencr fw \n");
  }


  fclose(fp);
}
