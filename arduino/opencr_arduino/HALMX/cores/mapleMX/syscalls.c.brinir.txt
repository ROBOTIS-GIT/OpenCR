/**************************************************************************//*****
 * @file     stdio.c
 * @brief    Implementation of newlib syscall
 ********************************************************************************/

#include <stm32f4xx_hal.h>
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#undef errno
extern int errno;
extern int  _end;

//UART_HandleTypeDef huart1;

__attribute__ ((used))
caddr_t _sbrk (int size)
{
  return 0;
}

__attribute__ ((used))
int link(char *old, char *new) {
return -1;
}

__attribute__ ((used))
int _close(int file)
{
  return -1;
}

__attribute__ ((used))
int _fstat(int file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

__attribute__ ((used))
int _isatty(int file)
{
  return 1;
}

__attribute__ ((used))
int _lseek(int file, int ptr, int dir)
{
  return 0;
}
__attribute__ ((used))
int _read(int file, char *ptr, int len)
{
  return 0;
}
__attribute__ ((used))
int _write(int file, char *ptr, int len)
{
	int txCount;
	(void)file;
	HAL_UART_Transmit(&huart6, (uint8_t*)ptr, len, 100);
	return len;
}

__attribute__((used))
void abort(void)
{
	while (1);
}

__attribute__ ((used))
void _exit(int status)
{
  while(1);
}

__attribute__ ((used))
int _open(const char *path, int access)
{
  return 0;
}

//__attribute__ ((used))
//void _fini(void)
//{
//}

/* --------------------------------- End Of File ------------------------------ */
