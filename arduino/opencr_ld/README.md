stm32ld
=======

stm32 downloader

OpenCM 다운로드 기능 추가. 


Compile - Mac/Linux
gcc -o stm32ld main.c stm32ld.c main_OpenCM.c serial_posix.c

Compile - Windows
gcc -o stm32ld main.c stm32ld.c main_OpenCM.c serial_win32.c  -DWIN32_BUILD



Execute 

stm32ld /dev/tty 115200 main.bin 1 


Execute OpenCM

stm32ld /dev/tty 115200 main.bin 1 opencm


