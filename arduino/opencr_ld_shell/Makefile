


all: clean  opencr_ld


SRCS  = main.c
SRCS += opencr_ld.c
SRCS += serial_posix.c
SRCS += ./msg/msg.c


opencr_ld:
	gcc -o opencr_ld_shell $(SRCS)

clean:
	rm -f opencr_ld_shell
