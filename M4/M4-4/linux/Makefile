CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra -std=c11 -D_POSIX_C_SOURCE=200809L -D_DEFAULT_SOURCE
TARGET   = carlink
OBJS     = main.o serial.o protocol.o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS)

main.o: main.c serial.h protocol.h
	$(CC) $(CFLAGS) -c main.c -o main.o

serial.o: serial.c serial.h
	$(CC) $(CFLAGS) -c serial.c -o serial.o

protocol.o: protocol.c protocol.h
	$(CC) $(CFLAGS) -c protocol.c -o protocol.o

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
