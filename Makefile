# Makefile for Mars Rover Memory Allocator

CC = gcc
CFLAGS = -Wall -Werror -std=c11 -O2

all: liballocator.so runme

# Build shared allocator library
liballocator.so: allocator.c allocator.h
	$(CC) $(CFLAGS) -fPIC -shared allocator.c -o liballocator.so

# Build runme, linked against the shared library
runme: runme.c liballocator.so allocator.h
	$(CC) $(CFLAGS) runme.c -L. -lallocator -Wl,-rpath=. -o runme

# Run main test driver
test: runme
	./runme

clean:
	rm -f runme *.o *.so
