CC = gcc
CFLAGS = -Wall -Wextra -Werror -std=c99 -O2

all: main

main: main.c
	$(CC) $(CFLAGS) -o main main.c
	@echo "Build successful! Run ./main to test saturation functions"

run: main
	./main

clean:
	rm -f main *.o

.PHONY: all run clean
