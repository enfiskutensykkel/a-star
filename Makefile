ifneq ($(shell which colorgcc),)
CC := golorgcc
else
CC := gcc
endif

COPTS := -Wall -Wextra -pedantic -g -O2

.PHONY: a-star all clean

all: a-star

clean:
	-$(RM) astar.o

a-star: astar.o
	$(CC) -o $@ $^

%.o: %.c
	$(CC) $(COPTS) -o $@ $< -c
