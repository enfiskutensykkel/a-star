ifneq ($(shell which colorgcc),)
CC := golorgcc
else
CC := gcc
endif

COPTS := -Wall -Wextra -pedantic -O2

.PHONY: a-star all clean

all: a-star dijkstra

clean:
	-$(RM) a-star dijkstra

a-star: astar.c
	$(CC) $(COPTS) -o $@ $< -lm

dijkstra: astar.c
	$(CC) $(COPTS) -DDIJKSTRA -o $@ $< -lm

