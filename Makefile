ifneq ($(shell which colorgcc),)
CC := golorgcc
else
CC := gcc
endif

DEFS  := -DALLOW_DIAGONAL
COPTS := -std=c99 -Wall -Wextra -pedantic -O2

.PHONY: a-star all clean

all: a-star dijkstra

clean:
	-$(RM) a-star dijkstra

a-star: astar.c
	$(CC) $(COPTS) $(DEFS) -o $@ $< -lm

dijkstra: astar.c
	$(CC) $(COPTS) $(DEFS) -DDIJKSTRA -o $@ $< -lm

