ifneq ($(shell which colorgcc),)
CC := golorgcc
else
CC := gcc
endif

COPTS := -std=c99 -Wall -Wextra -pedantic -O2

.PHONY: all clean diagonal

all: astar.c
	$(CC) $(COPTS) $(DEFS) -o a-star $< -lm
	$(CC) $(COPTS) $(DEFS) -DDIJKSTRA -o dijkstra $< -lm

clean:
	-$(RM) a-star dijkstra

diagonal: DEFS += -DALLOW_DIAGONAL
diagonal: all
