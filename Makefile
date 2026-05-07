CC = gcc
CFLAGS = -Wall -O2

all: sim benchmark

sim: src/sim.c src/sim.h
	$(CC) $(CFLAGS) -o sim src/sim.c -lm -Isrc

benchmark: src/sim.c src/sim.h
	$(CC) $(CFLAGS) -DBENCHMARK -o benchmark src/sim.c -lm -Isrc

clean:
	rm -f sim benchmark sim_out.txt
