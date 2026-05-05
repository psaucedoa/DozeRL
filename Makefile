CC = gcc
CFLAGS = -Wall -O2

all: sim benchmark

sim: sim.c
	$(CC) $(CFLAGS) -o sim sim.c -lm

benchmark: sim.c
	$(CC) $(CFLAGS) -DBENCHMARK -o benchmark sim.c -lm

clean:
	rm -f sim benchmark sim_out.txt
