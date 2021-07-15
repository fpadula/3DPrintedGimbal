CC=g++
CFLAGS=-Wall

all: Controller.o main.o 
	$(CC) $(CFLAGS) ./bin/main.o ./bin/Controller.o -o ./bin/main

Controller.o: ./src/Controller.cpp ./src/Controller.h
	$(CC) $(CFLAGS) -c ./src/Controller.cpp -o ./bin/Controller.o

main.o: Controller.o ./src/main.cpp
	$(CC) $(CFLAGS) -c ./src/main.cpp -o ./bin/main.o

clean:
	rm ./bin/*