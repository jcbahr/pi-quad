all:
	gcc -o a.out main.c -lpigpio -lpthread -lrt -lm
server:
	gcc -o server.out server.c

