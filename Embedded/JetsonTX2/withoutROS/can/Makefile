#!/bin/bash

all:
	g++ -o can_msq can_msq.cpp
	g++ -o can_sender can_sender.cpp
	g++ -o can_receiver can_receiver.cpp
	gcc -o can_test_transmitter can_test_transmitter.c
	gcc -o can_test_receiver can_test_receiver.c

clean:
	rm can_msq can_sender can_receiver can_test_transmitter can_test_receiver
