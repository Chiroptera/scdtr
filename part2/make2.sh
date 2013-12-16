#!/bin/bash
g++ -std=c++11 -Wall -g -gdwarf-2 -o bin/prog src/threadhello.cpp src/minicom2.cpp src/udpserver.cpp src/nodeState.cpp src/udpClientClass.cpp src/minicomnClass.cpp -lpthread -lboost_system-mt
