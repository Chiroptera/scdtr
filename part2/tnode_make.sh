#!/bin/bash
g++ -std=c++11 -Wall -g -gdwarf-2 -o bin/tnode src/test_node.cpp src/threadhello.cpp src/udpserver.cpp src/udpClientClass.cpp src/nodeState.cpp -lpthread -lboost_system-mt
