#!/bin/bash
g++ -std=c++11 -Wall -g -o bin/tnode src/test_node.cpp src/threadhello.cpp src/udpserver.cpp -lpthread -lboost_system-mt
