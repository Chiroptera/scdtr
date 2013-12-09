#!/bin/bash
g++ -std=c++11 src/udpserver2.cpp -lpthread -lboost_system-mt -o bin/udpserver
g++ -std=c++11 src/udpclient2.cpp -lpthread -lboost_system-mt -o bin/udpclient
#g++ -std=c++11 src/central.cpp -lpthread -lboost_system-mt -o bin/central
