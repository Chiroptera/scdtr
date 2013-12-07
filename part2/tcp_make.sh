#!/bin/bash
g++ -std=c++11 src/tcpserver.cpp -lpthread -lboost_system-mt -o bin/tcpserver
g++ -std=c++11 src/tcpclient.cpp -lpthread -lboost_system-mt -o bin/tcpclient
