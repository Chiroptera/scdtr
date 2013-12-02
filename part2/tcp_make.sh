#!/bin/bash
g++ -std=c++11 tcpserver.cpp -lpthread -lboost_system-mt -o tcpserver
g++ -std=c++11 tcpclient.cpp -lpthread -lboost_system-mt -o tcpclient
