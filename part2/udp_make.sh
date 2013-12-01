#!/bin/bash
g++ -std=c++11 udpserver.cpp -lpthread -lboost_system-mt -o udpserver
g++ -std=c++11 udpclient.cpp -lpthread -lboost_system-mt -o udpclient

