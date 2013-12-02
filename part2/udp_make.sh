#!/bin/bash
g++ -std=c++11 udpserver2.cpp -lpthread -lboost_system-mt -o udpserver
g++ -std=c++11 udpclient2.cpp -lpthread -lboost_system-mt -o udpclient

