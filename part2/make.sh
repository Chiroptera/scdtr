#!/bin/bash
g++ -std=c++11 -Wall -o bin/prog src/threadhello.cpp src/minicom.cpp -lpthread -lboost_system-mt
