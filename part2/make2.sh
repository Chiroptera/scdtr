#!/bin/bash
g++ -std=c++11 -Wall -g -o bin/prog src/threadhello.cpp src/minicom2.cpp -lpthread -lboost_system-mt
