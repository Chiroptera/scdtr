#!/bin/bash
g++ -std=c++11 -Wall -o prog threadhello.cpp minicom.cpp -lpthread -lboost_system-mt
