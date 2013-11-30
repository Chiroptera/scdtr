#!/bin/bash
g++ -std=c++11 -Wall -o prog threadhello.cpp main.cpp -lpthread -lboost_system-mt
