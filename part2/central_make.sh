#g++-4.8 -std=c++11 src/central.cpp src/udpClientClass.cpp -lpthread -lboost_system-mt -o bin/central
#g++-4.8 -std=c++11 src/central.cpp -lpthread -lboost_system-mt -o bin/central
g++-4.8 -std=c++11 -g -gdwarf-2 src/central2.cpp src/threadhello.cpp -lpthread -lboost_system-mt -o bin/central2
