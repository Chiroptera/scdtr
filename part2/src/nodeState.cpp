#include <string>
#include <iostream>
#include <sstream>
#include "threadhello.h"
#include <stdlib.h>
#include "nodeState.h"

nodeState::nodeState(int myPort, std::string neighbour1Add, std::string neighbour2Add)
    : add1(neighbour1Add), add2(neighbour2Add),
      backgroundCounter(0), couplingCounter(0),
      ready_(false), toWrite_(false)

{
    myOccPos = myPort % 10 - 1;
    for (int i=0;i<8;i++)
    {
        occupancy_[i]=0;
        background_[i]=0;
        for (int j=0;j<8;j++)
        {
            coupling_[i][j]=0;
        }
    }
}

std::string nodeState::getStringSelf(){
    return micro_.getString();
}

void nodeState::setParametersSelf(std::string state){
    micro_.set_parameters(state);
}

std::string nodeState::getStringN1(){
    return micro1_.getString();
}

void nodeState::setParametersN1(std::string state){
    micro1_.set_parameters(state);
}

std::string nodeState::getStringN2(){
    return micro2_.getString();
}

void nodeState::setParametersN2(std::string state){
    micro2_.set_parameters(state);
}

void nodeState::setMyOccupancy(){
    std::cout << "INSIDE SETMYOCCUPANCY I HAVE PRESENCE " << micro_.getPresence() << std::endl;
    std::cout << "MYOCCPOS IS " << myOccPos << std::endl;

    occupancy_[myOccPos]=micro_.getPresence();

    std::cout << "INSIDE OCCUPANCY VECTOR IN MYOCCPOS IS " << occupancy_[myOccPos] << std::endl;
}

void nodeState::setOccupancy(std::string occ){

    for (int i=0;i<8;i++){
        if (i==myOccPos) continue;
        occupancy_[i] = (occ[i+1] == '1') ? 1 : 0;
    }
}

int nodeState::getOccupancy(int pos){
    return (pos >= 0 && pos <=7) ? occupancy_[pos] : -1;
}

void nodeState::setBackground(std::string bck){
    int i = atoi(bck.substr(1,1).c_str());
    float value = atof(bck.substr(2,bck.back()).c_str());
    background_[i] = value;
    backgroundCounter++;
    if (backgroundCounter == 7 && couplingCounter == 63) ready_ = true;
}

float nodeState::getBackground(int pos){
    return (pos >= 0 && pos <=7) ? background_[pos] : -1;
}

void nodeState::setCoupling(std::string coup){
    int j = atoi(coup.substr(1,1).c_str());
    int i = atoi(coup.substr(2,1).c_str());
    float value = atof(coup.substr(3,coup.back()).c_str());
    coupling_[j][i] = value;
    couplingCounter++;
    if (backgroundCounter == 7 && couplingCounter == 63) ready_ = true;

}

float nodeState::getCoupling(int line, int col){
    return coupling_[line][col];
}

std::string nodeState::getOccString(){
    std::stringstream out;
    out << "O";
    for (int i=0;i<8;i++){
        out << std::dec << occupancy_[i];
    }
    return out.str();
}

int nodeState::parseState(std::string msg, std::string sender){
    if (msg[0] == 'O'){
        std::cout << "GOT OCCUPANCY " << msg << "FROM " << sender << std::endl;
        setOccupancy(msg);
    }
    else if (msg[0] == 'B'){
        std::cout << "GOT BACKGROUND " << msg << "FROM " << sender << std::endl;
        setBackground(msg);
    }
    else if (msg[0] == 'C'){
        std::cout << "GOT COUPLING " << msg << "FROM " << sender << std::endl;
        setCoupling(msg);
    }
    else if (sender == add1){
        micro1_.set_parameters(msg);
    }
    else if (sender == add2){
        micro2_.set_parameters(msg);
    }
    else if(msg[0]=='Z'){
        return 0;
    }
    else {
        return -1;
    }

    return 0;
}
