#include <string>
#include "threadhello.h"
#include "nodeState.h"

nodeState::nodeState(int myPort, std::string neighbour1Add, std::string neighbour2Add)
    : add1(neighbour1Add), add2(neighbour2Add)

{
    myOccPos = myPort % 10 - 1;
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

void nodeState::setOccupancy(std::string occ){
    for (int i=0;i<8;i++){
        occupancy_[i] = (occ[i+1] == '1') ? 1 : 0;
    }
}

int nodeState::getOccupancy(int pos){
    return (pos >= 0 && pos <=7) ? occupancy_[pos] : -1;
}

void nodeState::setBackground(std::string bck){
    for (int i=0;i<8;i++){
        background_[i] = (bck[i+1] == '1') ? 1 : 0;
    }
}

float nodeState::getBackground(int pos){
    return (pos >= 0 && pos <=7) ? background_[pos] : -1;
}

void nodeState::setCoupling(std::string coup){
    int j = atoi(coup.substr(1,1).c_str());
    int i = atoi(coup.substr(2,1).c_str());
    float value = atoi(coup.substr(3,coup.back()).c_str());
    coupling_[j][i] = value;
}

float nodeState::getCoupling(int line, int col){
    return coupling_[line][col];
}
