#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include "threadhello.h"

# ifndef NODESTATE_H
# define NODESTATE_H


using boost::asio::ip::udp;

class nodeState
{
 public:

    nodeState(int myPort, std::string neighbour1Add, std::string neighbour2Add);


    std::string getStringSelf();
    void setParametersSelf(std::string state);
    std::string getStringN1();
    void setParametersN1(std::string state);
    std::string getStringN2();
    void setParametersN2(std::string state);

    void setOccupancy(std::string occ);
    int getOccupancy(int pos);

    void setBackground(std::string bck);
    float getBackground(int pos);

    void setCoupling(std::string coup);
    float getCoupling(int line, int col);

 public:
    Arduino micro_;
    int myOccPos;
    Arduino micro1_; std::string add1;
    Arduino micro2_; std::string add2;
    float occupancy_[8], background_[8], coupling_[8][8];
};

#endif
