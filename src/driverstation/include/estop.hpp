#ifndef ESTOP_HEADER_INCLUDED
#define ESTOP_HEADER_INCLUDED

#include <iostream>

namespace driverstation::estop{
    void estop(){
        std::cout << "ESTOP TRIGGERED" << std::endl;
    }
}

#endif
