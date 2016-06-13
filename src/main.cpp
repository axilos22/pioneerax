//ARIA
#include "Aria.h"

//CPP
#include <iostream>

int trajectory() {
}

int main(int argc, char** argv) {
    Aria::init();
    ArArgumentParser parser(&argc,argv);
    std::cout << "ARIA LIB ready" << std::endl;
    return 0;
}
