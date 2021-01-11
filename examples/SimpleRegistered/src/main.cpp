#include <iostream>
#include <sstream>

#include "registeredviz.h"

int main( int argc, char* argv[] )
{
    try{
        RegisteredViz viz;
        viz.run();
    } catch( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }

    return 0;
}
