#include <iostream>
#include <sstream>

#include "depthviz.h"

int main( int argc, char* argv[] )
{
    try{
        DepthViz viz;
        viz.run();
    } catch( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }

    return 0;
}
