#include <iostream>
#include <sstream>

#include "colorviz.h"

int main( int argc, char* argv[] )
{
    try{
        ColorViz viz;
        viz.run();
    } catch( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }

    return 0;
}
