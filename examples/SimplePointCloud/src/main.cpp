#include <iostream>
#include <sstream>

#include "pointcloudviz.h"

int main( int argc, char* argv[] )
{
    try{
        PointcloudViz viz;
        viz.run();
    } catch( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }

    return 0;
}
