#include <Eigen/Core>
#include <Eigen/Geometry>
#include <obj.h>
#include <iostream>


int main(int argc, char const *argv[])
{
    // option -----------------------------------------------------------------
    if(argc <= 1) {
        std::cout << "Error: missing argument" << std::endl;
        std::cout << "Usage: ransac <filename>.obj" << std::endl;
        return 0;
    }
    const std::string filename = argv[1];

    // load -------------------------------------------------------------------
    std::vector<Eigen::Vector3f> points;
    
    if(not tnp::load_obj(filename, points)) {
        std::cout << "Failed to open input file '" << filename << "'" << std::endl;
        return 1;
    }



    
    return 0;
}
