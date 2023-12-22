#include <Eigen/Core>
#include <Eigen/Geometry>
#include <obj.h>
#include <iostream>
#include <random>

std::vector<Eigen::Vector3f> getSamplePoints(std::vector<Eigen::Vector3f> points, int n){
    std::vector<Eigen::Vector3f> sample_points;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size()-1);
    while (sample_points.size() < n){
        Eigen::Vector3f p = points[dis(gen)];
        if (std::find(sample_points.begin(), sample_points.end(), p) == sample_points.end()){
            sample_points.push_back(p);
        }
    }
    return sample_points;
}

void simpleRansac(std::vector<Eigen::Vector3f> points, std::vector<Eigen::Vector3f> & colors){
    std::random_device rd;
    std::mt19937 gen(rd());

    int m = 100;
    float epsilon = 0.15f;
    Eigen::Vector3f best_p;
    Eigen::Vector3f best_n;
    int best_count = 0;
    for (int i = 0; i < m; i++){
        std::vector<Eigen::Vector3f> sample_points = getSamplePoints(points, 3);

        Eigen::Vector3f p = sample_points[0];
        Eigen::Vector3f q = sample_points[1];
        Eigen::Vector3f r = sample_points[2];
        Eigen::Vector3f n = ((q-p).cross(r-p)).normalized();
        int count = 0;
        for (auto & point : points){
            Eigen::Vector3f v = point - p;
            if (std::abs(v.dot(n)) < epsilon){
                count++;
            }
        }
        if (count > best_count){
            best_count = count;
            best_p = p;
            best_n = n;
        }
    }

    for (size_t i = 0; i < points.size(); i++){
        Eigen::Vector3f v = points[i] - best_p;
        if (std::abs(v.dot(best_n)) < epsilon){
            colors[i] = Eigen::Vector3f(1, 0, 0);
        }
    }
}

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
    std::vector<Eigen::Vector3f> normals;
    std::vector<Eigen::Vector3f> colors;

    if(not tnp::load_obj(filename, points, normals, colors)) {
        std::cout << "Failed to open input file '" << filename << "'" << std::endl;
        return 1;
    }

    // ransac -----------------------------------------------------------------
    simpleRansac(points, colors);

    tnp::save_obj("ransac.obj", points, normals, colors);
    
    return 0;
}
