#include <Eigen/Core>
#include <Eigen/Geometry>
#include <obj.h>
#include <iostream>
#include <random>

std::vector<Eigen::Vector3f> getSamplePoints(std::vector<Eigen::Vector3f> points, size_t n, std::vector<bool> remainingPoints){
    std::vector<Eigen::Vector3f> sample_points;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size()-1);
    while (sample_points.size() < n){
        Eigen::Vector3f p = points[dis(gen)];
        if (!remainingPoints[dis(gen)]){
            continue;
        }
        if (std::find(sample_points.begin(), sample_points.end(), p) == sample_points.end()){
            sample_points.push_back(p);
        }
    }
    return sample_points;
}

void simpleRansac(std::vector<Eigen::Vector3f> points, std::vector<Eigen::Vector3f> & colors, Eigen::Vector3f color, std::vector<bool>& remainingPoints){

    std::random_device rd;
    std::mt19937 gen(rd());

    int m = 2000;
    float epsilon = 0.3f;
    Eigen::Vector3f best_p;
    Eigen::Vector3f best_n;
    int best_count = 0;
    for (int i = 0; i < m; i++){
        std::vector<Eigen::Vector3f> sample_points = getSamplePoints(points, 3, remainingPoints);

        Eigen::Vector3f p = sample_points[0];
        Eigen::Vector3f q = sample_points[1];
        Eigen::Vector3f r = sample_points[2];
        Eigen::Vector3f n = ((q-p).cross(r-p)).normalized();
        int count = 0;
        for (int j = 0; j < points.size(); j++){
            if (!remainingPoints[j]){
                continue;
            }
            Eigen::Vector3f point = points[j];
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
        if (remainingPoints[i] && std::abs(v.dot(best_n)) < epsilon){
            colors[i] = color;
            remainingPoints[i] = false;
        }
    }
}

void multiRansac(std::vector<Eigen::Vector3f> points, std::vector<Eigen::Vector3f> & colors){
    Eigen::Vector3f color1(1.0f, 0.0f, 0.0f);  // Red
    Eigen::Vector3f color2(0.0f, 1.0f, 0.0f);  // Green
    Eigen::Vector3f color3(0.0f, 0.0f, 1.0f);  // Blue
    Eigen::Vector3f color4(1.0f, 1.0f, 0.0f);  // Yellow
    Eigen::Vector3f color5(1.0f, 0.0f, 1.0f);  // Magenta
    Eigen::Vector3f color6(0.0f, 1.0f, 1.0f);  // Cyan
    Eigen::Vector3f color7(0.5f, 0.5f, 0.5f);  // Gray
    Eigen::Vector3f color8(1.0f, 0.5f, 0.0f);  // Orange
    Eigen::Vector3f color9(0.5f, 1.0f, 0.0f);  // Lime
    Eigen::Vector3f color10(0.0f, 0.5f, 1.0f); // Light Blue
    Eigen::Vector3f color11(0.7f, 0.2f, 0.5f); // Custom Color
    Eigen::Vector3f color12(0.3f, 0.8f, 0.1f); // Custom Color

    std::vector<Eigen::Vector3f> colorVector = {color1, color2, color3, color4, color5, color6,
                                                color7, color8, color9, color10, color11, color12};
    std::vector<bool> remainingPoints(points.size(), true);
    for (int i = 0; i < 5; i++)
    {
        simpleRansac(points, colors, colorVector[i], remainingPoints);
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

    if (colors.size() == 0){
        colors = std::vector<Eigen::Vector3f>(points.size(), Eigen::Vector3f(0.5f, 0.5f, 0.5f));
    }

    // ransac -----------------------------------------------------------------
    multiRansac(points, colors);
    //simpleRansac(points, colors, Eigen::Vector3f(1.0f, 0.0f, 0.0f));

    tnp::save_obj("ransac.obj", points, normals, colors);
    
    return 0;
}
