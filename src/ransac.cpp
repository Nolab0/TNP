#include <Eigen/Core>
#include <obj.h>
#include <iostream>
#include <random>
#include "kdtree.h"

std::vector<Eigen::Vector3f> getSamplePoints(std::vector<Eigen::Vector3f> points, size_t n, std::vector<size_t> remainingPointsIndices){
    std::vector<Eigen::Vector3f> sample_points;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, remainingPointsIndices.size()-1);
    while (sample_points.size() < n){
        Eigen::Vector3f p = points[remainingPointsIndices[dis(gen)]];
        if (std::find(sample_points.begin(), sample_points.end(), p) == sample_points.end()){
            sample_points.push_back(p);
        }
    }
    return sample_points;
}

void buildPlane(std::vector<Eigen::Vector3f>& inliers, tnp::KdTree& kdTree, float radius, const Eigen::Vector3f& startingPoint, std::vector<Eigen::Vector3f> &colors, Eigen::Vector3f color, std::vector<size_t> &inliersIndices, std::vector<bool> &visited){
    kdTree.for_each_neighbors(inliers, startingPoint, radius, [&](int i){
        if (visited[i]){
            return;
        }
        colors[inliersIndices[i]] = color;
        Eigen::Vector3f p = inliers[i];
        visited[i] = true;
        buildPlane(inliers, kdTree, radius, p, colors, color, inliersIndices, visited);
    });
}

std::vector<size_t> simpleRansac(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& normals,std::vector<Eigen::Vector3f> &colors, Eigen::Vector3f color, std::vector<size_t> remainingPointsIndices)
{
    int m = 2000;
    float epsilon = 0.3f;
    Eigen::Vector3f best_p;
    Eigen::Vector3f best_n;
    int best_count = 0;
    for (int i = 0; i < m; i++){
        std::vector<Eigen::Vector3f> sample_points = getSamplePoints(points, 3, remainingPointsIndices);

        Eigen::Vector3f p = sample_points[0];
        Eigen::Vector3f q = sample_points[1];
        Eigen::Vector3f r = sample_points[2];
        Eigen::Vector3f n = ((q-p).cross(r-p)).normalized();
        int count = 0;
        for (int j = 0; j < remainingPointsIndices.size(); j++){
            Eigen::Vector3f point = points[remainingPointsIndices[j]];
            Eigen::Vector3f normal = normals[remainingPointsIndices[j]];
            Eigen::Vector3f v = point - p;
            if (std::abs(v.dot(n)) < epsilon and std::abs(normal.dot(n)) > 0.8f){
                count++;
            }
        }
        if (count > best_count){
            best_count = count;
            best_p = p;
            best_n = n;
        }
    }

    std::vector<size_t> newRemainingPointsIndices;
    std::vector<size_t> inliersIndices;
    for (size_t i = 0; i < remainingPointsIndices.size(); i++){
        Eigen::Vector3f v = points[remainingPointsIndices[i]] - best_p;
        Eigen::Vector3f n = normals[remainingPointsIndices[i]];

        if (std::abs(v.dot(best_n)) < epsilon and std::abs(best_n.dot(n)) > 0.8f){
            colors[remainingPointsIndices[i]] = color;
            inliersIndices.push_back(remainingPointsIndices[i]);
        }
        else{
            newRemainingPointsIndices.push_back(remainingPointsIndices[i]);
        }
    }

    /*std::vector<Eigen::Vector3f> inliers;
    for (size_t i = 0; i < inliersIndices.size(); i++){
        inliers.push_back(points[inliersIndices[i]]);
    }

    std::vector<bool> visited(inliers.size(), false);

    float radius = 0.5f;
    tnp::KdTree kdTree;
    kdTree.build(inliers);
    buildPlane(inliers, kdTree, radius, inliers[0], colors, color, inliersIndices, visited);*/

    return newRemainingPointsIndices;
}

Eigen::Vector3f generateColor(){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0f, 1.0f);
    return Eigen::Vector3f(dis(gen), dis(gen), dis(gen));
}

void multiRansac(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& normals, std::vector<Eigen::Vector3f> & colors){
    std::vector<size_t> remainingPointsIndices(points.size());
    std::iota(remainingPointsIndices.begin(), remainingPointsIndices.end(), 0);
    while (remainingPointsIndices.size() > (size_t)(points.size() * 0.5f))
    {
        remainingPointsIndices = simpleRansac(points, normals, colors, generateColor(), remainingPointsIndices);
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
    multiRansac(points, normals, colors);
    //simpleRansac(points, colors, Eigen::Vector3f(1.0f, 0.0f, 0.0f));

    tnp::save_obj("ransacNormals.obj", points, normals, colors);
    
    return 0;
}
