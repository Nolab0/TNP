#ifndef TP1_SMOOTH_H
#define TP1_SMOOTH_H

#include <vector>
#include <Eigen/Core>

std::pair<std::vector<Eigen::Vector3f>,std::vector<Eigen::Vector3f>> smooth(
        const std::vector<Eigen::Vector3f>& points,
        const std::vector<Eigen::Vector3f>& normals,
        float r);

#endif //TP1_SMOOTH_H
