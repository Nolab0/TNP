#include <kdtree.h>
#include <obj.h>

#include <Eigen/Eigenvalues>

std::pair<std::vector<Eigen::Vector3f>,std::vector<Eigen::Vector3f>> smooth(
        const std::vector<Eigen::Vector3f>& points,
        const std::vector<Eigen::Vector3f>& normals,
        float r)
{
    auto points_smooth = points;
    auto normals_smooth = normals;

    // step 3
    tnp::KdTree kdtree;
    kdtree.build(points);

    for (size_t i = 0; i < points.size(); i++) {
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        int m = 0;
        Eigen::Vector3f p_bar = Eigen::Vector3f::Zero();
        kdtree.for_each_neighbors(points, points[i], r, [&](size_t j) {
            if (i != j) {
                m++;
                p_bar += points[j];
            }
        });
        p_bar /= m;

        kdtree.for_each_neighbors(points, points[i], r, [&](size_t j) {
            if (i != j) {
                cov += (points[j] - p_bar) * (points[j] - p_bar).transpose();
            }
        });
        cov /= m;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(cov);
        eigensolver.compute(cov);
        Eigen::Vector3f v = eigensolver.eigenvectors().col(0);
        points_smooth[i] = p_bar;
        normals_smooth[i] = v;
    }

    return std::make_pair(std::move(points_smooth), std::move(normals_smooth));
}
