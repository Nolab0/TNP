#include <kdtree.h>
#include <stack>

namespace tnp {

KdTree::~KdTree()
{
    this->delete_rec(m_root);
}

// build the kdtree (without modifying the points)
void KdTree::build(const std::vector<Eigen::Vector3f>& points)
{
    // initialize indices with 0, 1, 2, ..., n-1
    m_indices.resize(points.size());
    std::iota(m_indices.begin(), m_indices.end(), 0);

    // allocate root node and recursively build the tree
    m_root = new Node();
    this->build_rec(points, m_indices.begin(), m_indices.end(), m_root);
}


// recursively build the tree
// consider points at indices in the range (begin,end(
// node is already allocated and must be filled
// points are not changed
void KdTree::build_rec(
    const std::vector<Eigen::Vector3f>& points, // point cloud
    iterator begin,                             // begin iterator over the current points indices
    iterator end,                               // end iterator over the current points indices
    Node* node)                                 // node to fill 
{
    // assert(node != nullptr);
    // assert(begin <= end);

    // step 1.1

    if (std::distance(begin, end) <= max_number_point_per_leaf) {
        node->cut_dim = -1;
        node->cut_value = 0;
        node->left_child = nullptr;
        node->right_child = nullptr;
        node->begin = begin;
        node->end = end;
        return;
    }
    else {
        Box3f box;
        for (auto it = begin; it != end; ++it) {
            box.extend(points[*it]);
        }
        const Eigen::Vector3f min = box.min();
        const Eigen::Vector3f max = box.max();
        const Eigen::Vector3f diagonal = box.diagonal();
        const int cut_dim = (diagonal.x() > diagonal.y()) ? ((diagonal.x() > diagonal.z()) ? 0 : 2) : ((diagonal.y() > diagonal.z()) ? 1 : 2);
        const float cut_value = (min[cut_dim] + max[cut_dim]) / 2;
        node->cut_dim = cut_dim;
        node->cut_value = cut_value;
        node->left_child = new Node();
        node->right_child = new Node();
        iterator mid = std::partition(begin, end, [&](int i) {return points[i][cut_dim] < cut_value; });
        this->build_rec(points, begin, mid, node->left_child);
        this->build_rec(points, mid, end, node->right_child);
    }
}


//
// neighbors range search from point p and distance r
// call f on each point at index i such that (p - points[i]).norm() < r
//
// Example: 
//     kdtree.for_each_neighbors(points, p, r, [&points](int i)
//     {
//         std::cout << "Found point " << i << ": " << points[i].transpose() << std::endl;
//     });
//
void KdTree::for_each_neighbors(
    const std::vector<Eigen::Vector3f>& points, // point cloud
    const Eigen::Vector3f& p,                   // query point
    float r,                                    // query radius
    Func f) const                               // function to called on resulting indices
{
    // stack used for iterative depth traversal
    std::stack<Node*> stack;
    stack.push(m_root);

    // step 1.2

    while (!stack.empty()) {
        Node* node = stack.top();
        stack.pop();
        if (node->cut_dim == -1) {
            for (auto it = node->begin; it != node->end; ++it) {
                if ((p - points[*it]).norm() < r) {
                    f(*it);
                }
            }
        }
        else {
            float ref = p[node->cut_dim];
            if (ref - r < node->cut_value) {
                stack.push(node->left_child);
            }
            if (ref + r >= node->cut_value) {
                stack.push(node->right_child);
            }
        }
    }
}

// recursively delete nodes
void KdTree::delete_rec(Node* node)
{
    // step 1.3

    if (node->cut_dim == -1) {
        delete node;
    }
    else {
        this->delete_rec(node->left_child);
        this->delete_rec(node->right_child);
        delete node;
    }
}

} // namespace tnp3