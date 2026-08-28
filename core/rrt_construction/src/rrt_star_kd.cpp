#include "rrt_construction/rrt_star_kd.h"

/* ----------------------- Construction & tree mutation ----------------------- */

rrt_star::Node::Node(const Eigen::Vector4d& p) : point(p), parent(nullptr), cost(0), gain(0), absolute_gain(-1.0), absolute_yaw(0.0), score(0), cum_gain(0) {}

void rrt_star::KDTree_data::clear() {
    points.clear();
    data.clear();
}

rrt_star::Node* rrt_star::addKDTreeNode(std::unique_ptr<Node> node) {
    Node* parentNode = node->parent;
    Node* observer = tree_data_.addNode(std::move(node), parentNode);
    kdtree_->addPoints(tree_data_.points.size() - 1, tree_data_.points.size() - 1);
    return observer;
}

void rrt_star::clearKDTree() {
    tree_data_.clear();
    kdtree_ = std::unique_ptr<Tree>(new Tree(3, tree_data_));
}

void rrt_star::initializeKDTreeWithNodes(std::vector<std::unique_ptr<Node>>& nodes) {
    tree_data_.addNodes(nodes);
    kdtree_->addPoints(0, tree_data_.points.size() - 1);
}

/* ----------------------- Sampling ----------------------- */

void rrt_star::computeSamplingDimensions(double radius, Eigen::Vector3d& result) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-radius, radius);

    bool solutionFound = false;
    double rand_x, rand_y, rand_z;
    while (!solutionFound) {
        rand_x = dis(gen);
        rand_y = dis(gen);
        rand_z = dis(gen);
        if (Eigen::Vector3d(rand_x, rand_y, rand_z).norm() > radius) {
            continue;
        }
        solutionFound = true;
    }

    result = Eigen::Vector3d(rand_x, rand_y, rand_z);
}

void rrt_star::computeSamplingDimensionsNBV(double radius, Eigen::Vector4d& result) {
    bool solutionFound = false;
    double rand_x, rand_y, rand_z, rand_yaw;
    while (!solutionFound) {
        rand_x = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        rand_y = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        rand_z = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        if (Eigen::Vector3d(rand_x, rand_y, rand_z).norm() > radius) {
            continue;
        }
        solutionFound = true;
    }
    rand_yaw = 2.0 * M_PI * (((double) rand()) / ((double) RAND_MAX) - 0.5);

    result = Eigen::Vector4d(rand_x, rand_y, rand_z, rand_yaw);
}

/* ----------------------- Nearest / nearby queries ----------------------- */

void rrt_star::findNearestKD(const Eigen::Vector3d& point, Node*& nearestNode) {
    double query_pt[3] = {point.x(), point.y(), point.z()};
    nanoflann::KNNResultSet<double> resultSet(1);
    size_t index;
    double out_dist_sqr;
    resultSet.init(&index, &out_dist_sqr);
    kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(10));
    nearestNode = tree_data_.data[index].get();
}

void rrt_star::findNearbyKD(Node* point, double radius, std::vector<Node*>& nearbyNodes) {
    nearbyNodes.clear();
    Eigen::Vector3d query_pt = {point->point.x(), point->point.y(), point->point.z()};
    std::size_t ret_index[10];
    double out_dist[10];
    nanoflann::KNNResultSet<double> resultSet(10);
    resultSet.init(ret_index, out_dist);
    kdtree_->findNeighbors(resultSet, query_pt.data(), nanoflann::SearchParameters(10));
    for (int i = 0; i < resultSet.size(); ++i) {
        if (out_dist[i] <= pow(radius, 2.0)) {
            nearbyNodes.push_back(tree_data_.data[ret_index[i]].get());
        }
    }
}

/* ----------------------- Steering ----------------------- */

void rrt_star::steer_parent(Node* fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::unique_ptr<Node>& new_node, bool fixed_step, double minEdge) {
    Eigen::Vector3d from = fromNode->point.head<3>();
    double dist = (toPoint - from).norm();
    double edge = stepSize;
    if (!fixed_step && dist < stepSize) {
        edge = dist;
        if (edge < minEdge) edge = minEdge;
    }
    Eigen::Vector3d newPoint = from + edge * (toPoint - from).normalized();
    new_node = std::make_unique<Node>(Eigen::Vector4d(newPoint.x(), newPoint.y(), newPoint.z(), fromNode->point.w()));
    new_node->parent = fromNode;
}

/* ----------------------- RRT* wiring ----------------------- */

void rrt_star::chooseParent(Node* new_node, const std::vector<Node*>& nearbyNodes) {
    double minCost = std::numeric_limits<double>::infinity();
    Node* parent = nullptr;
    for (const auto& node : nearbyNodes) {
        // Reject candidate parents whose connecting edge passes through an obstacle.
        if (edge_free_ && !edge_free_(node->point.head<3>(), new_node->point.head<3>())) continue;
        double cost = node->cost + (node->point.head<3>() - new_node->point.head<3>()).norm();
        if (cost < minCost) {
            minCost = cost;
            parent = node;
        }
    }
    new_node->parent = parent;
    new_node->cost = minCost;
}

void rrt_star::rewire(Node* new_node, std::vector<Node*>& nearby_nodes, double radius) {
    for (Node* node : nearby_nodes) {
        double new_cost = new_node->cost + (node->point.head<3>() - new_node->point.head<3>()).norm();
        if (new_cost < node->cost) {
            // Only re-parent through new_node if that edge is actually collision-free.
            if (edge_free_ && !edge_free_(new_node->point.head<3>(), node->point.head<3>())) continue;
            if (node->parent) {
                std::vector<Node*>& siblings = node->parent->children;
                siblings.erase(std::remove(siblings.begin(), siblings.end(), node), siblings.end());
            }
            // Re-link under the new parent (raw observers on both ends).
            node->parent = new_node;
            new_node->children.push_back(node);
            node->cost = new_cost;
            propagateCost(node);
        }
    }
}

void rrt_star::propagateCost(Node* node) {
    for (Node* child : node->children) {
        child->cost = node->cost + (child->point.head<3>() - node->point.head<3>()).norm();
        propagateCost(child);
    }
}

/* ----------------------- Path extraction ----------------------- */

void rrt_star::backtrackPathNode(Node* node, std::vector<Eigen::Vector4d>& path, Node*& nextBestNode) {
    Node* currentNode = node;
    while (currentNode) {
        path.push_back(currentNode->point);
        currentNode = currentNode->parent;
        if (currentNode && currentNode->parent && currentNode->cost < nextBestNode->cost) {
            nextBestNode = currentNode;
        }
    }
    std::reverse(path.begin(), path.end());
}

void rrt_star::backtrackPathAEP(Node* node, std::vector<std::unique_ptr<Node>>& path) {
    // Collect the branch (node -> root) as observers first.
    std::vector<Node*> chain;
    for (Node* currentNode = node; currentNode; currentNode = currentNode->parent) {
        chain.push_back(currentNode);
    }
    std::reverse(chain.begin(), chain.end());   // root first

    path.clear();
    Node* prev_clone = nullptr;
    for (Node* original : chain) {
        std::unique_ptr<Node> copy = std::make_unique<Node>(*original);
        copy->parent = prev_clone;
        copy->children.clear();   // drop copied links into the live tree; this branch is self-contained
        prev_clone = copy.get();
        path.push_back(std::move(copy));
    }
}

/* ----------------------- Static utilities ----------------------- */

void rrt_star::sortByDepth(std::vector<Node*>& nodes) {
    auto depth = [](Node* n) { int d = 0; for (auto* p = n->parent; p; p = p->parent) ++d; return d; };
    std::stable_sort(nodes.begin(), nodes.end(),
                     [&](Node* a, Node* b) { return depth(a) < depth(b); });
}
