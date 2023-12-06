/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cmath>
#include <vector>

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node *left;
  Node *right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}

  ~Node() {
    delete left;
    delete right;
  }
};

struct KdTree {
  Node *root;

  KdTree() : root(NULL) {}

  ~KdTree() { delete root; }

  void insertHelper(Node *&node, uint depth, std::vector<float> point, int id,
                    uint maxAxis) {
    if (node == NULL) {
      node = new Node(point, id);
    } else {
      uint axis = depth % maxAxis;
      if (point.at(axis) < (node)->point.at(axis)) {
        insertHelper((node->left), depth + 1, point, id, maxAxis);
      } else {
        insertHelper((node->right), depth + 1, point, id, maxAxis);
      }
    }
  }
  void insert(std::vector<float> point, int id) {
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the
    // root
    int maxAxis = 2;
    int depth = 0;
    int direction;
    insertHelper(*&root, depth, point, id, maxAxis);
  }
  // Helper function to compare two points by the depth to alternative the
  // comparison
  // @param a, b: Points to compare
  // @param maxAxis: Maximum axis of dof to compare
  // @param depth: Current depth of the tree, as the axis to be compared are
  // alternated by the depth
  int compare(std::vector<float> a, std::vector<float> b, uint depth,
              uint maxAxis) {
    return a.at(depth % maxAxis) < b.at(depth % maxAxis);
  }

  bool collideBox(std::vector<float> target, std::vector<float> point,
                  float distanceTol, uint maxAxis) {
    bool result = true;
    int i = 0;
    while (result && i < maxAxis) {

      result = target.at(i) - distanceTol <= point.at(i) &&
               point.at(i) <= target.at(i) + distanceTol;
      i++;
    }
    return result;
  }

  float euclideanDistance(std::vector<float> a, std::vector<float> b,
                          int maxAxis) {
    float dist = 0;
    for (int i = 0; i < maxAxis; i++) {
      dist += pow(a.at(i) - b.at(i), 2);
    }
    return sqrt(dist);
  }
  void searchHelper(Node *&node, std::vector<float> target,
                    std::vector<int> &points, float distanceTol, uint depth,
                    uint maxAxis) {
    if (node == NULL) {
      return;
    }
    uint axis = depth % maxAxis;
    bool withinBounds = collideBox(target, node->point, distanceTol, maxAxis);

    if (withinBounds) {
      float dx = node->point.at(0) - target.at(0);
      float dy = node->point.at(1) - target.at(1);
      if (euclideanDistance(target, node->point, maxAxis) < distanceTol) {
        points.push_back(node->id);
      }
    }
    if (target.at(axis) - distanceTol < (node)->point.at(axis)) {
      searchHelper((node->left), target, points, distanceTol, depth + 1,
                   maxAxis);
    }
    if (target.at(axis) + distanceTol > (node)->point.at(axis)) {
      searchHelper((node->right), target, points, distanceTol, depth + 1,
                   maxAxis);
    }
  }
  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    uint depth = 0;
    int maxAxis = 2;
    searchHelper(*&root, target, ids, distanceTol, depth, maxAxis);
    return ids;
  }
};
