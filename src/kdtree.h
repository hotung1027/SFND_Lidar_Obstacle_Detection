/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include <cmath>
#include <vector>

// Structure to represent node of kd tree
template <typename PointT> struct Node {
  PointT point;
  int id;
  Node *left;
  Node *right;

  Node(PointT pt, int setId) : point(pt), id(setId), left(NULL), right(NULL) {}

  ~Node() {
    delete left;
    delete right;
  }
};
template <typename PointT> struct KdTree {
  Node<PointT> *root;

  KdTree() : root(NULL) {}

  ~KdTree() { delete root; }

  void insertHelper(Node<PointT> *&node, uint depth, PointT point, int id,
                    uint maxAxis) {
    if (node == NULL) {
      node = new Node(point, id);
    } else {
      uint axis = depth % maxAxis;
      if (point(axis) < (node)->point(axis)) {
        insertHelper((node->left), depth + 1, point, id, maxAxis);
      } else {
        insertHelper((node->right), depth + 1, point, id, maxAxis);
      }
    }
  }
  void insert(PointT point, int id, int maxAxis) {
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the
    // root
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
  int compare(PointT a, PointT b, uint depth, uint maxAxis) {
    return a(depth % maxAxis) < b(depth % maxAxis);
  }

  bool collideBox(PointT target, PointT point, float distanceTol,
                  uint maxAxis) {
    bool result = true;
    int i = 0;
    while (result && i < maxAxis) {

      result = target(i) - distanceTol <= point(i) &&
               point(i) <= target(i) + distanceTol;
      i++;
    }
    return result;
  }

  float euclideanDistance(PointT a, PointT b, int maxAxis) {
    float dist = 0;
    for (int i = 0; i < maxAxis; i++) {
      dist += pow(a(i) - b(i), 2);
    }
    return sqrt(dist);
  }
  void searchHelper(Node<PointT> *&node, PointT target,
                    std::vector<int> &points, float distanceTol, uint depth,
                    uint maxAxis) {
    if (node == NULL) {
      return;
    }
    uint axis = depth % maxAxis;
    bool withinBounds = collideBox(target, node->point, distanceTol, maxAxis);

    if (withinBounds) {

      if (euclideanDistance(target, node->point, maxAxis) < distanceTol) {
        points.push_back(node->id);
      }
    }
    if (target(axis) - distanceTol < (node)->point(axis)) {
      searchHelper((node->left), target, points, distanceTol, depth + 1,
                   maxAxis);
    }
    if (target(axis) + distanceTol > (node)->point(axis)) {
      searchHelper((node->right), target, points, distanceTol, depth + 1,
                   maxAxis);
    }
  }
  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(PointT target, float distanceTol, int maxAxis) {
    std::vector<int> ids;
    uint depth = 0;
    searchHelper(*&root, target, ids, distanceTol, depth, maxAxis);
    return ids;
  }
};
