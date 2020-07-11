#ifndef TREE_H
#define TREE_H

#include "motion_planning/point.h"

using namespace std;

class Tree {
  public:
    node data;
    vector<Tree*> children;
    bool isGoal = false;

    Tree() {}
    Tree(point root, double start_yaw) {
      data.position = root;
      data.yaw = start_yaw;
    }
    Tree(node n) {
      data = n;
    }
};

#endif
