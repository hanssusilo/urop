#ifndef PENDULUM_H
#define PENDULUM_H

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>

#include "node.h"
#include "bounds.h"
#include "obstacles.h"
#include "dynamics.cpp"
#include "kdtree.h"
#include "point.h"

using namespace std;

class Pendulum {
  Point b; // goal
  Dynamics d;
  Node* root; // start
  Bounds* bounds;
  vector<Obstacles*> obstacles;
  kdtree* nodes_tree;
  vector<Node*> nodes;
  vector<Node*> goal_nodes;

  
  void LoadBounds();
  void LoadObstacles();

  bool CheckBounds(Node* n);
  bool CheckCollisions(Node* n);
  bool Feasible(Node* n);
  bool CheckGoal(Node* n);





//  vector<Node*> bestPath;

//  vector<Node*> nodesToDelete;

//  Node* GetNodeToExpand();
  Node* GetNodeToExpandUniform();
  Node* GetNodeToExpandRRT();

//  Node* GetNewNode(Node* parent, double u, double omega, double dist);
//  void AddNode(Node* n);
//  void DeleteNode(Node* n);

 public:
  Pendulum(Node* start, Point goal);
  ~Pendulum();
//  vector<Node*> FindPathSolutions(int iterations);
  vector<Node*> FindPathIterations(int iterations);
//  vector<Node*> GetNodes();
};
#endif