#ifndef EST_H
#define EST_H

#include "node.h"
#include "kdtree.h"

using namespace std;

class Est {
  Point b; // goal
  Node* root; // start
  double removeRange;
  double totalWeight;
  Bounds* bounds;
  vector<Bounds*> obstacles;
  vector<Node*> nodes;
  kdtree* nodes_tree;
  vector<Node*> bestPath;
  double bestDist;
  int iterat;
  vector<Node*> nodesToDelete;
  void LoadBounds();
  void LoadObstacles();
  Node* GetNodeToExpand();
  Node* GetNodeToExpandUniform();
  Node* GetNodeToExpandRRT();
  Node* GetNewNode(Node* parent, double u, double omega, double dist);
  void AddNode(Node* n);
  void DeleteNode(Node* n);
  bool CheckRange(Node* n);
  bool Feasible(Node* n);
  bool CheckGoal(Node* n, int);
 public:
  Est(Node* start, Point goal);
  ~Est();
  vector<Node*> FindPathSolutions(int iterations);
  vector<Node*> FindPathIterations(int iterations);
  vector<Node*> GetNodes();
};
#endif
