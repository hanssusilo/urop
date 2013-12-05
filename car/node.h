#include "bounds.h"
#include "point.h"
#include <vector>

#define PI 3.14159

using namespace std;

class Node {
 public:
  double x;
  double y;
  double theta;
  double distTravelled;
  Node* parent;
  vector<Node*> children;
  double weight;
  int rangeCount;
  int failedExpansions;
  vector<Point*> seen_points;
  vector<Node*> subnodes;
  Node(); // Root constructor
  Node(Node* n); // Copy constructor
  Node(double x, double y);
  Node(double x, double y, double theta, double distTravelled, Node* parent);
  ~Node();
  void weigh(int rangeCount, int failedExpansions);
  bool CheckBounds(Bounds* bounds, vector<Bounds*> obstacles);
  vector<Point*> GetVisibility(vector<Point*> points, vector<Bounds*> obstacles);
  bool CheckCollisions(Node* parent, vector<Bounds*> obstacles);
};
