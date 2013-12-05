#ifndef NODE_H
#define NODE_H
#include <vector>

#define PI 3.14159

using namespace std;

class Node {
 public:
  double t;
  double x;
  double v;
  double theta;
  double w;
  double u;
  double cost;
  Node* parent;
  vector<Node*> children;
  vector<Node*> subnodes;
  void print_node();
  Node(Node* n); // Copy constructor
  Node(double t, double x, double v, double theta, double w, double u, double cost, Node* parent);
  ~Node();
};
#endif
