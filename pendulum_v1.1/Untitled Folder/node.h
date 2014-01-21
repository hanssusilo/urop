#ifndef NODE_H
#define NODE_H
#include <vector>
#include <string>

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
  string print();
  void print_node();
  void add_child(Node* n);
  Node();
  Node(Node* n); // Copy constructor
  Node(double t, double x, double v, double theta, double w, double u, double cost, Node* parent);
  ~Node();
};
#endif
