#include "node.h"
#include <cmath>
#include <iostream>
#include <sstream>


std::string Node::print() {
    stringstream sstm;
    sstm << this->t << ',' << this->x << ',' << this->v << ',' << this->theta << ',' << this->w <<  ',' <<  this->u << ',' <<  this->cost << endl;
    return sstm.str();
}

void Node::print_node(){
    cout << this->t << '\t' << '\t' << "x:" <<  this->x << '\t' << '\t' << "v:" << this->v << '\t' << '\t' << "th:" << this->theta << '\t' << '\t' << "w:" << this->w << '\t' << '\t' << "cost:" << this->cost << endl;
}

void Node::add_child(Node* n){
    this->children.push_back(n);
}

Node::Node() {
  this->parent = NULL;
}

Node::Node(Node* n) {
  this->t = n->t;
  this->x = n->x;
  this->v = n->v;
  this->theta = n->theta;
  this->w = n->w;
  this->u = n->u;
  this->cost = n->cost;
  this->parent = n->parent;
  this->times = n->times;
  this->iteration = n->iteration;
  this->trajectory = n->trajectory;
  //subnodes = n->subnodes;
}

Node::Node(double t, double x, double v, double theta, double w, double u, double cost, Node* parent, vector<double> times, long iteration, vector<state_type> trajectory) {
  this->t = t;
  this->x = x;
  this->v = v;
  this->theta = theta;
  this->w = w;
  this->u = u;
  this->cost = cost;
  this->parent = parent;
  this->times = times;
  this->iteration = iteration;
  this->trajectory = trajectory;
}

Node::~Node() {
  for (int i = 0; i < subnodes.size(); i++) {
    delete subnodes[i];
  }
}


