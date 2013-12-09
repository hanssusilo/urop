#include "node.h"
#include <cmath>
#include <iostream>
#include <sstream>


std::string Node::print() {
    stringstream sstm;
    sstm << this->t << ',' << this->x << ',' << this->v << ',' << this->theta << ',' << this->w <<  ',' <<  this->cost << endl;
    return sstm.str();
}

void Node::print_node(){
    cout << this->t << '\t' << '\t' << "x:" <<  this->x << '\t' << '\t' << "v:" << this->v << '\t' << '\t' << "th:" << this->theta << '\t' << '\t' << "w:" << this->w << '\t' << '\t' << "cost:" << this->cost << endl;
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
  //subnodes = n->subnodes;
}

Node::Node(double t, double x, double v, double theta, double w, double u, double cost, Node* parent) {
  this->t = t;
  this->x = x;
  this->v = v;
  this->theta = theta;
  this->w = w;
  this->u = u;
  this->cost = cost;
  this->parent = parent;
}

Node::~Node() {
  for (int i = 0; i < subnodes.size(); i++) {
    delete subnodes[i];
  }
}


