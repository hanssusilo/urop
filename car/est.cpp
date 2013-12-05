#include "est.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>

#define MAXD 4
#define MAXT 0.5*2
#define RSQUARED 16.0
#define EPSILON 2 // the range for determining whether the goal has been reached
#define DELETE 1 // 1 = delete nodes and shrink removeRange, 0 = keep nodes with fixed removeRange
#define REMOVE_FACTOR 4 // for shrinking removeRange
#define SUBNODES 7
#define VALID_SUBNODES 1
#define NONUNIFORM 0
#define RRT 1  // delete should be activated (at the momment)

// the constructor: start specifies the starting position and theta, goal specifies the end position
Est::Est(Node* start, Point goal) {
  b = goal;
  LoadBounds();
  LoadObstacles();
  root = new Node(start->x, start->y, start->theta, 0.0, NULL);
  root->weigh(1, 0);
  totalWeight = root->weight;
  nodes.push_back(root);
  #if DELETE == 0
  removeRange = 0.5; // the range for determining whether to add a node in a neighborhood, use 0.0 to turn off this option
  #else
  //removeRange = 1.0/nodes.size()*REMOVE_FACTOR;
  removeRange=6;
  #endif
  // create a kd tree only if the option is turned on
  if (removeRange > 0.0) {
    nodes_tree = kd_create(3);
    double pt[3] = {root->x, root->y,(root->theta)*70/3.14};
    if (kd_insert(nodes_tree, pt, root) != 0) {
      cout << "didn't insert root successfully\n";
    }
  }
  bestDist =150.0;
  srand(time(NULL));
}

Est::~Est() {
  for (int i = 0; i < nodes.size(); i++) {
    delete nodes[i];
  }
  if (removeRange > 0.0) {
    kd_free(nodes_tree);
  }
}

void Est::LoadBounds() {
  char value[100];
  ifstream in("bounds.txt");
  if (in.is_open()) {
    bounds = new Bounds();
    in.getline(value, 100, ',');
    bounds->lx = atof(value);
    in.getline(value, 100, ',');
    bounds->ly = atof(value);
    in.getline(value, 100, ',');
    bounds->ux = atof(value);
    in.getline(value, 100);
    bounds->uy = atof(value);
    in.close();
  } else {
    cout << "Uable to open file";
  }
}

void Est::LoadObstacles() {
  char value[100];
  Bounds* obstacle;
  ifstream in("obstacles.txt");
  if (in.is_open()) {
    while (in.getline(value, 100, ',')) {
      obstacle = new Bounds();
      obstacle->lx = atof(value);
      in.getline(value, 100, ',');
      obstacle->ly = atof(value);
      in.getline(value, 100, ',');
      obstacle->ux = atof(value);
      in.getline(value, 100);
      obstacle->uy = atof(value);
      obstacles.push_back(obstacle);
    }
    in.close();
  } else {
    cout << "Unable to open file";
  }
}

Node* Est::GetNodeToExpand() {
  int index = -1;
  double randWeight = totalWeight * (double)rand() / RAND_MAX;
  for (int i = 0; i < nodes.size(); i++) {
    randWeight -= nodes[i]->weight;
    if (randWeight <= 0) {
      index = i;
      break;
    }
  }
  if (index == -1) {
    cout << "Random node generated too high a number. Using last node.\n";
    index = nodes.size()-1;
  }
  return nodes[index];
}

Node* Est::GetNodeToExpandUniform(){
  int index; 
  index= (nodes.size())* (int)rand() / RAND_MAX;  
  return nodes[index];
}

Node* Est::GetNodeToExpandRRT(){
  //get a random point
  double pt[3] = { (bounds->ux - bounds->lx)*rand() / RAND_MAX , (bounds->uy - bounds->ly)*rand() / RAND_MAX, (70*3.14-0.0)*rand() / RAND_MAX};
  kdres* resultsNearest = kd_nearest(nodes_tree, pt);
  double pos[3];
  Node* tree_node = (Node*) kd_res_item(resultsNearest, pos);
  return tree_node;
}


Node* Est::GetNewNode(Node* parent, double u, double omega, double dist) {
  double newTheta = parent->theta + omega;
  if (newTheta > 2*PI) {
    newTheta -= 2*PI;
  } else if (newTheta < -2*PI) {
    newTheta += 2*PI;
  }
  double avgAngle = parent->theta + omega / 2;
  if (avgAngle > 2*PI) {
    avgAngle -= 2*PI;
  } else if (avgAngle < -2*PI) {
    avgAngle += 2*PI;
  }
  return new Node(parent->x + dist*cos(avgAngle), parent->y + dist*sin(avgAngle), newTheta, parent->distTravelled + u, parent);
}

void Est::AddNode(Node* n) {
  int count = 1;
#if RRT
#else
  for (int i = 0; i < nodes.size(); i++) {
    Node* other = nodes[i];
    if (pow((n->x - other->x), 2) + pow((n->y - other->y), 2) < RSQUARED) {
      count++;
      totalWeight -= other->weight;
      other->weigh(1, 0);
      totalWeight += other->weight;
    }
  }
  n->weigh(count, 0);
  totalWeight += n->weight;
#endif
  nodes.push_back(n);
  #if DELETE
// removeRange = (1.0/log(nodes.size()))  *REMOVE_FACTOR; //GEO
  removeRange = ((1.0/log10(double(this->iterat)+2)))*REMOVE_FACTOR + 4/log(double(this->iterat)+10.0) ;
  #endif
  if (removeRange > 0.0) {
    double pt[3] = {n->x, n->y, (n->theta)*70/3.14};
    if (kd_insert(nodes_tree, pt, n) != 0) {
      cout << "didn't insert node successfully\n";
    }
  }
}

vector<Node*> Est::GetNodes() {
  return nodes;
}

// removes the node and its children recursively from nodes, adding them to nodesToDelete
void Est::DeleteNode(Node* n) {
  vector<Node*>::iterator it = n->children.begin();
  while (it != n->children.end()) {
    DeleteNode(*it);
    it = n->children.erase(it);
  }
  nodes.erase(remove(nodes.begin(), nodes.end(), n), nodes.end());
  nodesToDelete.push_back(n);
}

// check whether to add this node based on its cost and neighborhood nodes
bool Est::CheckRange(Node* n) {
  bool add = true;
  double pt[3] = {n->x, n->y,(n->theta)*70/3.14};
  kdres* results = kd_nearest_range(nodes_tree, pt, removeRange);
  if (results != NULL) {
    vector<Node*> toDelete;
    while (!kd_res_end(results)) {
      double pos[3];
      Node* tree_node = (Node*) kd_res_item(results, pos);
      #if DELETE
      if (n->distTravelled > tree_node->distTravelled) {
        add = false;
      } else if (tree_node->distTravelled - sqrt(pow(tree_node->x - n->x, 2) + pow(tree_node->y - n->y, 2) /* + pow(tree_node->theta - n->theta, 2)*/ ) > n->distTravelled) {
        toDelete.push_back(tree_node);
      }
      #else
      if (n->distTravelled > tree_node->distTravelled) {
        add = false;
        break;
      }
      #endif
      kd_res_next(results);
    }
    kd_res_free(results);
    #if DELETE
    if (add) {
      for (int i = 0; i < toDelete.size(); i++) {
        Node* tree_node = toDelete[i];
        if (find(nodesToDelete.begin(), nodesToDelete.end(), tree_node) == nodesToDelete.end()) {
          Node* p = tree_node->parent;
          p->children.erase(remove(p->children.begin(), p->children.end(), tree_node), p->children.end());
          DeleteNode(tree_node);
        }
      }
    }
    if (nodesToDelete.size() > 0) {
      for (int i = 0; i < nodesToDelete.size(); i++) {
        delete nodesToDelete[i];
      }
      nodesToDelete.clear();
      // reconstruct nodes list, weights, and kdtree
      vector<Node*> tempNodes = nodes;
      nodes.clear();
      kd_clear(nodes_tree);
      totalWeight = 0.0;
      for (int i = 0; i < tempNodes.size(); i++) {
        Node* node = tempNodes[i];
        node->rangeCount = 0;
        node->failedExpansions = 0;
        AddNode(node);
      }
    }
    #endif
  }
  return add;
}

bool Est::Feasible(Node* n) {
  return (n->CheckBounds(bounds, obstacles) &&  // within bounds
          n->CheckCollisions(n->parent, obstacles) &&  // obstacle-free
          (bestDist == 0.0 || n->distTravelled <300.00 /* bestDist*/));  // cheaper than best path*/
}

bool Est::CheckGoal(Node* n, int iter) {
  // if this node is close enough to the goal, we have found a solution
  if (sqrt(pow((n->x - b.x), 2) + pow((n->y - b.y), 2)) < EPSILON) {
  // cout << "found new best path\n";
   if(n->distTravelled<bestDist)               {//GEO
    // delete old copy of best path except for root
    // if(bestPath.size()>0.0)       {
    for (int i = 0; i < (int)bestPath.size()-1; i++) {
      delete bestPath[i];
    }  
    bestPath.clear();            // }
    // make copies of nodes in best path and store them
    Node* copy = new Node(n);
    while (copy->parent != NULL) {
      bestPath.push_back(copy);
      copy = new Node(copy->parent);
    }
    bestPath.push_back(root);
    cout <<"iterations: "<<iter<< ", cost: " << n->distTravelled<<" ,RemoveRange:"<<removeRange<< endl;
    bestDist = n->distTravelled;
    return true;
  }                                       }//geo
  return false;
}

// find a path by taking the best path out of 'iterations' solutions
vector<Node*> Est::FindPathSolutions(int iterations) {
  for (int iter = 0; iter < iterations; iter++) {
    bool found = false; // whether a solution has been found
    while (!found) {
      // choose random node to expand
      Node* prev = GetNodeToExpand();
      // calculate random values for location of next node
      double u = (double)rand() / RAND_MAX * MAXD;
      double MAXT1=u/20;
      double omega = (double)rand() / RAND_MAX * 2 * MAXT1 - MAXT1;
      double dist = 2 * u * sin(omega/2)/omega;
      #if VALID_SUBNODES
      for (int i = 0; i < SUBNODES+1; i++) {
        Node* n = GetNewNode(prev, u, omega, dist);
        // if the node isn't feasible or shouldn't be added based on its neighborhood, delete it
        if (!Feasible(n) || (removeRange > 0.0 && !CheckRange(n))) {
          delete n;
          break;
        } else {
          // add node
          prev->children.push_back(n);
          AddNode(n);
          if (CheckGoal(n,iter)) {
            found = true;
            break;
          }
          prev = n;
        }
      }
      #else
      bool feasible = true;
      vector<Node*> subnodes;
      Node* parent = prev;
      for (int i = 0; i < SUBNODES; i++) {
        Node* n = GetNewNode(prev, u, omega, dist);
        if (Feasible(n)) {
          subnodes.push_back(n);
          prev = n;
        } else {
          delete n;
          for (int j = 0; j < subnodes.size(); j++) {
            delete subnodes[j];
          }
          feasible = false;
          break;
        }
      }
      if (feasible) {
        Node* n = GetNewNode(prev, u, omega, dist);
        if (!Feasible(n) || (removeRange > 0.0 && !CheckRange(n))) {
          delete n;
          for (int j = 0; j < subnodes.size(); j++) {
            delete subnodes[j];
          }
        } else {
          n->parent = parent;
          n->subnodes = subnodes;
          parent->children.push_back(n);
          AddNode(n);
          if (CheckGoal(n,iter)) {
            found = true;
          }
        }
      }
      #endif
    }
  }
  return bestPath;
}
// find a path by taking the best path out of 'iterations' iterations of expanding a node
vector<Node*> Est::FindPathIterations(int iterations) {
  char filename2 [128];
  sprintf(filename2,"TESTObstaclesCostVsIterationsE1_10.txt");
  std::ofstream fp_out2;
  fp_out2.open(filename2,std::ios::out);


  for (int iter = 0; iter < iterations; iter++) {
    // choose random node to expand
    iterat=iter;
   // cout<<iterat<<endl;
    Node* prev = GetNodeToExpandUniform();
#if(NONUNIFORM)    
    Node* prev1 = GetNodeToExpand();
    prev=prev1;
#endif
#if(RRT)
    Node* prev2 =  GetNodeToExpandRRT();
    prev=prev2;
#endif
    // calculate random values for location of next node
    double u =(double)rand() / RAND_MAX * MAXD;
    //double omega = (double)rand() / RAND_MAX * 2 * MAXT - MAXT;
   // double dist = 2 * u * sin(omega/2)/omega;
    double MAXT1=u/3;
    double omega = (double)rand() / RAND_MAX * 2 * MAXT1 - MAXT1;
    double dist = 2 * u * sin(omega/2)/omega;


    #if VALID_SUBNODES
    for (int i = 0; i < SUBNODES+1; i++) {
      Node* n = GetNewNode(prev, u, omega, dist);
      // if the node isn't feasible or shouldn't be added based on its neighborhood, delete it
      if (!Feasible(n) || (removeRange > 0.0 && !CheckRange(n))) {
        delete n;
        break;
      } else {
        // add node
        prev->children.push_back(n);
        AddNode(n);
        CheckGoal(n,iter);
        prev = n;
      }
    }
    #else
    bool feasible = true;
    vector<Node*> subnodes;
    Node* parent = prev;
    for (int i = 0; i < SUBNODES; i++) {
      Node* n = GetNewNode(prev, u, omega, dist);
      if (Feasible(n)) {
        subnodes.push_back(n);
        prev = n;
      } else {
        delete n;
        for (int j = 0; j < subnodes.size(); j++) {
          delete subnodes[j];
        }
        feasible = false;
        break;
      }
    }
    if (feasible) {
      Node* n = GetNewNode(prev, u, omega, dist);
      if (!Feasible(n) || (removeRange > 0.0 && !CheckRange(n))) {
        delete n;
        for (int j = 0; j < subnodes.size(); j++) {
          delete subnodes[j];
        }
      } else {
        n->parent = parent;
        n->subnodes = subnodes;        
        parent->children.push_back(n);
        AddNode(n);
        CheckGoal(n,iter);
      }
    }
    #endif
     if ((bestDist>0.0)&& (iter%100000==0))
       fp_out2<<iter<<","<<bestDist<<endl;
  }
  cout << "done iterating\n";
  return bestPath;
}
