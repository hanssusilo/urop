#include "pendulum.h"


/*
    Pendulum constructor
    start = the starting state, 
    goal = the end state (x, v, and theta)
*/
Pendulum::Pendulum(Node* start, Point goal) {
  b = goal;
  
  LoadBounds();
  LoadObstacles();

  root = new Node(start);

  nodes.push_back(root);


  nodes_tree = kd_create(4);
  double pt[4] = {root->x, root->v, (root->theta)*70/3.14, root->w};
  if (kd_insert(nodes_tree, pt, root) != 0) {
    cout << "didn't insert root successfully\n";
  }

  srand(time(NULL));
}

Pendulum::~Pendulum() {
  for (int i = 0; i < nodes.size(); i++) {
    delete nodes[i];
  }
//  kd_free(nodes_tree);
}

void Pendulum::LoadBounds() {
  char value[100];
  ifstream in("bounds.txt");
  if (in.is_open()) {
    bounds = new Bounds();

    in.getline(value, 100, ',');
    bounds->lx = atof(value);
    in.getline(value, 100, ',');
    bounds->ux = atof(value);

    in.getline(value, 100, ',');
    bounds->max_v = atof(value);
    in.getline(value, 100, ',');
    bounds->max_w = atof(value);


    in.getline(value, 100, ',');
    max_u = atof(value);

    in.getline(value, 100, ',');
    x_epsilon = atof(value);
    in.getline(value, 100, ',');
    v_epsilon = atof(value);

    in.close();
  } else {
    cout << "Unable to open file";
  }
}

void Pendulum::LoadObstacles() {
  char value[100];
  Obstacles* obstacle;
  ifstream in("obstacles.txt");
  if (in.is_open()) {
    while (in.getline(value, 100, ',')) {
      obstacle = new Obstacles();

      obstacle->lx = atof(value);
      in.getline(value, 100, ',');
      obstacle->ux = atof(value);
      
      in.getline(value, 100, ',');
      obstacle->ly = atof(value);
      in.getline(value, 100);
      obstacle->uy = atof(value);

      obstacles.push_back(obstacle);
      //cout << obstacle->ly << obstacle->uy;
    }
    in.close();
  } else {
    cout << "Unable to open file";
  }
}



Node* Pendulum::GetNodeToExpandUniform(){
  int index; 
  index= rand() % (nodes.size());  
  return nodes[index];
}



Node* Pendulum::GetNodeToExpandRRT(){
  double max_v = bounds->max_v;
  double max_w = bounds->max_w;
  double pt[4] = { (bounds->ux - bounds->lx)*rand() / RAND_MAX , (max_v)*rand() / RAND_MAX - (max_v)*0.5, (70.0*6.28)*rand() / RAND_MAX, (max_w)*rand() / RAND_MAX - (max_w)*0.5};
  kdres* resultsNearest = kd_nearest(nodes_tree, pt);
  double pos[4];
  Node* tree_node = (Node*) kd_res_item(resultsNearest, pos);
  return tree_node;
}


/*
    Returns true if node state is within bounds
*/
bool Pendulum::CheckBounds(Node* n) {
    double x = n->x;
    double theta = n->theta;
    double px = x+d.L*sin(theta);
    double v = n->v;
    double w = n->w;

    double lx = bounds->lx;
    double ux = bounds->ux;
    double max_v = bounds->max_v;
    double max_w = bounds->max_w;

    bool v_out = (abs(v) > max_v);      //main mass vel out of bounds
    bool w_out = (abs(w) > max_w);      //pendulum ang vel out of bounds
    bool x_out = (x < lx || x > ux);    //main mass out of bounds
    bool p_out = (px < lx || px > ux);  //pendulum out of bounds
    return !(x_out || p_out || v_out || w_out);   
}
/*
    Returns true if node state has no collisions
    currently works for rectangular obstacles
*/
bool Pendulum::CheckCollisions(Node* n) {
    double theta = n->theta;
    double py = -d.L*cos(theta);
    double px = (n->x)+d.L*sin(theta);
    for (int i = 0; i < obstacles.size(); i++) {
        Obstacles* o = obstacles[i];
        double lx = o->lx;
        //double ly = o->ly;
        double ux = o->ux;
        double uy = o->uy;
        if (px < ux && px > lx && py < uy){
            //cout << "found collision!\n";
            return false;
        }
    }
    return true;
}
/*
    Returns true if node state is within bounds and obstacle-free
*/
bool Pendulum::Feasible(Node* n) {
    return (CheckBounds(n) && CheckCollisions(n));  
}
/*
    Returns true if node is close enough to goal
*/
bool Pendulum::CheckGoal(Node* n) {
    return ((abs(n->x - b.x) < x_epsilon) && (abs(n->v - b.v) < v_epsilon));
}

vector<Node*> Pendulum::FindPathIterations(int iterations) {
    cout << "starting path iteration!\n";
    vector<Node*> goal_nodes;
    double min_cost = DBL_MAX;
    int sol = 0;
    for (int iter = 0; iter < iterations; iter++){
        //cout << sol << endl;
        Node* prev = GetNodeToExpandRRT();
        double u =((double)rand() / RAND_MAX - 0.5)*max_u; 
        Node* n = d.update(prev, u);
        while (!Feasible(n)){
            //n->print_node();
            //cout << "found unfeasible node!\n";
            delete n;
            prev = GetNodeToExpandRRT();
            u =((double)rand() / RAND_MAX - 0.5)*max_u;
            n = d.update(prev, u);
        }
        //n->print_node();
        if (CheckGoal(n)){
            if (n->cost < min_cost){
                min_cost = n->cost;
                goal_nodes.push_back(n);
                cout << "found better goal node! " << n->cost << endl;
                sol = sol+1;
            } /*
            else{
                cout << "found less optimal goal node! " << n->cost << endl;
            }
            */
        //Time and work are strictly positive
        } else {
            double pt[4] = {n->x, n->v, (n->theta)*70/3.14, n->w};
            if (kd_insert(nodes_tree, pt, n) != 0) {
                cout << "didn't insert root successfully\n";
            }
            nodes.push_back(n);
        }
    }
    cout << "finished iterating!\n";
    cout << "Number of solutions found: " << sol << endl;
    return goal_nodes;
}

vector<Node*> Pendulum::TraverseNodes(Node* n) {
    cout << "starting traversal!\n";
    vector<Node*> traj;
    Node* p = n;
    while (p != NULL){
        traj.push_back(p);
        p = p->parent;
    }
    reverse(traj.begin(), traj.end());
    cout << "finished traversal!\n";
    return traj;
}

vector<Node*> Pendulum::FindOptimalPathIterations(int iterations){
    vector<Node*> goal_nodes = FindPathIterations(iterations);
    vector<Node*> traj = TraverseNodes(goal_nodes[goal_nodes.size()-1]);
/* 
    for (int i=0; i<traj.size(); i++){
        traj[i]->print_node();
    }
*/        
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];

    time (&rawtime);
    timeinfo = localtime (&rawtime);

    strftime(buffer,80,"%m_%d_%Y %H_%M_%S",timeinfo);

    char filename2 [128];
    sprintf(filename2,"cost/path_%s.txt", buffer);
    std::ofstream fp_out;
    fp_out.open(filename2,std::ios::out);

    fp_out << b.x << "," << x_epsilon << "," << d.L << endl;
    for (int i=0; i<traj.size(); i++){
        fp_out << traj[i]->print();
    }

    return traj;
}


/*    

    


void Planner::writepath(Node *n)
{
        Node *curnode;
        char filename[128];
        sprintf(filename,"%s_path_%d_%d_%f_%d_%f.txt",NAME,solutions,nodetracker->numnodes,elapsedtime,iterations,shortest_path);
        std::ofstream fp_out;
        fp_out.open(filename, std::ios::out);
        curnode = n;

        // We write out the path in reverse order by travelling up the tree from the leaf node to the parent
        while (curnode != NULL)
        {
                // We write out the subnodes in reverse order too
                for (int i=SUBSTEPS-1; i>-1; i--)
                {
                        fp_out << curnode->dubins[i].x << "\t" << curnode->dubins[i].y << "\t" << curnode->dubins[i].theta << "\n";
                }
                curnode = curnode->parent;
        }
        fp_out.close();
        return;
}


Node* Pendulum::MinCostPath(vector<Node*> nodes) {
    if (nodes.size() < 1){
        return NULL;
    }
    Node* node = nodes[0];
    for( vector<Node*>::size_type i = 1; i < nodes.size(); i++) {
        if (nodes[i]->cost < node->cost){
            node = nodes[i];
        }
    }
    return node;
}
*/
