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
  double pt[4] = {root->x*10.0/(bounds->ux-bounds->lx), root->v*10.0/(2*bounds->max_v)+5.0, root->theta*10.0/(6.28318530718), root->w*10.0/(2*bounds->max_w)+5.0};
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

    in.getline(value, 100, ',');
    theta_epsilon = atof(value);
    in.getline(value, 100);
    w_epsilon = atof(value);

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
  double pt[4] = {10.0*rand()/RAND_MAX, 10.0*rand()/RAND_MAX, 10.0*rand()/RAND_MAX, 10.0*rand()/RAND_MAX};
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
    double px = x+2*d.L*sin(theta);
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

double PerpDot(double ax, double ay, double bx, double by) {
  return (ax * by - ay * bx);
}


/*
    Returns true if vector a collides with vector b
*/
bool LineCollision(double alx, double aly, double aux, double auy, double blx, double bly, double bux, double buy) {
    if (alx < aux){
        double ax = alx;
        alx = aux;
        aux = ax;    
    }
    if (aly < auy){
        double ay = aly;
        aly = auy;
        auy = ay;    
    }
  double f = PerpDot(aux - alx, auy - aly, bux - blx, buy - bly);
  if (f == 0.0) {
    return false;
  }
  double r = PerpDot(aux - alx, auy - aly, bux - aux, buy - auy);
  double s = PerpDot(bux - blx, buy - bly, bux - aux, buy - auy);
  if ((f > 0.0 && r > 0.0 && s > 0.0 && f > r && f > s) ||
      (f < 0.0 && r < 0.0 && s < 0.0 && f < r && f < s)) {
    return true;
  }
  return false;
}


/*
    Returns true if vector (alx, aly)->(aux, auy) 
    collides with rectangular obstacle
*/
bool obstacleCollision(double alx, double aly, double aux, double auy, Obstacles* o){
    double lx = o->lx;
    double ly = o->ly;
    double ux = o->ux;
    double uy = o->uy;
    if (LineCollision(alx, aly, aux, auy, lx, ly, lx, uy) ||
            LineCollision(alx, aly, aux, auy, lx, ly, ux, ly) ||
            LineCollision(alx, aly, aux, auy, ux, ly, ux, uy) ||
            LineCollision(alx, aly, aux, auy, lx, uy, ux, uy)) {
        //cout << "found collision" << endl;
        return true;
    }
    return false;
}

/*
    Returns true if node state has no collisions
    currently works for rectangular obstacles
*/
bool Pendulum::CheckCollisions(Node* n) {
    double theta = n->theta;
    double py = -2*d.L*cos(theta);
    double px = (n->x)+2*d.L*sin(theta);
    for (int i = 0; i < obstacles.size(); i++) {
        if (obstacleCollision(n->x, 0, px, py, obstacles[i])){return false;}
    }
    /**/
    //We must also check the subnodes/trajectory nodes    
    vector<state_type> traj = n->trajectory;
    for (int j = 0; j < traj.size(); j++){
        double ttheta = traj[j][2];
        double tpy = -2*d.L*cos(ttheta);
        double tpx = (traj[j][0])+2*d.L*sin(ttheta);
        for (int i = 0; i < obstacles.size(); i++) {
            if (obstacleCollision(traj[j][0], 0, tpx, tpy, obstacles[i])){return false;}
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
    return ((abs(n->x - b.x) < x_epsilon) && (abs(n->v - b.v) < v_epsilon) && (abs(n->theta - b.theta) < theta_epsilon) && (abs(n->w - b.w) < w_epsilon));
}

vector<Node*> Pendulum::FindPathIterations(int iterations) {
    cout << "starting path iteration!\n";
    vector<Node*> goal_nodes;
    double min_cost = DBL_MAX;
    int sol = 0;
    for (int iter = 0; iter < iterations; iter++){
        if(iter%5000==0){cout<<"iterations:"<<iter<<endl;}
        //cout << sol << endl;
        /**/
        Node* prev = GetNodeToExpandRRT();
        double u =((double)rand() / RAND_MAX - 0.5)*max_u; 
        Node* n = d.update(prev, u);        

        /*Test-Node generation
        double testx = 2;
        double testtheta = .1;
        
        vector<double> times;
        times.push_back(0);
        times.push_back(0);
        
        state_type s1;
        s1.push_back(3);
        s1.push_back(0);
        s1.push_back(0);
        s1.push_back(0);

        state_type s2;
        s2.push_back(40);
        s2.push_back(0);
        s2.push_back(2*PI-0.1);
        s2.push_back(0);

        vector<state_type> trajectory;
        trajectory.push_back(s1);
        trajectory.push_back(s2);

        Node* n = new Node(0, testx, 0, testtheta, 0, 0, 0, NULL, times, trajectory);
        n->print_node();
        cout << Feasible(n) << endl;
        */
        /**/
        while (!Feasible(n)){
            //n->print_node();
            //cout << "found unfeasible node!\n";
            int baditer = 1;
            if(baditer%5000==0){cout<<"bad iterations:"<<baditer<<endl;}
            delete n;
            prev = GetNodeToExpandRRT();
            u =((double)rand() / RAND_MAX - 0.5)*max_u;
            n = d.update(prev, u);
        }
        
        prev->add_child(n);    
        
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
            double pt[4] = {n->x*10.0/(bounds->ux-bounds->lx), n->v*10.0/(2*bounds->max_v)+5.0, n->theta*10.0/(6.28318530718), n->w*10.0/(2*bounds->max_w)+5.0};
            if (kd_insert(nodes_tree, pt, n) != 0) {
                cout << "didn't insert root successfully\n";
            }
            nodes.push_back(n);
        }
    }
    cout << "finished iterating!\n";
    cout << "Number of solutions found: " << sol << endl;

    #ifdef OUTNEAREST
    if (sol < 1){
        double pt[4] = {b->x*10.0/(bounds->ux-bounds->lx), b->v*10.0/(2*bounds->max_v)+5.0, b->theta*10.0/(6.28318530718), b->w*10.0/(2*bounds->max_w)+5.0};
        goalnodes.push_back()
    }

    #endif    


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
    vector<Node*> traj;
    if (goal_nodes.size() < 1) {return traj;}
    traj = TraverseNodes(goal_nodes[goal_nodes.size()-1]);
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
    sprintf(filename2,"cost/path_%s_%f.txt", buffer, traj[traj.size()-1]->cost);
    std::ofstream fp_out;
    fp_out.open(filename2,std::ios::out);

    fp_out << b.x << "," << x_epsilon << "," << d.L << endl;
    for (int i=0; i<traj.size(); i++){
        fp_out << traj[i]->print();
    }

    return traj;
}


void Pendulum::PhaseDiagram(int* argc, char** argv){   
    glutInit(argc, argv);                 // Initialize GLUT
    glutInitWindowSize(1280, 1280);   // Set the window's initial width & height
    glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
    glutCreateWindow("Phase Diagram"); // Create a window with the given title
    
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
    glClear(GL_COLOR_BUFFER_BIT); 
    double x_max = bounds->ux-bounds->lx;
    for (int i = 0; i < nodes.size(); i++){
        glBegin(GL_POINTS); // HERE THE POINTS SHOULD BE CREATED
		glPointSize(8);
		glColor3f(1.0f, 0.0f, 0.0f); // Red
		glVertex3f(2.0*nodes[i]->theta/6.28-1.0, nodes[i]->w/(bounds->max_w), 0.0f);
		glEnd();
        glFlush();  // Render now
    }
    glutMainLoop();           // Enter the infinitely event-processing loop
}





/*    
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
