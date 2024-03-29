#include <cstdlib>
#include <cstring>
#include <iostream>
#include <math.h>

#include "node.h"
#include "pendulum.h"
#include "point.h"

using namespace std;

#ifndef st_type
#define st_type
typedef vector<double> state_type;
#endif 

int main(int argc, char* argv[]) {
    //cout << (fmod((-6.3*3.14),(2*3.14))/3.14) << endl;
    Point goal;
    int iterations;
    if (argc == 6) {
        iterations = atoi(argv[1]);
        goal.x = atof(argv[2]);
        goal.v = atof(argv[3]);
        goal.theta = atof(argv[4]);
        goal.w = atof(argv[5]);
    } else{
        cout << "Wrong number of arguments, exiting.\n";
        exit(0);
    }

    vector<double> times;
    vector<state_type> trajectory;

    Node* start = new Node(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, NULL, times, trajectory);
    Pendulum* p = new Pendulum(start, goal);
    p->FindOptimalPathIterations(iterations);
    p->PhaseDiagram(&argc, argv);
}
