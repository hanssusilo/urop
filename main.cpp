#include <cstdlib>
#include <cstring>
#include <iostream>
#include <math.h>

#include "node.h"
#include "pendulum.h"
#include "point.h"

using namespace std;

int main(int argc, char* argv[]) {
    //cout << (fmod((-6.3*3.14),(2*3.14))/3.14) << endl;
    if (argc != 4) {
        cout << "Wrong number of arguments, exiting.\n";
        exit(0);
    }
    Point goal;
    int iterations = atoi(argv[1]);
    goal.x = atof(argv[2]);
    goal.v = atof(argv[3]);
    Node* start = new Node(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, NULL);
    Pendulum* p = new Pendulum(start, goal);
    p->FindOptimalPathIterations(iterations);
}
