#include "visualizer.h"
#include <cstring>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <vector>
#include <unistd.h>
#include "../node.h"
#include "../bounds.h"


#define W 2400


double x;
double x_goal;
double x_epsilon;
double l;
bool space_pressed = false;


vector<Node*> nodes;

void set_bounds() {
    char value[100];
    double x1, x2, y1, y2;
    ifstream fp_in("../bounds.txt");
    fp_in.getline(value,100,',');
    x1 = atof(value);
    fp_in.getline(value,100,',');
    x2 = atof(value);
    fp_in.close();
    x = x2-x1;
    //cout << "bounds:" << x1 << " " << x2 << endl;
}

void load_nodes(char* filename) {
    char value[100];
    Node* n;
    ifstream in(filename);
    if (in.is_open()) {
        in.getline(value, 100, ',');
        x_goal = atof(value);
        in.getline(value, 100, ',');
        x_epsilon = atof(value);
        in.getline(value, 100);
        l = atof(value);
        
        while (in.getline(value, 100, ',')) {
            n = new Node();

            n->t = atof(value);
            in.getline(value, 100, ',');
            n->x = atof(value);
            in.getline(value, 100, ',');
            n->v = atof(value);
            in.getline(value, 100, ',');
            n->theta = atof(value);
            in.getline(value, 100, ',');
            n->w = atof(value);
            in.getline(value, 100, ',');
            n->u = atof(value);
            in.getline(value, 100);
            n->cost = atof(value);

            nodes.push_back(n);
            //n->print_node();
        }
        in.close();
    } else {
        cout << "Unable to open file";
    }
}

void draw_box(double x1, double y1, double x2, double y2) {
    glBegin(GL_POLYGON);
    glNormal3f(0.0f,0.0f,1.0f);
    glVertex3f(x1,y1,0.0f);
    glVertex3f(x2,y1,0.0f);
    glVertex3f(x2,y2,0.0f);
    glVertex3f(x1,y2,0.0f);
    glEnd();
}

void draw_line(double x1, double y1, double x2, double y2) {
    glBegin(GL_LINES);
    glVertex3f(x1,y1,0.0);
    glVertex3f(x2,y2,0.0);
    glEnd();
}

void draw_obstacles() {
    glColor3f(1.0,0.0,0.0);
    ifstream fp_in("../obstacles.txt");
    char value[100];	
    while (fp_in.getline(value, 100, ',')) {
        double x1, y1, x2, y2;
        x1 = atof(value);
        fp_in.getline(value,100,',');
        x2 = atof(value);
        fp_in.getline(value,100,',');
        y1 = atof(value);
        fp_in.getline(value,100);
        y2 = atof(value);
        //cout << "obstacles:" << x1*2.0/x-1 << " " << x2*2.0/x-1 << " " << y1/l << " " << y2/l << endl;
        draw_box(x1*2.0/x-1,y1*6.0/x,x2*2.0/x-1,y2*6.0/x);
    }
    fp_in.close();
}

void draw_node(Node* n) {
    glColor3f(0.0,0.0,1.0);
    //cout << (n->x)*2.0/x-1 << "  " << endl;
    draw_box((n->x-.5)*2.0/x-1, -.25*6.0/x, (n->x+.5)*2.0/x-1, .25*6.0/x);
    glColor3f(1.0,1.0,1.0);
    glLineWidth(2.5);
    draw_line((n->x)*2.0/x-1, 0, ((n->x)+l*sin(n->theta))*2.0/x-1, (-l*cos(n->theta))*6.0/x);
    glLineWidth(1.f);
    glColor3f(1.0,0.0,1.0);
    //cout << n->u << endl;
    draw_line((n->x)*2.0/x-1, 0, (n->x+n->u/300.0)*2.0/x-1, 0);
}

void draw_goal() {
    glColor3f(0.0,1.0,0.0);
    draw_box((x_goal-x_epsilon)*2.0/x-1, -.25*6.0/x, (x_goal+x_epsilon)*2.0/x-1, .25*6.0/x);
}


/* Handler for window-repaint event. Call back when the window first appears and
   whenever the window needs to be re-painted. */
static void display(int i) {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black/opaque
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear buffers

    draw_goal();
    draw_obstacles(); 
    draw_node(nodes[i]);

    glutSwapBuffers();  // Render now

    if (space_pressed){
        i = 0;
        space_pressed = false;
    }
    if ((i+1) < nodes.size()){
        i++;
    }
    glutTimerFunc(100, display, i);
}


void processKeys(unsigned char key, int x, int y) {
    if (key == 27) { // esc
        glutLeaveMainLoop();
    } else if (key == 32) { // spacebar
        space_pressed = true;
    }
}


/* Main function: GLUT runs as a console application starting at main()  */
int main(int argc, char** argv) {
    /**/
    set_bounds();
    load_nodes(argv[1]);
    
    glutInit(&argc, argv);                 // Initialize GLUT
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(W/2.0, W/6.0);   // Set the window's initial width & height
    glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
    glutCreateWindow("Pendulum Simulation"); // Create a window with the given title
    
    glutTimerFunc(100, display, 0);
    glutKeyboardFunc(processKeys);
    glutMainLoop();           // Enter the infinitely event-processing loop

    return 0;
}
