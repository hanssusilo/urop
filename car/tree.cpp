#include "tree.h"
#include <cstring>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <fstream>
#include <iostream>
#include <math.h>
int it = 1; // used for stepping through paths
double x, y, viewDistance;
Point start, goal;
Est* e;
bool iterationsOn;
int iterations;
vector<Node*> tree;
vector<Node*> solution;

void set_bounds() {
  char value[100];
  double x1, x2, y1, y2;
  ifstream fp_in("bounds.txt");
  fp_in.getline(value,100,',');
  x1 = atof(value);
  fp_in.getline(value,100,',');
  y1 = atof(value);
  fp_in.getline(value,100,',');
  x2 = atof(value);
  fp_in.getline(value,100);
  y2 = atof(value);
  fp_in.close();
  x = x2-x1;
  y = y2-y1;
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

void draw_obstacles() {
  glColor3f(0.3,0.3,0.3);
  ifstream fp_in("obstacles.txt");
  char value[100];	
  while (fp_in.getline(value, 100, ',')) {
    double x1, y1, x2, y2;
    x1 = atof(value);
    fp_in.getline(value,100,',');
    y1 = atof(value);
    fp_in.getline(value,100,',');
    x2 = atof(value);
    fp_in.getline(value,100);
    y2 = atof(value);		
    draw_box(x1,y1,x2,y2);
  }
  fp_in.close();
}

void draw_line(double x1, double y1, double x2, double y2) {
  glBegin(GL_LINES);
  glVertex3f(x1,y1,0.0);
  glVertex3f(x2,y2,0.0);
  glEnd();
}

void draw_field() {
  glColor3f(1.0,1.0,1.0);
  double size_x = viewDistance*0.4*x/y, size_y = viewDistance*0.4;
  glBegin(GL_TRIANGLES);
  glVertex3f(-size_x,-size_y,0.0);
  glVertex3f(-size_x,size_y,0.0);
  glVertex3f(size_x,-size_y,0.0);		
  glVertex3f(size_x,size_y,0.0);
  glVertex3f(-size_x,size_y,0.0);
  glVertex3f(size_x,-size_y,0.0);
  glEnd();
}

void render_scene() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  gluLookAt(0.0f, 0.0f, viewDistance,
            0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f);
  glTranslatef(0.0,0.0,-5.0);
  draw_field();
  glTranslatef(-x/2, -y/2, 0.0f);
  draw_obstacles();
  glLineWidth(1.0);

  // draw starting and ending points
  glPushMatrix();
  glTranslatef(start.x, start.y, 1.0);
  glColor3f(1.0,0.0,0.0);
  glutSolidSphere(viewDistance/500,20,2);
  glPopMatrix();
  glPushMatrix();
  glTranslatef(goal.x, goal.y, 1.0);
  glColor3f(0.0,1.0,0.0);
  glutSolidSphere(viewDistance/48,20,2);
  glPopMatrix();
//  for(int i=0; i<30000;i++) {draw_line(10, 10, 50, log(50+i));}
    

  // draw the tree so far, in order of nodes as they were added
  for (int i = 0; i < it; i++) {
    Node* n = tree[i];
    // draw the added node from expansion
  //  glPushMatrix();
  //  glTranslatef(n->x, n->y, 1.0);
 //   glColor3f(0.0,0.0,0.0);
 //   glutSolidSphere(viewDistance/1000,20,2);
 //   glPopMatrix();
    // draw a line from the node to its parent

    if (n->parent != NULL) {
      Node* prev = n->parent;
      for (int j = 0; j < n->subnodes.size(); j++) {
        Node* cur = n->subnodes[j];
     //   glPushMatrix();
     //   glTranslatef(cur->x, cur->y, 1.0);
     //   glColor3f(0.3,0.3,0.3);
     //   glutSolidSphere(viewDistance/1100,20,2);
     //   glPopMatrix();
    //    glColor3f(0.0,0.0,1.0);
        draw_line(prev->x, prev->y, cur->x, cur->y);
        prev = cur;
      } 
      glColor3f(0.0,0.0,1.0);
      draw_line(prev->x, prev->y, n->x, n->y);

    }
  }
 
  if (it < tree.size()) {
    it++;
  } else if (solution.size() != 0) {
    glLineWidth(3.0);
    glColor3f(1.0,0.0,0.0);
    int j = solution.size()-1;
    Node* cur = solution[j];
    while (j > 0) {
      vector<Node*> subnodes = cur->subnodes;
      Node* sub = cur;
      for (int i = (int)subnodes.size()-1; i >= 0; i--) {
        draw_line(sub->x, sub->y, subnodes[i]->x, subnodes[i]->y);
        sub = subnodes[i];
      }
      cur = solution[--j];
      draw_line(sub->x, sub->y, cur->x, cur->y);
    }
  }

  glPushMatrix();
  glTranslatef(goal.x, goal.y, 1.0);
  glColor3f(0.0,1.0,0.0);
  glutSolidSphere(viewDistance/48,20,2);
  glPopMatrix();


  glutSwapBuffers();
}

void change_size(int w, int h) {
  // Prevent a divide by zero, when window is too short
  // (you cant make a window of zero width).
  if (h == 0) h = 1;
  float ratio = 1.0 * w / h;

  // Use the Projection Matrix
  glMatrixMode(GL_PROJECTION);

  // Reset Matrix
  glLoadIdentity();

  // Set the viewport to be the entire window
  glViewport(0, 0, w, h);

  // Set the correct perspective.
  gluPerspective(45,ratio,1,1000);

  // Get Back to the Modelview
  glMatrixMode(GL_MODELVIEW);
}

void processKeys(unsigned char key, int x, int y) {
  if (key == 27) { // esc
    glutLeaveMainLoop();
  } else if (key == 32) { // spacebar
    if (iterationsOn) {
      solution = e->FindPathIterations(iterations);
      tree = e->GetNodes();
      it = 1;
    } else {
      solution = e->FindPathSolutions(iterations);
      tree = e->GetNodes();
      it = 1;
    }
  }
  glutPostRedisplay();
}

Tree::Tree(int argc, char* argv[]) {
  if (argc != 3) {
    cout << "Wrong number of arguments, exiting.\n";
    exit(0);
  }
  Node* root = new Node(45.0, 0.0, PI/2, 0.0, NULL);
  start.x = root->x;
  start.y = root->y;
  goal.x = 5.0;
  goal.y = 65.0;
  // find a path between start and goal using the specified options
  e = new Est(root, goal);
  if (strcmp(argv[1], "-i") == 0) {
    iterationsOn = true;
    iterations = atoi(argv[2]);
    solution = e->FindPathIterations(iterations);
  } else {
    iterationsOn = false;
    iterations = atoi(argv[2]);
    solution = e->FindPathSolutions(iterations);
  }
  tree = e->GetNodes();

  set_bounds();
  viewDistance = (x > y) ? x*1.4 : y*1.4;

  // init GLUT and create Window
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
  glutInitWindowPosition(1350,0);
  glutInitWindowSize(800,800*y/x);
  glutCreateWindow("Point to Point Path Planner");

  // register callbacks
  glutDisplayFunc(render_scene);
  glutReshapeFunc(change_size);
  glutKeyboardFunc(processKeys);
  glutIdleFunc(render_scene);///Geo

  // enter GLUT event processing cycle
  glutMainLoop();
}
