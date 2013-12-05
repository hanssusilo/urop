#include "visualizer.h"
#include <cstring>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <fstream>
#include <iostream>
#include <math.h>

double x, y, 

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
