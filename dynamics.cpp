#ifndef DYNAMICS_CPP
#define DYNAMICS_CPP
#include <iostream>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include "node.h"

#define DT 0.01

using namespace std;
using namespace boost::numeric::odeint;

/*
States
0: x
1: x_dot
2: theta
3: theta_dot
*/


typedef vector<double> state_type;

struct record
{
    vector<state_type>& m_states;
    vector<double>& m_times;

    record(vector<state_type> &states, vector<double> &times)
    : m_states(states), m_times(times) { }

    void operator()(const state_type &y, double t)
    {
        m_states.push_back(y);
        m_times.push_back(t);
    }
};



class Dynamics {
    //PARAMETERS
    const double pi = 3.14159265358979;    //Mass of object (kg)
    const double M = 10.0;    //Mass of object (kg)
    const double m = 2.0;	    //Pendulum mass (kg)
    const double I = 2.0;	    //Moment of Inertia of pendulum (kg*m^2)
    const double G = 9.8;     //Gravitational Acceleration (m/s^2)
    
    
  public:
    const double L = 1;	    //Length of pendulum (m)
    double f;

    Node* update(Node* n, double input){
        f = input;
	    state_type x(4);
        double t1 = n->t;
        double t2 = t1+DT*10;
        double cost = n->cost + (t2-t1)*(t2-t1) + f*f;
        x[0] = n->x;
        x[1] = n->v;
        x[2] = n->theta;
        x[3] = n->w;
	    integrate(*this, x, t1, t2, DT);  
//        cout << t2 << '\t' << '\t' << "x:" <<  x[0] << '\t' << '\t' << "x_d:" << x[1] << '\t' << '\t' << "th:" << x[2] << '\t' << '\t' << "th_d:" << x[3] << endl;
        x[2] = fmod(x[2], 2.0*pi);        
        if (x[2] < 0){
            x[2] = x[2]+2.0*pi;
        }
        return new Node(t2, x[0], x[1], x[2], x[3], input, cost, n);
    }
    
    void operator() ( const state_type &x, state_type &dxdt, const double /*t*/){         
	    dxdt[0] = x[1];
	    dxdt[1] = (((I+m*L*L)*(f+m*L*x[3]*x[3]*sin(x[2]))+(m*m*L*L)*cos(x[2])*sin(x[2])*G)/((M+m)*(I+m*L*L)-(m*m*L*L)*cos(x[2])*cos(x[2])));
	    dxdt[2] = x[3];
	    dxdt[3] = (((-m*L*cos(x[2]))*(f+m*L*x[3]*x[3]*sin(x[2]))+(M+m)*(-m*G*L*sin(x[2])))/((M+m)*(I+m*L*L)-m*m*L*L*cos(x[2])*cos(x[2]))); 
    }
};





/*
#ifndef WRITE_OUTPUT_FUN
#define WRITE_OUTPUT_FUN
void write_output( const state_type &x , const double t ){
	cout << t << '\t' << '\t' << "x:" <<  x[0] << '\t' << '\t' << "x_d:" << x[1] << '\t' << '\t' << "th:" << x[2] << '\t' << '\t' << "th_d:" << x[3] << endl;
}
#endif


int main(int argc, char** argv){
    Dynamics d;
    d.f = 9.8;
	state_type x(4);
    x[0] = 0;
    x[1] = 0;
    x[2] = 0;
    x[3] = 0;
	integrate(d, x, 0.0, 100.0, 0.1, write_output);
    
}












#define M 10	//Mass of object (kg)
#define m 2	//Pendulum mass (kg)
#define L 1	//Length of pendulum (m)
#define I m*L*L	//Moment of Inertia of pendulum (kg*m^2)
#define G 9.8	//Gravitational Acceleration (m/s^2)

 * state update_state(state s, double f, double dt){
	state s_out;
	double x = s.x;
	double v = s.v;
	double theta = s.theta;
	double omega = s.omega;

	double sinth = sin(theta);
	double costh = cos(theta);

	s_out.x = v*dt;
	s_out.v = (((I+m*L*L)*(f+m*L*omega*omega*sinth)+(m*m*L*L)*costh*sinth*G)/((M+m)*(I+m*L*L)-(m*m*L*L)*costh*costh))*dt;
	s_out.theta = omega*dt;
	s_out.omega = (((-m*L*costh)*(f+m*L*theta*theta*sinth)+(M+m)*(-m*G*L*sinth))/((M+m)*(I+m*L*L)-m*m*L*L*costh*costh))*dt;

	return s_out;
}*/

#endif
