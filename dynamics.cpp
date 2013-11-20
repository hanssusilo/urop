#include<iostream>
#include <math.h>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

using namespace std;
using namespace boost::numeric::odeint;


//PARAMETERS
#define M 10	//Mass of object (kg)
#define m 2		//Pendulum mass (kg)
#define L 1		//Length of pendulum (m)
#define I m*L*L	//Moment of Inertia of pendulum (kg*m^2)
#define G 9.8	//Gravitational Acceleration (m/s^2)

typedef std::array< double ,4> state_type;

class state {
  double x,v,theta,omega;
public:
  state(double _x, double _v, double _theta, double _omega) : x(_x),v(_v),theta(_theta),omega(_omega) { }
  double f = 1;
  void operator ()( const state_type &x, state_type &dxdt, const double /*t*/){
	  dxdt[0] = x[1];
	  dxdt[1] = (((I+m*L*L)*(f+m*L*omega*omega*sin(x[2]))+(m*m*L*L)*cos(x[2])*sin(x[2])*G)/((M+m)*(I+m*L*L)-(m*m*L*L)*cos(x[2])*cos(x[2])));
	  dxdt[2] = x[3];
	  dxdt[3] = (((-m*L*cos(x[2]))*(f+m*L*theta*theta*sin(x[2]))+(M+m)*(-m*G*L*sin(x[2])))/((M+m)*(I+m*L*L)-m*m*L*L*cos(x[2])*cos(x[2])));
  }
} ;


void write_output( const state_type &x , const double t )
{
    cout << t << '\t' << x[0] << '\t' << x[1] << '\t' << x[2] << '\t' << x[3] << endl;
}


int main(int argc, char* argv[]){
	state s(1,0, 1, 0);
	integrate( state, x, 0.0 , 25.0 , 0.1 , write_output );
}

/*
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
