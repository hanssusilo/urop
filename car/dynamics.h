#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <boost/numeric/odeint/integrate/integrate_n_steps.hpp>
#include <boost/numeric/odeint.hpp>

using namespace std;
using namespace boost::numeric::odeint;

//[ rhs_function
/* The type of container used to hold the state vector */
typedef vector<double> state_type;

class dynamics {
  double m11, m22, m33, d11, d22, d33, F, u, dz;
 public:
  dynamics(double _F, double _u, double _dz) {
    F = _F;
    u = _u;
    dz = _dz;
    m11 = 50;
    m22 = 50;
    m33 = 50;//USED TO BE 2
    d11 = 50;
    d22 = 50;
    d33 = 100; // USED TO BE 10
  }
  /* The rhs of x' = f(x) */
  void operator() (const state_type &y, state_type &dydt, const double /*t*/)
  {
    dydt[0] = (m22/m11)*y[1]*y[2] - (d11/m11)*y[0] + F*cos(u)/m11;
    dydt[1] = (-1.0*m11/m22)*y[0]*y[2] - (d22/m22)*y[1] - F*sin(u)/m22;
    dydt[2] = ((m11-m22)/m33)*y[0]*y[1] - (d33/m33)*y[2] + 1.0*F*sin(u)/m33;
    dydt[3] = y[0]*cos(y[5]) - y[1]*sin(y[5]);
    dydt[4] = y[0]*sin(y[5]) + y[1]*cos(y[5]);
    dydt[5] = y[2];
    dydt[6] = dz;
  }
//]
};

//[ integrate_observer
struct push_back_state_and_time
{
    vector<state_type>& m_states;
    vector<double>& m_times;

    push_back_state_and_time(vector<state_type> &states, vector<double> &times)
    : m_states(states), m_times(times) { }

    void operator()(const state_type &y, double t)
    {
        m_states.push_back(y);
        m_times.push_back(t);
    }
};
//]
