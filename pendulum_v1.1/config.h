#ifndef CONFIG_H
#define CONFIG_H
class Config {
  public:
    double x_epsilon, v_epsilon, theta_epsilon, w_epsilon;
    int method_of_expansion, verbose;
    bool phase_diagram, debug, node_gen;
};
#endif
