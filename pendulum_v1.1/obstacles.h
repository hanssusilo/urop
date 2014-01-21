#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <sstream>
class Obstacles {
  public:
    double lx;
    double ux;
    double ly;
    double uy;
  
    std::string print() {
        stringstream sstm;
        sstm << this->lx << ',' << this->ux << ',' << this->ly << ',' << this->uy << endl;
        return sstm.str();
}
};
#endif
