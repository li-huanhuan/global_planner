#include <global_planner/expander.h>

namespace global_planner{

Expander::Expander(PotentialCalculator* p_calc, int nx, int ny) :
  unknown_(true), lethal_cost_(253), neutral_cost_(50),
  factor_(3.0), p_calc_(p_calc)
{
    setSize(nx, ny);
}

Expander::~Expander(){}

void Expander::setSize(int nx, int ny)
{
    nx_ = nx;
    ny_ = ny;
    ns_ = nx * ny;
}

void Expander::clearEndpoint(unsigned char* costs, float* potential, int gx, int gy, int s)
{
    int startCell = toIndex(gx, gy);
    for(int i=-s;i<=s;i++)
    {
      for(int j=-s;j<=s;j++)
      {
          int n = startCell+i+nx_*j;
          if(potential[n]<POT_HIGH)
              continue;
          float c = costs[n]+neutral_cost_;
          float pot = p_calc_->calculatePotential(potential, c, n);
          potential[n] = pot;
      }
    }
}


}
