#include <global_planner/potential_calculator.h>

namespace global_planner{

PotentialCalculator::~PotentialCalculator(){}

float PotentialCalculator::calculatePotential(float* potential, unsigned char cost, int n, float prev_potential)
{
    if(prev_potential < 0)
    {
        // get min of neighbors
        float min_h = std::min( potential[n - 1], potential[n + 1] );
        float min_v = std::min( potential[n - nx_], potential[n + nx_]);
        prev_potential = std::min(min_h, min_v);
    }

    return prev_potential + cost;
}

void PotentialCalculator::setSize(int nx, int ny)
{
    nx_ = nx;
    ny_ = ny;
    ns_ = nx * ny;
} /**< sets or resets the size of the map */

}
