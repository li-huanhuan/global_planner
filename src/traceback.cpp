#include <global_planner/traceback.h>

namespace global_planner{

Traceback::Traceback(){}

Traceback::Traceback(PotentialCalculator* p_calc) : p_calc_(p_calc) {}

Traceback::~Traceback(){}

}
