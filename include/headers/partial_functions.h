#ifndef MULTIPLE_TURTLEBOTS_PARTIAL_FUNCTIONS_H
#define MULTIPLE_TURTLEBOTS_PARTIAL_FUNCTIONS_H

#include "DakaiAlgo.h"
#include "helper_functions.h"

double partialInterrobotCollisionPotential(double z, const Variables &v); // potential function depending on interrobot distance

double partialObstacleCollisionPotential(double z, const Variables& v); // potential function depending on distance to obstacles

double partialLOSPreservePotential(double z, const Variables& v); // potential function depending on LOS preservation

double partialCohesionPotential(double z, const Variables& v); // potential function of group cohesion

#endif //MULTIPLE_TURTLEBOTS_PARTIAL_FUNCTIONS_H
