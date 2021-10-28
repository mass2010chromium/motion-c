#pragma once
#include <math.h>

#define UR5_CONFIG_LEN 6
#define UR5_CL UR5_CONFIG_LEN

const double MAX_JOINTS[UR5_CONFIG_LEN] = {2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI};
const double MIN_JOINTS[UR5_CONFIG_LEN] = {-2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI, -2*M_PI};
const double MAX_VEL[UR5_CONFIG_LEN] = {M_PI, M_PI, M_PI, M_PI, M_PI, M_PI};
const double MIN_VEL[UR5_CONFIG_LEN] = {-M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI};

#define PROTECTIVE_STOP_MASK 0b100
