#ifndef ALG_USAGE
#define ALG_USAGE

#include <iostream>
#include <vector>
#include <fstream>

#include "task.h"
#include "algorithm_base.h"


void build_traj();

void write_traj(const std::vector<ModelState>& traj);

//void estimate_traj();

#endif //ALG_USAGE
