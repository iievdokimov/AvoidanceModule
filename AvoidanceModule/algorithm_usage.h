#ifndef ALG_USAGE
#define ALG_USAGE

#include <iostream>
#include <vector>
#include <fstream>

#include "task.h"
#include "algorithm_base.h"


void build_traj();

void write_traj(const std::vector<ModelState>& traj);


std::vector<ModelState> fake_trajectory(Vector ship_pos, Vector final_target, int steps);


std::vector<Vector> fake_follow_targets(Vector ship_pos, Vector final_target, int steps);

//void estimate_traj();

#endif //ALG_USAGE
