#ifndef PREPROCESS_TASK
#define PREPROCESS_TASK

#include <vector>
#include <cmath>

#include "models.h"
#include "math_tools.h"

std::vector<Obstacle> preprocess_coastline(const std::vector<Vector>& coastline, double obsts_rad, unsigned int start_id);


std::vector<Vector> get_line_obsts(Vector start, Vector end, double obsts_rad);



#endif // PREPROCESS_TASK
