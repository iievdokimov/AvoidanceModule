#ifndef RISK_IND
#define RISK_IND

#include <algorithm>

#include "models.h"
#include "config.h"
#include "math_tools.h"


double collision_risk(const Ship& ship, Vector velocity, const Obstacle& obst, const Hyperparams& hyperparams);

std::pair<double, double> calculate_dcpa_tcpa(const Ship& ship, Vector velocity, const Obstacle& obst);

//double old_generalized_rating_func(double arg, double arg_bound, double mult);

double f3_generalized_rating_func(double x, double x_safe, double x_track);

double dcpa_index(double dcpa, double safe_dist);

double tcpa_index(double tcpa, double safe_time);

double dist_index(double dist, double safe_dist);

double rp_index(double relative_pos_angle);

double speed_ratio_index(double speed_ratio);

double dcpa_static_index(double dcpa, double safe_dist);

double tcpa_static_index(double tcpa, double safe_time);

double dist_static_index(double dist, double safe_dist);

double rp_static_index(double relative_pos_angle);


#endif //RISK_IND