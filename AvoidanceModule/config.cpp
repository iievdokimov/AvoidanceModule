#include "config.h"

Hyperparams::Hyperparams(double scale){
    ship_radius *= scale;
    ship_radar_radius *= scale;
    safe_dist *= scale;
    max_speed *= scale;
    max_speed_change *= scale;
    safe_time *= 1;
    target_reached_rad *= scale;
    intermediate_target_reached_rad *= scale;
    rules_application_dist *= scale;
}
