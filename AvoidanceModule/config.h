#ifndef ALG_CFG
#define ALG_CFG

#include <vector>

class Hyperparams {
public:
    Hyperparams(double scale);

//private:
    //steps limit of building route(unsigned int)
    const unsigned int max_steps = 60;
    // ship radius(half - length), [meters](unsigned double)
    double ship_radius = 20;

    // radar view distance, [meters](unsigned double)
    // tracking dist (in terms of algo) == radar_radius (in terms of algo modeling process)
    // obstacles out of tracking-distance are ignored by algorithm [meters] (unsigned double)
    // enlarging this parametr leads to productivity loss (cause of counting risk indicies for more obstacles)
    // decreasing this paremetr leads to trajectory-quality loss
    double ship_radar_radius = 750; //#550 # 1800

    // minimum distance to obstacle that needs to be maintained[meters](unsigned int)
    double safe_dist = ship_radius;
    // safe time, [seconds](unsigned double)
    double safe_time = 6;
    // maximum possible speed [m / s](unsigned double)
    double max_speed = 40;
    // maximum possible heading angle change [degrees](unsigned double)
    double max_turn_angle = 30;
    // (?) maximum possible speed change [m / s] (unsigned double)
    double max_speed_change = 15;
    // fixed time value that passes each step[seconds(? )](unsigned int)
    // dt = 1
    // min dist to target[meters](unsigned double), after reaching - building route is stopped
    double target_reached_rad = ship_radius * 20;
    // min dist to intermediate target[meters](unsigned double), after reaching - next intermediate target is chosen
    double intermediate_target_reached_rad = ship_radius * 12;
    // max angle to intermediate target pos [degrees] (unsigned double)
    double max_angle_to_intermediate_target = 120;
    // dist in which rules must be applied [meters] (unsigned double)
    double rules_application_dist = ship_radar_radius;
    // modes
    bool follow_trajectory_mode = false;

    const std::vector<double> opt_vel_weights{ 0.155, 0.665, 0.095, 0.02, 0.065 };
    const std::vector<double> risk_weights{ 0.4, 0.367, 0.167, 0.033, 0.033 };

};



#endif //ALG_CFG