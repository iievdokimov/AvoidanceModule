#ifndef ALG_CFG
#define ALG_CFG

#include <vector>
#include <string>

class Hyperparams {
public:
    Hyperparams() {};
    Hyperparams(const Hyperparams& other);

    std::string str() const;

//private:
    //steps limit of building route(unsigned int)
    unsigned int max_steps = 200;
    // ship radius(half - length), [meters](unsigned double)
    double ship_radius = 5;

    // radar view distance, [meters](unsigned double)
    // tracking dist (in terms of algo) == radar_radius (in terms of algo modeling process)
    // obstacles out of tracking-distance are ignored by algorithm [meters] (unsigned double)
    // enlarging this parametr leads to productivity loss (cause of counting risk indicies for more obstacles)
    // decreasing this paremetr leads to trajectory-quality loss
    double ship_radar_radius = 187; //750 //550 // 1800

    // minimum distance to obstacle that needs to be maintained[meters](unsigned int)
    double safe_dist = ship_radius * 2;
    // safe time, [seconds](unsigned double)
    double safe_time = 6;
    // maximum possible speed [m / s](unsigned double)
    double max_speed = 10;
    // maximum possible heading angle change [degrees](unsigned double)
    double max_turn_angle = 30;
    // (?) maximum possible speed change [m / s] (unsigned double)
    double max_speed_change = 15;
    // fixed time value that passes each step[seconds(? )](unsigned int)
    double dt = 1.0;
    // min dist to target[meters](unsigned double), after reaching - building route is stopped
    double target_reached_rad = ship_radius * 20;
    // min dist to intermediate target[meters](unsigned double), after reaching - next intermediate target is chosen
    double intermediate_target_reached_rad = ship_radius * 3.0;
    // max angle to intermediate target pos [degrees] (unsigned double)
    double max_angle_to_intermediate_target = 120;
    // dist in which rules must be applied [meters] (unsigned double)
    double rules_application_dist = ship_radar_radius;
    // dist ourside of which static obstacles are ignored [meters] (unsigned double)
    double ignore_static_obst_dist = ship_radar_radius; //ship_radius * 7;

    // modes
    bool follow_trajectory_mode = false;
    bool rules_application_mode = false;

    // optimization weights
    std::vector<double> opt_vel_weights{ 0.155, 0.665, 0.095, 0.02, 0.065 };
    std::vector<double> risk_weights{ 0.4, 0.367, 0.167, 0.033, 0.033 };

    // optimization modes
    bool ignore_VO_static_obsts = true;

    bool estimate_given_trajectory = false;
};



#endif //ALG_CFG