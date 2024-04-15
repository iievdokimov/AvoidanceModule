#include "config.h"

/*
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
*/

std::string Hyperparams::str() const
{
    std::string res = "Hyperparams(\n";
    res = res + "dt = " + std::to_string(dt) + "\n";
    res = res + "max_steps = " + std::to_string(max_steps) + "\n";
    res = res + "ship_radius = " + std::to_string(ship_radius) + "\n";
    res = res + "tracking_dist = " + std::to_string(ship_radar_radius) + "\n";
    res = res + "safe_dist = " + std::to_string(safe_dist) + "\n";
    res = res + "safe_time = " + std::to_string(safe_time) + "\n";
    res = res + "max_speed = " + std::to_string(max_speed) + "\n";
    res = res + "max_turn_angle = " + std::to_string(max_turn_angle) + "\n";
    res = res + "max_speed_change = " + std::to_string(max_speed_change) + "\n";

    res = res + "\nReached rad-s:\n";
    res = res + "target_reached_rad = " + std::to_string(target_reached_rad) + "\n";
    res = res + "intermediate_target_reached_rad = " + std::to_string(intermediate_target_reached_rad) + "\n";
    res = res + "max_angle_to_intermediate_target = " + std::to_string(max_angle_to_intermediate_target) + "\n";
    res = res + "rules_application_dist = " + std::to_string(rules_application_dist) + "\n";

    res = res + "\nModes:\n";
    res = res + "follow_trajectory_mode = " + std::to_string(follow_trajectory_mode) + "\n";
    res = res + "rules_application_mode = " + std::to_string(rules_application_mode) + "\n";

    res = res + "\nWeights:\n";
    res = res + "opt_vel_weights = { ";
    for (auto weight : opt_vel_weights) {
        res = res + std::to_string(weight) + ", ";
    }
    res = res + "}\n";
    res = res + "risk_weights = { ";
    for (auto weight : risk_weights) {
        res = res + std::to_string(weight) + ", ";
    }
    res = res + "}\n";

    res = res + "\n)";

    return res;
}

Hyperparams::Hyperparams(const Hyperparams& other)
{
    max_steps = other.max_steps;
    ship_radius = other.ship_radius;
    ship_radar_radius = other.ship_radar_radius;
    safe_dist = other.safe_dist;
    safe_time = other.safe_time;
    max_speed = other.max_speed;
    max_turn_angle = other.max_turn_angle;
    max_speed_change = other.max_speed_change;
    dt = other.dt;
    target_reached_rad = other.target_reached_rad;
    intermediate_target_reached_rad = other.intermediate_target_reached_rad;
    max_angle_to_intermediate_target = other.max_angle_to_intermediate_target;
    rules_application_dist = other.rules_application_dist;
    ignore_static_obst_dist = other.ignore_static_obst_dist;
    follow_trajectory_mode = other.follow_trajectory_mode;
    rules_application_mode = other.rules_application_mode;
    opt_vel_weights = other.opt_vel_weights;
    risk_weights = other.risk_weights;
    ignore_VO_static_obsts = other.ignore_VO_static_obsts;
    estimate_given_trajectory = other.estimate_given_trajectory;
}