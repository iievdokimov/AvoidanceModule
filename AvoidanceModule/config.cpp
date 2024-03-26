#include "config.h"

#include <iostream>

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

std::string Hyperparams::str() const
{
    std::string res = "Hyperparams(\n";
    res = res + "dt = " + std::to_string(dt) + "\n";
    res = res + "max_steps = " + std::to_string(max_steps) + "\n";
    res = res + "ship_radius = " + std::to_string(ship_radius) + "\n";
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
