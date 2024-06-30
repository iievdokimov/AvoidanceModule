#include "dynamic_models.h"


std::vector<Vector> CasualDynamicModel::all_possible_velocities() const
{
	//res.push_back({ 20, -20 });
	double len_vec = ship.max_speed();
	if (len_vec == 0) {
		return {};
	}
	std::vector<Vector> possible_velocities;


    // possible velocities sector 
    double max_turn_angle = possible_turns_sector / 2;
    int num_vel_slops = 6;
    int num_vel_circle = 61;
    double angle_step_slope = max_turn_angle / (num_vel_slops - 1);
    double angle_step_circle = 360.0 / (num_vel_circle - 1);

    Vector axis = prev_nonzero_vel();

    return get_sector_vecs(axis, angle_step_slope, angle_step_circle, max_turn_angle);


    //std::vector<double> speed_coef = {1, 0.75, 0.5, 0.25};

    //for (auto c : speed_coef) {
    //    for (const auto& el : get_sector_vecs(left_angle, left_angle + sector, len_vec*c, angle_step)) {
    //        possible_velocities.push_back(el);
    //    }
    //} 
    //return possible_velocities;
}