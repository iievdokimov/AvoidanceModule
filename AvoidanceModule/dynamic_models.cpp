#include "dynamic_models.h"


std::vector<Vector> CasualDynamicModel::all_possible_velocities() const
{
	//res.push_back({ 20, -20 });
	double len_vec = ship.max_speed();
	if (len_vec == 0) {
		return {};
	}
	std::vector<Vector> possible_velocities;

    double alfa = get_course_angle();
    // possible velocities sector : alfa + -20 degrees
    double sector = possible_turns_sector;
    int num_vel = 18;
    double angle_step = sector / (num_vel - 1);
    double left_angle = alfa - possible_turns_sector / 2;

    std::vector<double> speed_coef = {1, 0.75, 0.5, 0.25};

    for (auto c : speed_coef) {
        for (const auto& el : get_sector_vecs(left_angle, left_angle + sector, len_vec*c, angle_step)) {
            possible_velocities.push_back(el);
        }
    } 
    return possible_velocities;
}

double CasualDynamicModel::get_course_angle() const
{
    Vector vel = ship.vel();
    double alfa;
    if (vel.x() == 0 and vel.y() == 0) {
        // default vec
        vel = prev_nonzero_vel();
        if (vel.x() == 0 and vel.y() == 0) {
            //  throw std::runtime - exist only if construct ship with vel=(0, 0) which is forbidden
            //  no adequate soluituion 
            return 0;
        }
        alfa = deg_clockwise_angle(_Ox_axis, vel);
    }
    else {
        alfa = deg_clockwise_angle(_Ox_axis, vel);
    }
    return alfa;
}
