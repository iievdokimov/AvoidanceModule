#include "algorithm_base.h"
#include <iostream>


TrajectoryBuilder::TrajectoryBuilder(Task task, Hyperparams hyperparams) :
	ship{ task.ship() }, final_target{ task.target() }, obst_list{ task.obst_list() },
	hyperparams{ hyperparams }, dynamic_model{ ship, hyperparams.max_turn_angle * 2 },
	follow_targets_list{ follow_targets_list }, init_task{ task }
{
	_is_finished = false;
	_cur_step = 0;
	_chek_local_traj_mode = false;


	_collision_happened = false;
	_stop_happened = false;
	_unsafe_happened = false;
	_target_reached = false;

	_data_radius = ship.pos().x(); // points_dist(ship.pos(), final_target);
	_min_obst_dist_static = points_dist(ship.pos(), final_target);
	_min_obst_dist_dynamic = points_dist(ship.pos(), final_target);
	_route_length = 0;
	_route_time = 0;
	_max_collision_risk = 0;

	_step_vel_est = {};
	
	//configure_follow_targets(task.cur_trajectory());
	follow_targets_list = task.follow_targets();
	follow_target_idx = 0;
	if (hyperparams.follow_trajectory_mode && follow_targets_list.size() > 0) {
		follow_target = follow_targets_list[follow_target_idx];
	}
}

TrajectoryEstimator comparison_build(Task task)
{
	if (task.follow_targets().size() < 2) {
		//return TrajectoryEstimator();
		//throw Unfilled follow targets
		std::cout << "Comparison build error: Unfilled follow targets" << std::endl;
	}
	Hyperparams hyperparams_new;
	hyperparams_new.estimate_given_trajectory = false;
	hyperparams_new.follow_trajectory_mode = true;
	TrajectoryBuilder builder_new(task, hyperparams_new);
	Hyperparams hyperparams_cur;
	hyperparams_cur.estimate_given_trajectory = true;
	hyperparams_cur.follow_trajectory_mode = true;
	TrajectoryBuilder estimator_cur(task, hyperparams_cur);
	auto new_result = builder_new.get_full_trajectory();
	auto cur_result = estimator_cur.get_full_trajectory();
	TrajectoryEstimator result(new_result.first, new_result.second, cur_result.first, cur_result.second);
	return result;
}


std::pair<std::vector<ModelState>, FinishLog> TrajectoryBuilder::get_full_trajectory()
{
	auto start = std::chrono::steady_clock::now();
	while (!_is_finished && _cur_step < hyperparams.max_steps) {
		next_step();
	}
	//if (!_is_finished) { }

	auto end = std::chrono::steady_clock::now();
	auto diff = end - start;
	double seconds = std::chrono::duration<double>(diff).count();

	RouteQualityData quality_data;
	quality_data.max_collision_risk = _max_collision_risk;
	quality_data.min_obst_dist_dynamic = _min_obst_dist_dynamic;
	quality_data.min_obst_dist_static = _min_obst_dist_static;
	quality_data.route_length = _route_length;
	quality_data.route_time = _route_time;
	
	TaskObstsInfo obsts_info = get_obsts_info(_data_radius);
	FinishLog finish_log(_target_reached, _cur_step, seconds, _events, quality_data, obsts_info, hyperparams);
	return { ship.traj(), finish_log };
}


void TrajectoryBuilder::next_step(){
	if (_is_finished) return;

	//std::cout << "step: " << _cur_step << "max_steps: " << hyperparams.max_steps << std::endl;
	
	
	Vector best_velocity = choose_velocity();
	Vector prev_ship_pos = ship.pos();
	ship.set_vel(best_velocity);
	// estimate cr before moving
	_max_collision_risk = cur_collision_risk();
	_move_all(hyperparams.dt);

	_route_length += ship.pos().sub(prev_ship_pos).magnitude();
	_route_time += hyperparams.dt;
	
	//std::cout << ship.str() << std::endl;
	if (hyperparams.follow_trajectory_mode) {
		update_follow_target();
	}

	update_step_flags();
	fix_step_events();
	//std::cout << ship.pos().str() << " " << final_target.str() << std::endl;
	//std::cout << _is_finished << " " << _cur_step << std::endl;
	_cur_step++;	
}


void TrajectoryBuilder::_move_all(int steps, double step_t)
{
	ship.move(steps, step_t);
	for (int i = 0; i < obst_list.size(); ++i) {
		obst_list[i].move(steps, step_t);
	}
}

bool TrajectoryBuilder::in_tracking_dist(const Obstacle& obst) const
{
	//std::cout << obst_list.size() << std::endl;
	//std::cout << obst.str() << " " << points_dist(ship.pos(), obst.pos()) - obst.rad() << " " << ship.radar_rad() << std::endl;
	if (points_dist(ship.pos(), obst.pos()) - obst.rad() < ship.radar_rad()) {
		return true;
	}
	return false;
}

void TrajectoryBuilder::set_follow_targets(std::vector<Vector> trajectory)
{
	follow_targets_list = trajectory;
	follow_target_idx = 0;
	if (follow_targets_list.size() == 0) {
		return;
	}
	follow_target = follow_targets_list[follow_target_idx];
	update_follow_target();
}

void TrajectoryBuilder::set_follow_traj_mode(bool flag)
{
	hyperparams.follow_trajectory_mode = flag;
	update_follow_target();
}


void TrajectoryBuilder::update_step_flags() {
	_collision_happened = false;
	_stop_happened = false;
	_unsafe_happened = false;

	for (const auto& obst : obst_list) {
		double d = points_dist(ship.pos(), obst.pos()) - ship.rad() - obst.rad();
		// double approximation_delta = hyperparams.safe_dist;
		if (d <= 0) {
			_collision_happened = true;
		}
		else if (d <= (hyperparams.safe_dist)) {
			_unsafe_happened = true;
		}

		if (points_dist(ship.pos(), final_target) < hyperparams.target_reached_rad) {
			_is_finished = true;
			_target_reached = true;
		}

		if (obst.type() == ModelType::static_obst) {
			_min_obst_dist_static = std::min(_min_obst_dist_static, d);
		}
		else if(obst.type() == ModelType::dynamic_obst) {
			_min_obst_dist_dynamic = std::min(_min_obst_dist_dynamic, d);
		}

	}

	if (ship.vel().magnitude() == 0) {
		_stop_happened = true;
	}

	//if (hyperparams.follow_trajectory_mode) {}
}

void TrajectoryBuilder::update_follow_target()
{
	if ((not hyperparams.follow_trajectory_mode) || (follow_targets_list.size() == 0)) {
		return;
	}
	if ( hyperparams.estimate_given_trajectory ) {
		follow_target_idx = std::min((unsigned long long)follow_target_idx + 1, follow_targets_list.size() - 1);
		return;
	}
	if (points_dist(ship.pos(), follow_target) < hyperparams.intermediate_target_reached_rad) {
		follow_target_idx = std::min((unsigned long long)follow_target_idx + 1, follow_targets_list.size() - 1);
		follow_target = follow_targets_list[follow_target_idx];
	}
	else {
		// consider changing by dist flag only when in reached-rad
		// otherwise may be problems with rounding thin coastal
		double min_dist = points_dist(ship.pos(), final_target);
		int target_idx_dist = follow_target_idx;
		for (int i = follow_target_idx; i < follow_targets_list.size() - 1; ++i) {
			double lcl_target_dist = points_dist(ship.pos(), follow_targets_list[i]);
			if (lcl_target_dist < min_dist && lcl_target_dist < hyperparams.intermediate_target_reached_rad) {
				min_dist = lcl_target_dist;
				target_idx_dist = i;
			}
		}

		int target_idx_angle = follow_target_idx;
		// firstly cosider d_traj to current follow_target from previous follow_target
		if (follow_target_idx > 0) {
			//int start_i = std::max((unsigned int)0, follow_target_idx - 1);
			int start_i = follow_target_idx;
			for (int i = start_i; i < follow_targets_list.size(); ++i) {
				Vector dtraj = follow_targets_list[i].sub(follow_targets_list[i - 1]);
				Vector dist_vec = ship.pos().sub(follow_targets_list[i]);
				double lcl_dist_traj_angle = deg_unsigned_angle(dtraj, dist_vec);
				if (lcl_dist_traj_angle > hyperparams.max_angle_to_intermediate_target) {
					target_idx_angle = i;
					break;
				}
			}
		}
		follow_target_idx = std::max(target_idx_angle, target_idx_dist);
		follow_target = follow_targets_list[follow_target_idx];
	}
}


void TrajectoryBuilder::fix_step_events()
{
	if (_collision_happened) {
		_events.push_back(SimulationEvent(_cur_step, collision));
	} 
	if (_unsafe_happened) {
		_events.push_back(SimulationEvent(_cur_step, unsafe));
	}
	if (_stop_happened) {
		_events.push_back(SimulationEvent(_cur_step, stop));
	} 
	//if (_is_finished) 
}


Vector TrajectoryBuilder::choose_velocity() //cosnt
{
	if (hyperparams.estimate_given_trajectory && follow_targets_list.size() > 2) {
		if (follow_target_idx < follow_targets_list.size() - 1) {
			Vector next_pos = follow_targets_list[follow_target_idx];
			Vector cur_pos = ship.pos(); //follow_targets_list[follow_target_idx];
			Vector vel = next_pos.sub(cur_pos);
			return vel;
		}
		else {
			_is_finished = true;
			return Vector(0, 0);
		}
	}

	auto velocities = dynamic_model.all_possible_velocities();
	return optimization_velocity(velocities);
}


Vector TrajectoryBuilder::optimization_velocity(const std::vector<Vector>& velocities) //const
{
	//brute force choose velocity
	
	//worst case is estimation = 1
	double best_estimation = 1.1;
	Vector best_vel(0, 0);


	// updateing once here
	// or updating for every vel (unnecessary) 
	for (auto& obst : obst_list) {
		if (not in_tracking_dist(obst)) {
			continue;
		}

		// if
		if (hyperparams.ignore_VO_static_obsts && obst.type() == ModelType::static_obst) {
			continue;
		}

		obst.update_collision_cone(ship, hyperparams.safe_dist);
	}

	_step_vel_ratings.clear();

	for (const auto& vel : velocities) {
		double vel_est = velocity_estimation(vel);
		_step_vel_ratings.push_back({ vel, vel_est });
		if (vel_est < best_estimation) {
			best_estimation = vel_est;
			best_vel = vel;
		}
	}
	return best_vel;
}


double TrajectoryBuilder::velocity_estimation(Vector vel) //const
{
	double inside_vo_rating = 0;

	
	double collision_risk_rating = 0;
	double target_heading_rating = rating_target_heading(vel);
	for (auto& obst : obst_list) {
		if (not in_tracking_dist(obst)) {
			continue;
		}
		collision_risk_rating = std::max(collision_risk_rating, collision_risk(ship, vel, obst, hyperparams));

		// counting inside vo rating
		if (inside_vo_rating == 0) {
			if (obst.type() == ModelType::static_obst) {
				if (not hyperparams.ignore_VO_static_obsts) {
					inside_vo_rating = obst.velocity_inside_vo(ship.pos(), vel);
				}
			}
			else {
				inside_vo_rating = obst.velocity_inside_vo(ship.pos(), vel);
			}
		}
	}

	Vector cur_vel = ship.vel();
	double diff_speed = abs(vel.magnitude() - cur_vel.magnitude());
	double diff_speed_rating = rating_diff_speed(diff_speed, ship.max_speed());
	double speed_rating = rating_speed(vel.magnitude(), ship.max_speed());

	if (collision_risk_rating < 0.3) {
		collision_risk_rating = 0;
		inside_vo_rating = 0;
	}
	const std::vector<double>& weights = hyperparams.opt_vel_weights;


	return (inside_vo_rating * weights[0] + collision_risk_rating * weights[1] + target_heading_rating * weights[2]
		+ diff_speed_rating * weights[3] + speed_rating * weights[4]); // + diff_heading_rating * weights[5]
}

double TrajectoryBuilder::cur_collision_risk() const
{
	double cr = 0;
	for (auto& obst : obst_list) {
		if (not in_tracking_dist(obst)) {
			continue;
		}
		cr = std::max(cr, collision_risk(ship, ship.vel(), obst, hyperparams));
	}
	return cr;
}

double TrajectoryBuilder::rating_target_heading(Vector vel) const
{
	Vector target = final_target;
	if (hyperparams.follow_trajectory_mode && follow_targets_list.size() > 0)
	{
		target = follow_target;
	}

	Vector vec_to_target = target.sub(ship.pos());
	double target_angle = deg_unsigned_angle(vec_to_target, vel);
	if (target_angle <= 30)
		return target_angle * target_angle / 900.0 / 6.0;
	else
		return (double)1.0 / 6.0 + (target_angle - 30.0) / 180.0;

}

double TrajectoryBuilder::rating_diff_speed(double diff_speed, double ship_max_speed) const
{
	// gives penalty only for deceleration (?)
	if (diff_speed < 0)
		return std::min(diff_speed * diff_speed / (ship_max_speed * ship_max_speed), 1.0);
	else
		return 0;
}

double TrajectoryBuilder::rating_speed(double speed, double ship_max_speed) const
{
	// gives penalty for low speed (?)
	double s = ship_max_speed - speed;
	return std::min(s * s / (ship_max_speed * ship_max_speed), 1.0);
}

TaskObstsInfo TrajectoryBuilder::get_obsts_info(double data_radius)
{
	//bool flag_cnt_limit_coastline_density = false;
	TaskObstsInfo res{0,0,0};
	double dynamic_nom = 0;
	double static_nom = 0;
	double limit_coastline_diametr = ship.rad() * 2;
	double denom = acos(-1) * std::pow(data_radius + limit_coastline_diametr, 2);
	int num_dynamic_obsts = 0, num_static_obsts = 0;
	for (const auto& obst : obst_list) {
		if (obst.type() == ModelType::static_obst) {
			num_static_obsts++;
			static_nom += acos(-1) * std::pow(obst.rad(), 2);
		}
		else if (obst.type() == ModelType::dynamic_obst) {
			num_dynamic_obsts++;
			dynamic_nom += acos(-1) * std::pow(obst.rad(), 2);
		}
	}
	res.dynamic_obsts_density = dynamic_nom / denom;
	res.static_obsts_density = static_nom / denom;
	res.obsts_density = (dynamic_nom + static_nom) / denom;
	res.num_static = num_static_obsts;
	res.num_dynamic = num_dynamic_obsts;
	return res;
}

