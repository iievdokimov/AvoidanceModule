#include "algorithm_base.h"
#include <iostream>


TrajectoryBuilder::TrajectoryBuilder(Task task, Hyperparams hyperparams) :
	ship{ task.ship() }, final_target{ task.target() }, obst_list{ task.obst_list() },
	hyperparams{ hyperparams }, dynamic_model{ ship, hyperparams.max_turn_angle * 2 }
{
	_is_finished = false;
	_cur_step = 0;
	_chek_local_traj_mode = false;


	_collision_happened = false;
	_stop_happened = false;
	_unsafe_happened = false;
}


std::vector<ModelState> TrajectoryBuilder::get_full_trajectory()
{
	auto start = std::chrono::steady_clock::now();
	while (!_is_finished && _cur_step < hyperparams.max_steps) {
		next_step();
	}
	if (!_is_finished) {
		//fix_finish(false);
		std::cout << "unfinished" << std::endl;
	}
	auto end = std::chrono::steady_clock::now();
	auto diff = end - start;
	double seconds = std::chrono::duration<double>(diff).count();
	std::cout << "Built time: " << seconds << " sec." << std::endl;
	return ship.traj();
}


std::vector<ModelState> TrajectoryBuilder::fake_trajectory()
{
	std::vector<ModelState> res;
	Vector to_target = final_target.sub(ship.pos());
	int points_num = 12;
	//double step_x = to_target.x() / points_num;
	//double step_y = to_target.y() / points_num;
	for (int i = 0; i <= points_num; ++i) {
		double k = (double)i / points_num;
		Vector cur = ship.pos().add(to_target.mul(k));
		res.push_back({ cur, {0, 0} });
	}
	return res;
}


void TrajectoryBuilder::next_step(){
	if (_is_finished) return;

	//std::cout << "step: " << _cur_step << "max_steps: " << hyperparams.max_steps << std::endl;
	
	
	Vector best_velocity = choose_velocity();
	ship.set_vel(best_velocity);
	_move_all();

	//std::cout << ship.str() << std::endl;
	
	update_step_flags();
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
	if (points_dist(ship.pos(), obst.pos()) - obst.rad() < ship.radar_rad()) {
		return true;
	}
	return false;
}


void TrajectoryBuilder::update_step_flags() {
	_collision_happened = false;
	_stop_happened = false;
	_unsafe_happened = false;

	for (const auto& obst : obst_list) {
		double d = points_dist(ship.pos(), obst.pos()) - ship.rad() - obst.rad();
		double approximation_delta = hyperparams.safe_dist;
		if (d <= (0 - approximation_delta)) {
			_collision_happened = true;
		}
		else if (d <= (hyperparams.safe_dist - approximation_delta)) {
			_unsafe_happened = true;
		}

		if (points_dist(ship.pos(), final_target) < hyperparams.target_reached_rad) {
			_is_finished = true;
		}
	}

	if (ship.vel().magnitude() == 0) {
		_stop_happened = true;
	}

	//if (hyperparams.follow_trajectory_mode) {}
}


Vector TrajectoryBuilder::choose_velocity() //cosnt
{
	//if (_chek_local_traj_mode)
	//	return Vector();

	
	auto velocities = dynamic_model.all_possible_velocities();
	//for (auto& el : velocities) {
	//	std::cout << el.str() << " ";
	//}
	//std::cout << std::endl;
	//std::string flag;
	//std::cin >> flag;
	return optimization_velocity(velocities);
}


Vector TrajectoryBuilder::optimization_velocity(const std::vector<Vector>& velocities) //const
{
	//std::cout << "optimization" << std::endl;
	//brute force choose velocity
	
	//worst case is estimation = 1
	double best_estimation = 1.1;
	Vector best_vel(0, 0);

	//_step_vel_ratings.clear();
	// std::cout << "Vels num: " << velocities.size() << std::endl;
	for (const auto& vel : velocities) {
		double vel_est = velocity_estimation(vel);
		// vel_ratings[vel] = vel_est
		// print("vel, est, inside_vo", vel, vel_est, inside_vo, "\n")
		if (vel_est < best_estimation) {
			best_estimation = vel_est;
			best_vel = vel;
		}
		//std::cout << "vel est: " << vel.str() << " <> " << vel_est << std::endl;
		
	}

	//std::cout << "res: " << best_vel.str() << " " << best_estimation << std::endl;

	//std::string flag;
	//std::cin >> flag;

	return best_vel;
}


double TrajectoryBuilder::velocity_estimation(Vector vel) //const
{
	double inside_vo_rating = 0;

	
	double collision_risk_rating = 0;
	double target_heading_rating = rating_target_heading(vel);
	for (auto& obst : obst_list) {
		//std::cout << "CR for obst" << collision_risk(ship, vel, obst, hyperparams) << std::endl;
		if (not in_tracking_dist(obst)) {
			continue;
		}

		collision_risk_rating = std::max(collision_risk_rating, collision_risk(ship, vel, obst, hyperparams));

		//std::cout << "Before CC constructing for obst id=" << obst.id() << std::endl;

		//counting inside vo rating
		if (inside_vo_rating == 0) {
			// update_CC may be relocated inside obst_move (use const Ship& in Obst constructor then)
			obst.update_collision_cone(ship, hyperparams.safe_dist); //ruins velocity_estimation const cvalifier
			//std::cout << "CC updated" << std::endl;
			inside_vo_rating = obst.velocity_inside_vo(ship.pos(), vel);
		}
		//std::string flag;
		//std::cin >> flag;

		//std::cout << "VO: ";
		//for (auto el : obst.collision_cone())
		//	std::cout << el.str() << " ";
		//std::cout << std::endl;
		// std::cout << "Inside VO: " << inside_vo_rating  << " " << obst.str() << " " << vel.str() << std::endl;
	}

	//std::cout << "Inside VO: " << inside_vo_rating  << " " << vel.str() << std::endl;

	Vector cur_vel = ship.vel();
	double diff_speed = abs(vel.magnitude() - cur_vel.magnitude());
	double diff_speed_rating = rating_diff_speed(diff_speed, ship.max_speed());
	double speed_rating = rating_speed(vel.magnitude(), ship.max_speed());

	// diff_heading = math_tools.deg_unsigned_angle(velocity, cur_vel)
	// diff_heading_rating = 0
	// print(f"vel={velocity} result collision risk: ", collision_risk_rating)

	if (collision_risk_rating < 0.3) {
		collision_risk_rating = 0;
		inside_vo_rating = 0;
	}
	const std::vector<double>& weights = hyperparams.opt_vel_weights;

	//std::cout << "CR: " << collision_risk_rating << " " << vel.str() << std::endl;

	//double flag;
	//std::cin >> flag;

	return (inside_vo_rating * weights[0] + collision_risk_rating * weights[1] + target_heading_rating * weights[2]
		+ diff_speed_rating * weights[3] + speed_rating * weights[4]); // + diff_heading_rating * weights[5]
}

double TrajectoryBuilder::rating_target_heading(Vector vel) const
{
	Vector vec_to_target = final_target.sub(ship.pos());
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
