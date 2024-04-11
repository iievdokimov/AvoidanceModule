#ifndef ALG_BASE
#define ALG_BASE

#include <math.h>
#include <vector>
#include <algorithm>
#include <chrono>

#include <stdexcept>

#include "task.h"
#include "models.h"
#include "config.h"
#include "logger.h"
#include "dynamic_models.h"
#include "risk_indices.h"
#include "test_tools.h"


class TrajectoryBuilder {
public:
	TrajectoryBuilder(Task task, Hyperparams hyperparams, std::vector<Vector> follow_targets_list = {});
	
	std::pair<std::vector<ModelState>, FinishLog> get_full_trajectory();

	void next_step();
	Vector choose_velocity(); //const;
	Vector optimization_velocity(const std::vector<Vector>& possible); //const;

	Vector get_final_target() const { return final_target; };
	const Ship& get_ship() const { return ship; };
	const std::vector<Obstacle>& get_obst_list() const { return obst_list; };
	const Hyperparams& get_hyperparams() const { return hyperparams; };
	const std::vector<std::pair<Vector, double>>& get_vel_ratings() const { return _step_vel_ratings; };
	const std::vector<Vector>& get_follow_targets() const { return follow_targets_list; };
	unsigned int get_follow_target_idx() const { return follow_target_idx; };

	bool in_tracking_dist(const Obstacle& obst) const;

	// for qulity data
	double route_length() const { return _route_length; };
	double route_time() const { return _route_time; };
	double min_dist_static() const { return _min_obst_dist_static; };
	double min_dist_dynamic() const { return _min_obst_dist_dynamic; };
	double max_CR() const { return _max_collision_risk; };

	// for debug-ui usage
	void add_obstacle(Obstacle obst) { obst_list.push_back(obst); };
	void set_follow_targets(std::vector<Vector> trajectory);
	void set_follow_traj_mode(bool flag);
private:
	Vector final_target;
	Ship ship;
	std::vector<Obstacle> obst_list;
	// Task init_task;

	Hyperparams hyperparams;
	CasualDynamicModel dynamic_model;

	Vector follow_target;
	unsigned int follow_target_idx;
	std::vector<Vector> follow_targets_list;

	//double possible_speed_sector = 360;
	
	bool _is_finished;
	unsigned int _cur_step;
	bool _target_reached;

	// quality data
	double _min_obst_dist_static;
	double _min_obst_dist_dynamic;
	double _route_length;
	double _route_time;
	double _max_collision_risk;
	double cur_collision_risk() const;
	
	double _data_radius;

	void _move_all(int steps=1, double step_t=1.0);
	
	//logging
	//FinishLog _finish_log;
	Logger _logger;
	std::vector<SimulationEvent> _events;

	// modes
	bool _chek_local_traj_mode;


	// flags
	bool _collision_happened;
	bool _stop_happened;
	bool _unsafe_happened;
	//_inside_vo_assumption;
	void update_step_flags();
	void update_follow_target();
	void fix_step_events();

	// step optimization data
	double _step_vel_est;
	std::vector < std::pair<Vector, double> > _step_vel_ratings;
	// std::vector <Vector> _step_candidate_velocities;

	// estimation tools
	// non const - updating obstacles collision cones
	double velocity_estimation(Vector vel); // const; 
	double rating_target_heading(Vector vel) const;
	double rating_diff_speed(double speed, double ship_max_speed) const;
	double rating_speed(double speed, double ship_max_speed) const;

	TaskObstsInfo get_obsts_info(double data_radius);

};

#endif //ALG BASE