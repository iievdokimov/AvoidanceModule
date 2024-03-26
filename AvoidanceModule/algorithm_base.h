#ifndef ALG_BASE
#define ALG_BASE

#include <math.h>
#include <vector>
#include <algorithm>

#include <stdexcept>

#include "task.h"
#include "models.h"
#include "config.h"
#include "logger.h"
#include "dynamic_models.h"
#include "risk_indices.h"


class TrajectoryBuilder {
public:
	TrajectoryBuilder(Task task, Hyperparams hyperparams);
	
	std::vector<ModelState> get_full_trajectory();
	std::vector<ModelState> fake_trajectory();

	void next_step();
	Vector choose_velocity(); //const;
	Vector optimization_velocity(const std::vector<Vector>& possible); //const;

private:
	Vector final_target;
	Ship ship;
	std::vector<Obstacle> obst_list;

	Hyperparams hyperparams;
	CasualDynamicModel dynamic_model;

	//double possible_speed_sector = 360;
	
	bool _is_finished;
	unsigned int _cur_step;

	void _move_all(int steps=1, double step_t=1.0);
	
	//logging
	//FinishLog _finish_log;
	Logger _logger;

	// modes
	bool _chek_local_traj_mode;


	// flags
	bool _collision_happened;
	bool _stop_happened;
	bool _unsafe_happened;
	//_inside_vo_assumption;
	void update_step_flags();


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

};



#endif //ALG BASE