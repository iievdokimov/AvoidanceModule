#ifndef ALG_TEST
#define ALG_TEST

#include <string>
#include <random>
#include <chrono>

#include "task.h"
#include "algorithm_base.h"
#include "algorithm_usage.h"
#include "config.h"
#include "math_tools.h"
#include "preprocess_task.h"
#include "logger.h"


void run_stress_tests(unsigned int num_tests, bool );


Task generate_empty_task(double data_radius);

std::vector<Vector> get_random_follow_traj(Vector center_pos, Vector target_pos);

class Test {
public:
	Test(double data_radius);

	Task get_empty_task();
	Task get_random_task();
	Task get_random_task_followtraj();

	const Ship& ship() const { return _ship; }
	const std::vector<Obstacle>& obst_list() const { return _obst_list; }
	const std::vector<Vector>& follow_targets() const { return _follow_traj; }
	const Vector& target() const { return _target; }
	
	//double get_obst_density() const;
private:
	Hyperparams hyperparams;
	double data_radius;
	Vector margin;
	Ship _ship;
	Vector _target;
	std::vector<Obstacle> _obst_list;
	std::vector<Vector> _follow_traj;
	double coast_obst_rad;

	Vector center;

	unsigned int num_dynamic_obsts;
	unsigned int num_static_obsts;
	unsigned int num_static_round_obsts;
	unsigned int num_static_curve_obsts;
	double max_static_obst_size;
	double max_static_curve_obst_size;
	double max_dynamic_obst_size;
	double max_dynamic_obst_speed;
	double min_static_obst_size;
	double min_dynamic_obst_size;
	double min_dynamic_obst_speed;
	double ship_margin;
	double target_margin;

	std::default_random_engine random_engine;

	void set_constraints();
	
	void target_ship_config();

	//void add_static_obst();

	void add_dynamic_obst();

	void make_random_task();

	void make_random_task_followtraj();

	void add_round_static_obst();

	void add_curve_static_obst();

	void make_boards();

	void make_obstacles();
};


#endif // ALG_TEST