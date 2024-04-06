#ifndef ALG_TEST
#define ALG_TEST

#include <string>
#include <random>

#include "task.h"
#include "algorithm_base.h"
#include "algorithm_usage.h"
#include "config.h"
#include "math_tools.h"


void run_stress_tests(unsigned int num_tests);


Task generate_empty_task(double data_radius);


class Test {
public:
	Test(double data_radius);

	Task get_empty_task();
	//Task get_random_task();


	const Ship& ship() const { return _ship; }
	const std::vector<Obstacle>& obst_list() const { return _obst_list; }
	const Vector& target() const { return _target; }
	
	//double get_obst_density() const;
private:
	Hyperparams hyperparams;
	double data_radius;
	Vector margin;
	Ship _ship;
	Vector _target;
	std::vector<Obstacle> _obst_list;

	unsigned int num_dynamic_obsts;
	unsigned int num_static_obsts;
	double max_static_obst_size;
	double max_dynamic_obst_size;
	double max_dynamic_obst_speed;
	double ship_margin;
	double target_margin;

	void set_constraints();
	
	void target_ship_config();

	//void add_static_obst();

	//void add_dynamic_obst();

	//void make_random_task();

	void make_boards();
};


#endif // ALG_TEST