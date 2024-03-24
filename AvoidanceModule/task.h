#ifndef ALG_TASK
#define ALG_TASK

#include <string>
#include "models.h"
#include <fstream>


class Task {
public:
	Task(Ship ship, Vector target, std::vector<Obstacle> obst_list, double scale)
		: _ship{ ship }, _target{ target }, _obst_list{ obst_list }, _scale{ scale } {};

	Ship ship() const { return _ship; };
	std::vector<Vector> cur_trajectory() const { return _cur_trajectory; };
	std::vector<Obstacle> obst_list() const { return _obst_list; };
	Vector target() const { return _target; };
	double scale() const { return _scale; };
private:
	Ship _ship;
	std::vector<Vector> _cur_trajectory;
	std::vector<Obstacle> _obst_list;
	Vector _target;
	double _scale;
	//Hyperparams hyperparams;
};



Task create_task(const std::string& task_filename);


// void execute_task(const Task& task);



#endif //ALG_TASK
