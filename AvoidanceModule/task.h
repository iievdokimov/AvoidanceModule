#ifndef ALG_TASK
#define ALG_TASK

#include <string>
#include "models.h"
#include <fstream>


class Task {
public:
	Task(Ship ship, Vector target, std::vector<ModelObject> obst_list)
		: _ship{ ship }, _target{ target }, _obst_list{ obst_list } {};

	Ship ship() const { return _ship; };
	std::vector<Vector> cur_trajectory() const { return _cur_trajectory; };
	std::vector<ModelObject> obst_list() const { return _obst_list; };
	Vector target() const { return _target; };

private:
	Ship _ship;
	std::vector<Vector> _cur_trajectory;
	std::vector<ModelObject> _obst_list;
	Vector _target;
	//Hyperparams hyperparams;
};



Task create_task(const std::string& task_filename);


// void execute_task(const Task& task);



#endif //ALG_TASK
