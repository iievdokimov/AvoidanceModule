#ifndef ALG_TASK
#define ALG_TASK

#include <string>
#include "models.h"
#include <fstream>

class Task {
public:
	//Task(Ship ship, Vector target, std::vector<Obstacle> obst_list, std::vector<ModelState> cur_trajectory = {});
	Task(Ship ship, Vector target, std::vector<Obstacle> obst_list, std::vector<Vector> follow_targets = {});

	Ship ship() const { return _ship; };
	std::vector<ModelState> cur_trajectory() const { return _cur_trajectory; };
	std::vector<Vector> follow_targets() const { return _follow_targets; };
	std::vector<Obstacle> obst_list() const { return _obst_list; };
	Vector target() const { return _target; };
	//Hyperparams hyperparams() const { return _hyperparams; };

private:
	Ship _ship;
	std::vector<ModelState> _cur_trajectory;
	std::vector<Vector> _follow_targets;
	std::vector<Obstacle> _obst_list;
	Vector _target;

	//Hyperparams _hyperparams;

	void configure_follow_targets();
};



Task create_task(const std::string& task_filename);

void write_task(const Task& task, const std::string& task_filename);

// void execute_task(const Task& task);



#endif //ALG_TASK
