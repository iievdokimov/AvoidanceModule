#ifndef ALG_TASK
#define ALG_TASK

#include <string>


class Task {
public:
	Task();
private:
	double ship;
	double cur_trajectory;
	double obst_list;
	double hyperparams;
};

// void create_task(const std::string& task_filename);

// void execute_task(const Task& task);



#endif //ALG_TASK
