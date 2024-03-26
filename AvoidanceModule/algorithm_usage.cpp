#include "algorithm_usage.h"


void build_traj(){
	std::string task_filename = "./cpptask.txt";
	Task task = create_task(task_filename);
	Hyperparams hyperparams{ task.scale()};
	TrajectoryBuilder algo(task, hyperparams);
	auto build_res = algo.get_full_trajectory();
	
	auto traj = build_res.first;

	FinishLog log = build_res.second;

	// # sending AM log
	// send_estimation_log(log)
	// send_traj()

	std::cout << "built complete: " + std::to_string(log.built_time()) << "sec.\n log:\n" << log.str() << std::endl;

	write_traj(traj);
}	


void write_traj(const std::vector<ModelState>& traj) {
	std::string filename = "./cpptraj.txt";
	std::ofstream out;      
	out.open(filename);
	if (out.is_open())
	{
		for (auto& state : traj) {
			out << state._pos.x() << " " << state._pos.y() << std::endl;
		}
	}

}