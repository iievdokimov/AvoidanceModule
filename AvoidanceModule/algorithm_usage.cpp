#include "algorithm_usage.h"


void build_traj(){
	std::string task_filename = "./task.txt";
	Task task = create_task(task_filename);
	Hyperparams hyperparams{ task.scale()};
	TrajectoryBuilder algo(task, hyperparams);
	std::vector<ModelState> traj = algo.get_full_trajectory();
	
	// # sending AM log
	// send_estimation_log(log)
	// send_traj()

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