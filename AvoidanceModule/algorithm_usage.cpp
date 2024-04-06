#include "algorithm_usage.h"


void build_traj(){
	std::string task_filename = "./cpptask.txt";
	Task task = create_task(task_filename);
	Hyperparams hyperparams;

	std::vector<Vector> follow_targets_list = fake_follow_targets(task.ship().pos(), task.target(), 12);

	TrajectoryBuilder algo(task, hyperparams, follow_targets_list);
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

std::vector<ModelState> fake_trajectory(Vector ship_pos, Vector final_target, int steps)
{
	std::vector<ModelState> res;
	Vector to_target = final_target.sub(ship_pos);
	int points_num = steps;
	//double step_x = to_target.x() / points_num;
	//double step_y = to_target.y() / points_num;
	for (int i = 1; i <= points_num; ++i) {
		double k = (double)i / points_num;
		Vector cur = ship_pos.add(to_target.mul(k));
		res.push_back({ cur, {0, 0} });
	}
	return res;
}

std::vector<Vector> fake_follow_targets(Vector ship_pos, Vector final_target, int steps)
{
	std::vector<Vector> follow_targets_list = {};
	for (const auto& el : fake_trajectory(ship_pos, final_target, steps)) {
		follow_targets_list.push_back(el._pos);
	}
	return follow_targets_list;
}