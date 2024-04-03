#include "test_tools.h"

void run_stress_tests(unsigned int num_tests) {
	
	int cnt_collided = 0, cnt_unreached = 0, cnt_safe_reached = 0, cnt_unsafe_reached = 0;
	int all_steps = 0;
	double all_time = 0;
	int all_static_obst = 0;
	int all_dynamic_obst = 0;
	double all_scale = 0;
	
	std::vector<int> collided_test_ids;

	double max_time = 0;
	for (int i = 1; i < num_tests + 1; ++i) {
		std::string filename = "./tasks_cpp_version/task" + std::to_string(i) + ".txt";
		Task task = create_task(filename);
		std::cout << "Obst list size: " << task.obst_list().size() << std::endl;
		Hyperparams hyperparams{ task.scale() };
		std::vector<Vector> follow_targets_list = fake_follow_targets(task.ship().pos(), task.target(), 12);
		TrajectoryBuilder builder(task, hyperparams, follow_targets_list);

		if (i == 88) {
			std::cout << std::endl;
		}

		auto build_res = builder.get_full_trajectory();
		auto traj = build_res.first;
		FinishLog log = build_res.second;
		std::cout << "built complete: " + std::to_string(log.built_time()) << "sec.\n log:\n" << log.str() << std::endl;

		bool r = log.target_reached();
		bool c = log.collision_happened();
		bool u = log.unsafe_happened();

		if ((r and u) and (not c)) {
			cnt_unsafe_reached += 1;
		}
		if( r and (not u and not c) ){
			cnt_safe_reached += 1;
		}
		if (c) {
			cnt_collided += 1;
			collided_test_ids.push_back(i);
		}
		if (not r and not c) {
			cnt_unreached += 1;
		}

	
		all_steps += log.steps();
		all_time += log.built_time();
		max_time = std::max(max_time, log.built_time());

		
		std::cout << "cur res: passed test [" << std::to_string(i) << "/" << std::to_string(num_tests) << "]" << std::endl;
		std::cout << "cnt_collided=" << std::to_string(cnt_collided) << std::endl;
		std::cout << "cnt_safe_reached=" << std::to_string(cnt_safe_reached) << std::endl;
		std::cout << "cnt_unsafe_reached=" << std::to_string(cnt_unsafe_reached) << std::endl;
		std::cout << "cnt_unreached(but_not_collided)=" << std::to_string(cnt_unreached) << std::endl;
		std::cout << "steps=" << std::to_string(log.steps()) << std::endl;
		
		std::cout << "Collided tests: ";
		for (auto el : collided_test_ids)
			std::cout << el << " ";
		std::cout << std::endl;

		//std::string flag;
		//std::cin >> flag;

	}

	std::cout << "av_steps=" << std::to_string((double)all_steps/num_tests) << std::endl;
	std::cout << "av_time=" << std::to_string((double)all_time/num_tests) << std::endl;
	std::cout << "max_time=" << std::to_string(max_time) << std::endl;
}

