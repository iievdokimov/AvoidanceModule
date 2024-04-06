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
		Hyperparams hyperparams;
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

Task generate_empty_task(double data_radius)
{
	Test empty_test(data_radius);
	return empty_test.get_empty_task();
}

Test::Test(double data_radius)
	: data_radius{ data_radius },
	_ship{ Ship(Vector(data_radius, data_radius), Vector(0, 0), 
		hyperparams.ship_radius, ModelType::ship_obst, -1, hyperparams.max_speed, hyperparams.ship_radar_radius) }
{
	set_constraints();
	target_ship_config();
	make_boards();
}


void Test::set_constraints() 
{
	num_dynamic_obsts = 95;
	num_static_obsts = 45;
	max_static_obst_size = ship().rad() * 4;
	max_dynamic_obst_size = ship().rad() * 6;
	max_dynamic_obst_speed = ship().max_speed() * 3;
	ship_margin = ship().radar_rad() / 1.5;
	target_margin = hyperparams.target_reached_rad;
}

Task Test::get_empty_task()
{
	return Task(_ship, _target, _obst_list);
}

void Test::target_ship_config()
{
	std::uniform_real_distribution<double> unif(0, 360.0);
	std::default_random_engine re;

	double taget_angle = unif(re);

	Vector delta_pos = Vector(data_radius, data_radius);

	double dist_to_target = data_radius + hyperparams.target_reached_rad / 2.0;
	_target = delta_pos.add(rotate_vector(Vector(dist_to_target, 0), taget_angle));

	Vector unit_v = get_directional_vec(ship().pos(), target());
	Vector ship_vel = Vector(unit_v.x() * ship().max_speed(), unit_v.y() * ship().max_speed());
	_ship.set_vel(ship_vel);
}

void Test::make_boards()
{
	Vector center_pos = Vector(data_radius, data_radius);

	double l = data_radius * acos(-1) * 2;
	double board_obst_rad = ship().rad();
	//double board_obst_rad = ship().rad() * 4;
	double angle_step = 180.0 / (l / board_obst_rad);
	double dist_to_boards = data_radius + board_obst_rad;

	std::vector<Vector> upper_vectors = get_sector_vecs(0.0, 360.0, dist_to_boards, angle_step);
	for (const auto& vec : upper_vectors) {
		Vector obst_vec = vec.add(center_pos);
		double	d_add = ship().rad() + board_obst_rad;
		if (Vector(target().x() - obst_vec.x(), target().y() - obst_vec.y()).magnitude() - d_add > target_margin) {
			_obst_list.push_back(Obstacle(obst_vec, { 0, 0 }, board_obst_rad, ModelType::static_obst, _obst_list.size()));
		}
	}
}
