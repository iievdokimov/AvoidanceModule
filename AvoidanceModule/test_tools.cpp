#include "test_tools.h"

void run_stress_tests(unsigned int num_tests, bool use_saved_tests) {
	
	int cnt_collided = 0, cnt_unreached = 0, cnt_safe_reached = 0, cnt_unsafe_reached = 0;
	int all_steps = 0;
	double all_time = 0;
	long long all_static_obst = 0;
	long long all_dynamic_obst = 0;
	double all_min_dist_static = 0;
	double all_min_dist_dynamic = 0;
	double all_route_time = 0;
	double all_route_length = 0;
	double all_cr = 0;
	double all_obst_density = 0;
	double all_dynamic_obst_density = 0;
	double all_static_obst_density = 0;
	
	std::vector<int> collided_test_ids;

	double max_time = 0;
	Hyperparams hyperparams;
	double data_radius = 400;

	// for logging -> remove to logging
	std::string session_result_folder = "./logger_data/sessions_results/";
	std::vector<std::string> stress_test_session_folders =
	{
		"./logger_data/stress_test_session/collided/",
		"./logger_data/stress_test_session/unreached/",
		"./logger_data/stress_test_session/unsafe/",
		"./logger_data/stress_test_session/safe/",
	};
	//clear_directory("./logger_data/");
	for (const auto& el : stress_test_session_folders) {
		clear_directory(el);
	}
	for (int i = 1; i < num_tests + 1; ++i) {
		Task* task;
		if (use_saved_tests) {
			std::string filename = "./tasks_cpp_version/task" + std::to_string(i) + ".txt";
			task = new Task(create_task(filename));
		}
		else {
			if (hyperparams.follow_trajectory_mode) {
				Test test(data_radius);
				task = new Task(test.get_random_task_followtraj());
			}
			else {
				Test test(data_radius);
				task = new Task(test.get_random_task());
			}
		}
		std::cout << "Obst list size: " << task->obst_list().size() << std::endl;
		TrajectoryBuilder builder(*task, hyperparams);

		auto build_res = builder.get_full_trajectory();
		auto traj = build_res.first;
		FinishLog log = build_res.second;
		std::cout << "built complete: " + std::to_string(log.built_time()) << "sec.\n log:\n" << log.str() << std::endl;

		bool r = log.target_reached();
		bool c = log.collision_happened();
		bool u = log.unsafe_happened();

		int folder_idx = 0;
		if ((r and u) and (not c)) {
			cnt_unsafe_reached += 1;
			folder_idx = 2;
		}
		if( r and (not u and not c) ){
			cnt_safe_reached += 1;
			folder_idx = 3;
		}
		if (c) {
			cnt_collided += 1;
			collided_test_ids.push_back(i);
			folder_idx = 0;
		}
		if (not r and not c) {
			cnt_unreached += 1;
			folder_idx = 1;
		}

		// for logging -> remove to logging
		std::string task_filename = stress_test_session_folders[folder_idx] + "task" + std::to_string(i) + ".txt";
		write_task(*task, task_filename);

		all_steps += log.steps();
		all_time += log.built_time();
		max_time = std::max(max_time, log.built_time());

		all_min_dist_static += log.quality_data().min_obst_dist_static;
		all_min_dist_dynamic += log.quality_data().min_obst_dist_dynamic;
		all_route_time += log.quality_data().route_time;
		all_route_length += log.quality_data().route_length;
		all_cr += log.quality_data().max_collision_risk;

		all_obst_density += log.task_obsts_info().obsts_density;
		all_dynamic_obst_density += log.task_obsts_info().dynamic_obsts_density;
		all_static_obst_density += log.task_obsts_info().static_obsts_density;

		all_dynamic_obst += log.task_obsts_info().num_dynamic;
		all_static_obst += log.task_obsts_info().num_static;
		
		std::cout << "cur res: passed test [" << std::to_string(i) << "/" << std::to_string(num_tests) << "]" << std::endl;
		std::cout << "cnt_collided=" << std::to_string(cnt_collided) << std::endl;
		std::cout << "cnt_safe_reached=" << std::to_string(cnt_safe_reached) << std::endl;
		std::cout << "cnt_unsafe_reached=" << std::to_string(cnt_unsafe_reached) << std::endl;
		std::cout << "cnt_unreached(but_not_collided)=" << std::to_string(cnt_unreached) << std::endl;
		std::cout << "steps=" << std::to_string(log.steps()) << std::endl;
		
		delete task;
	}

	double av_dynamic_obsts = (double)all_dynamic_obst / num_tests;
	double av_static_obsts = (double)all_static_obst / num_tests;
	double av_obst_density = all_obst_density / num_tests;
	double av_dynamic_obst_density = all_dynamic_obst_density / num_tests;
	double av_static_obst_density = all_static_obst_density / num_tests;
	std::cout << std::endl << "Session finished." << std::endl;
	std::cout << "av_dynamic_obsts=" << std::to_string(av_dynamic_obsts) << std::endl;
	std::cout << "av_static_obsts=" << std::to_string(av_static_obsts) << std::endl;
	std::cout << "av_obsts_density=" << std::to_string(av_obst_density) << std::endl;
	std::cout << "av_dynamic_obsts_density=" << std::to_string(av_dynamic_obst_density) << std::endl;
	std::cout << "av_static_obsts_density=" << std::to_string(av_static_obst_density) << std::endl;
	std::cout << "av_min_dist_static=" << std::to_string((double)all_min_dist_static / num_tests) << std::endl;
	std::cout << "av_min_dist_dynamic=" << std::to_string((double)all_min_dist_dynamic / num_tests) << std::endl << std::endl;
	std::cout << "av_steps=" << std::to_string((double)all_steps / num_tests) << std::endl;
	std::cout << "av_time=" << std::to_string((double)all_time / num_tests) << std::endl;
	std::cout << "max_time=" << std::to_string(max_time) << std::endl;
	
	SessionResult result;
	result.passed_tests = num_tests;
	result.cnt_collided = cnt_collided;
	result.cnt_safe = cnt_safe_reached;
	result.cnt_unreached = cnt_unreached;
	result.cnt_unsafe = cnt_unsafe_reached;
	result.av_dynamic_obsts = av_dynamic_obsts;
	result.av_static_obsts = av_static_obsts;
	result.av_steps = (double)all_steps / num_tests;
	result.av_time = all_time / num_tests;
	result.max_time = max_time;
	result.hyperparams_str = hyperparams.str();
	result.av_min_dist_dynamic = (double)all_min_dist_dynamic / num_tests;
	result.av_min_dist_static = (double)all_min_dist_static / num_tests;
	result.av_route_length = (double)all_route_length / num_tests;
	result.av_route_time = (double)all_route_time / num_tests;
	result.av_max_collision_risk = (double)all_cr / num_tests;
	result.av_obsts_density = av_obst_density;
	result.av_dynamic_obsts_density = av_dynamic_obst_density;
	result.av_static_obsts_density = av_static_obst_density;
	write_stress_session_result(SessionResultLog(result), session_result_folder);
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

	unsigned time_seed = std::chrono::system_clock::now().time_since_epoch().count();
	random_engine.seed(time_seed);

	center = Vector(data_radius, data_radius);

	set_constraints();
	target_ship_config();
	make_boards();
}


void Test::set_constraints() 
{
	coast_obst_rad = ship().rad();

	num_dynamic_obsts = 45;
	num_static_round_obsts = 0;
	num_static_curve_obsts = 0;
	num_static_obsts = num_static_round_obsts + num_static_curve_obsts;

	max_static_obst_size = ship().rad() * 16;
	max_static_curve_obst_size =  ship().rad() * 64;
	max_dynamic_obst_size = ship().rad() * 6;
	max_dynamic_obst_speed = ship().max_speed() * 2;

	min_static_obst_size = ship().rad();
	min_dynamic_obst_size = ship().rad();
	min_dynamic_obst_speed = 0;

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

	Vector delta_pos = center;

	double dist_to_target = data_radius + hyperparams.target_reached_rad / 2.0;
	_target = delta_pos.add(rotate_vector(Vector(dist_to_target, 0), taget_angle));

	Vector unit_v = get_directional_vec(ship().pos(), target());
	Vector ship_vel = Vector(unit_v.x() * ship().max_speed(), unit_v.y() * ship().max_speed());
	_ship.set_vel(ship_vel);
}

std::vector<Vector> get_random_follow_traj(Vector center_pos, Vector target_pos)
{
	double data_radius =  center_pos.x(); // points_dist(center_pos, target_pos);
	std::vector<Vector> follow_traj;
	follow_traj.push_back(center_pos);
	Vector Ox(1, 0);
	Vector center = center_pos;
	double rotation_angle = deg_clockwise_angle(Ox, target_pos.sub(center));
	//int cnt_sections = 3;
	std::vector<double> section_angles = { 70, 45, 0 }; //{80, 65, 45, 20}; //10, 20, 45,
	Vector start_section = center_pos;
	Vector end_section;
	double targets_dist = 20;
	for (int i = 0; i < section_angles.size(); ++i) {
		double y_begin = -data_radius * abs(sin(radians(section_angles[i])));
		double y_end = +data_radius * abs(sin(radians(section_angles[i])));
		std::uniform_real_distribution<double> unif(y_begin, y_end);

		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		std::default_random_engine re(seed);


		double y1 = unif(re);
		double y2 = unif(re);
		double x = data_radius * cos(radians(section_angles[i]));
		Vector delta(x, y1);
		// rotate
		delta = rotate_vector(delta, rotation_angle);
		// relative center shift
		end_section = delta.add(center);

		// conection targets
		Vector vec_to_end = end_section.sub(start_section);
		int num_steps = ceil(vec_to_end.magnitude() / targets_dist);
		for (int i = 1; i <= num_steps; ++i) {
			follow_traj.push_back(start_section.add(vec_to_end.mul((double)i / (double) num_steps)));
		}
		delta = Vector(x, y2);
		//rotate
		start_section = rotate_vector(delta, rotation_angle);
		start_section = start_section.add(center);

		// conection targets
		Vector vec_to_start = start_section.sub(end_section);
		num_steps = ceil(vec_to_start.magnitude() / targets_dist);
		for (int i = 1; i <= num_steps; ++i) {
			follow_traj.push_back(end_section.add(vec_to_start.mul((double)i / (double)num_steps)));
		}

		follow_traj.push_back(start_section);
	}
	follow_traj.push_back(target_pos);
	return follow_traj;
}

void Test::make_boards()
{
	//Vector center_pos = Vector(data_radius, data_radius);

	double l = data_radius * acos(-1) * 2;
	double board_obst_rad = coast_obst_rad;
	double angle_step = 180.0 / (l / board_obst_rad);
	double dist_to_boards = data_radius + board_obst_rad;

	std::vector<Vector> upper_vectors = get_sector_vecs(0.0, 360.0, dist_to_boards, angle_step);
	for (const auto& vec : upper_vectors) {
		Vector obst_vec = vec.add(center);
		double	d_add = ship().rad() + board_obst_rad;
		if (Vector(target().x() - obst_vec.x(), target().y() - obst_vec.y()).magnitude() - d_add > target_margin) {
			_obst_list.push_back(Obstacle(obst_vec, { 0, 0 }, board_obst_rad, ModelType::static_obst, _obst_list.size()));
		}
	}
}


void Test::add_round_static_obst()
{
	std::uniform_real_distribution<double> unif_angle(0.0, 360.0);
	std::uniform_real_distribution<double> unif_obst_pos_rad(ship_margin, data_radius);
	std::uniform_real_distribution<double> unif_obst_size_rad(min_static_obst_size, max_static_obst_size);

	double angle = unif_angle(random_engine);
	double r = unif_obst_pos_rad(random_engine);

	Vector obst_pos = center.add(rotate_vector(Vector(r, 0), angle)); 
	bool near_follow_traj = false;
	double obst_r = unif_obst_size_rad(random_engine);
	double traj_margin = obst_r;
	if (points_dist(_ship.pos(), obst_pos) < traj_margin or points_dist(_target, obst_pos) < traj_margin) {
		add_round_static_obst();
		return;
	}

	for (const auto& pos : _follow_targets) {
		if (points_dist(pos, obst_pos) < traj_margin) {
			add_round_static_obst();
			return;
		}
	}

	int num_edges = 9;
	std::vector<Vector> coastline;
	double angle_step = 360.0 / (num_edges);
	auto vecs = get_sector_vecs(0.0, 360.0, obst_r, angle_step);
	for (const auto& vec : vecs) {
		coastline.push_back(obst_pos.add(vec));
	}

	for (const Obstacle& obst : preprocess_coastline(coastline, _ship.rad(), _obst_list.size())) {
		_obst_list.push_back(obst);
	}
}


void Test::add_curve_static_obst()
{
	std::uniform_real_distribution<double> unif_angle(0.0, 360.0);
	std::uniform_real_distribution<double> unif_obst_pos_rad(ship_margin, data_radius);
	std::uniform_real_distribution<double> unif_obst_size(min_static_obst_size, max_static_curve_obst_size);

	double angle = unif_angle(random_engine);
	double r = unif_obst_pos_rad(random_engine);

	Vector obst_pos = center.add(rotate_vector(Vector(r, 0), angle));
	double obst_w = unif_obst_size(random_engine);
	double obst_h = obst_w / 2;
	double traj_margin = obst_w;

	if (points_dist(_ship.pos(), obst_pos) < traj_margin or points_dist(_target, obst_pos) < traj_margin) {
		add_round_static_obst();
		return;
	}

	for (const auto& pos : _follow_targets) {
		if (points_dist(pos, obst_pos) < traj_margin) {
			add_round_static_obst();
			return;
		}
	}

	int num_sections = 4;
	std::vector<Vector> upper_coastline;
	std::vector<Vector> lower_coastline;
	std::uniform_real_distribution<double> unif_obst_section(0, obst_h);
	double rotation = unif_angle(random_engine);
	for (int i = 0; i < num_sections; ++i) {
		double y1 = unif_obst_section(random_engine);
		double y2 = unif_obst_section(random_engine);
		double x = -obst_w / 2 + obst_w * (double)i / num_sections;
		Vector vec1(x, std::min(y1, y2));
		Vector vec2(x, std::max(y1, y2));
		// rotate
		vec1 = rotate_vector(vec1, rotation);
		vec2 = rotate_vector(vec2, rotation);

		upper_coastline.push_back(obst_pos.add(vec2));
		lower_coastline.push_back(obst_pos.add(vec1));
	}

	for (int i = lower_coastline.size() - 1; i >= 0; --i) {
		upper_coastline.push_back(lower_coastline[i]);
	}
	upper_coastline.push_back(upper_coastline[0]);

	for (const Obstacle& obst : preprocess_coastline(upper_coastline, _ship.rad(), _obst_list.size())) {
		_obst_list.push_back(obst);
	}
}


void Test::add_dynamic_obst() {
	std::uniform_real_distribution<double> unif_angle(0.0, 360.0);
	std::uniform_real_distribution<double> unif_obst_pos_rad(ship_margin, data_radius);
	std::uniform_real_distribution<double> unif_obst_size_rad(min_dynamic_obst_size, max_dynamic_obst_size);
	std::uniform_real_distribution<double> unif_obst_speed(min_dynamic_obst_speed, max_dynamic_obst_speed);

	double angle = unif_angle(random_engine);
	double r = unif_obst_pos_rad(random_engine);

	Vector obst_pos = center.add(rotate_vector(Vector(r, 0), angle));
	double obst_r = unif_obst_size_rad(random_engine);
	double vel_rotation = unif_angle(random_engine);
	double speed = unif_obst_speed(random_engine);
	Vector obst_vel = rotate_vector(Vector(speed, 0), vel_rotation);
	int obst_id = _obst_list.size();
	_obst_list.push_back(Obstacle(obst_pos, obst_vel, obst_r, ModelType::dynamic_obst, obst_id));
}


void Test::make_random_task() {
	make_obstacles();
}

void Test::make_random_task_followtraj() {
	_follow_targets = get_random_follow_traj(_ship.pos(), _target);
	make_obstacles();
}

void Test::make_obstacles()
{
	for (int i = 0; i < num_static_round_obsts; ++i) {
		add_round_static_obst();
	}
	for (int i = 0; i < num_static_curve_obsts; ++i) {
		add_curve_static_obst();
	}
	for (int i = 0; i < num_dynamic_obsts; ++i) {
		add_dynamic_obst();
	}
}

Task Test::get_random_task() {
	make_random_task();
	return Task(_ship, _target, _obst_list);
}

Task Test::get_random_task_followtraj() {
	make_random_task_followtraj();
	return Task(_ship, _target, _obst_list, _follow_targets);
}
