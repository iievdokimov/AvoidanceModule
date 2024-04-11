#ifndef LOGGER
#define LOGGER

#include <string>
#include <vector>
#include "config.h"


#include <iostream>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;


enum EventType
{
	unsafe = 0,
	collision,
	stop,
};



class SimulationEvent {
public:
	SimulationEvent(unsigned int step, EventType event) :
		_step{ step }, _event{ event } {}

	unsigned int step() const { return _step; };
	EventType event() const { return _event; };
	std::string str() const;

private:
	unsigned int _step;
	EventType _event;
};

struct RouteQualityData {
	double min_obst_dist_static;
	double min_obst_dist_dynamic;
	double route_length;
	double route_time;
	double max_collision_risk;
};

struct TaskObstsInfo {
	int num_dynamic;
	int num_static;
	double obsts_density;
	double dynamic_obsts_density;
	double static_obsts_density;
};

class FinishLog {
public:
	FinishLog(bool target_reached, unsigned int steps, double build_time, std::vector<SimulationEvent> events, 
		RouteQualityData quality_data, TaskObstsInfo task_obsts_info, const Hyperparams& hyperparams);

	std::string str() const;

	double built_time() const { return _built_time; };

	
	bool target_reached() const { return _target_reached; };
	bool collision_happened() const{ return _collision_happened; };
	bool stop_happened() const{ return _stop_happened; };
	bool unsafe_happened() const { return _unsafe_happened; };
	unsigned int steps() const{ return _steps; };
	const auto& events() const { return _events; };
	const auto& quality_data() const { return _quality_data; };
	const auto& task_obsts_info() const { return _task_obsts_info; };

private:
	void set_result();

	RouteQualityData _quality_data;
	TaskObstsInfo _task_obsts_info;
	const Hyperparams& _hyperparams;
	bool _target_reached;
	bool _collision_happened;
	bool _unsafe_happened;
	bool _stop_happened;
	unsigned int _steps;
	double _built_time;
	std::vector<SimulationEvent> _events;
};



void clear_directory(const fs::path& directory);

int count_files_in_directory(const fs::path& directory);

struct SessionResult {
	int passed_tests;
	int cnt_collided;
	int cnt_unsafe;
	int cnt_unreached;
	int cnt_safe;

	double av_steps;
	double av_time;
	double max_time;

	double av_dynamic_obsts;
	double av_static_obsts;
	double av_obsts_density;
	double av_static_obsts_density;
	double av_dynamic_obsts_density;

	double av_min_dist_static;
	double av_min_dist_dynamic;
	double av_route_length;
	double av_route_time;
	double av_max_collision_risk;

	std::string hyperparams_str;
};

class SessionResultLog {
public:
	SessionResultLog(SessionResult res) : result{ res } {};
	
	std::string str() const;
private:
	SessionResult result;
};


void write_stress_session_result(const SessionResultLog& result, const fs::path& directory);

class Logger {
public:
    // save_error(self, file_name, data) :
        
    // save_test(self, test, step, event = "collision") :
        
    // save_stress_tests_result(self, optimization_mode, cnt_args, av_args, max_args, num_tests) :
   
    // cnt_insts_folder(self, path) :
       
    // write_file(self, log_file_name, data) :
      
    // clean_errors_folder(self) :
    
    // clean_stress_test_folder(self) :
       
    // clean_folder(self, folder) :
      
    // write_stress_test(self, test, test_id, folder) :
      
private:

};




#endif //LOGGER