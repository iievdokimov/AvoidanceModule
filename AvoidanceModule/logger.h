#ifndef LOGGER
#define LOGGER

#include <string>
#include <vector>
#include "config.h"


enum EventType
{
	unsafe = 0,
	collision,
	stop,
};



class SimulationEvent {
public:
	SimulationEvent(int step, EventType event) :
		_step{ step }, _event{ event } {}

	int step() const { return _step; };
	EventType event() const { return _event; };
	std::string str() const;

private:
	int _step;
	EventType _event;
};


class FinishLog {
public:
	FinishLog(bool target_reached, unsigned int steps, double time, std::vector<SimulationEvent> events, const Hyperparams& hyperparams);

	std::string str() const;

	/*
	bool target_reached() const { return _target_reached; };
	bool collision_happened() const{ return _collision_happened; };
	bool stop_happened() const{ return _stop_happened; };
	bool unsafe_happened() const { return _unsafe_happened; };
	unsigned int steps() const{ return _steps; };
	const auto& events() const { return _events; };
	*/
private:
	void set_result();

	const Hyperparams& _hyperparams;
	bool _target_reached;
	bool _collision_happened;
	bool _unsafe_happened;
	bool _stop_happened;
	unsigned int _steps;
	double _built_time;
	std::vector<SimulationEvent> _events;
};



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