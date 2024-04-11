#include "logger.h"

FinishLog::FinishLog(bool target_reached, unsigned int steps, double build_time, std::vector<SimulationEvent> events, 
    RouteQualityData quality_data, TaskObstsInfo task_obsts_info, const Hyperparams& hyperparams)
    :_target_reached{ target_reached }, _steps{ steps }, _built_time{ build_time },
    _events{ events }, _hyperparams{ hyperparams }, _quality_data{ quality_data }, _task_obsts_info{ task_obsts_info }
{
    set_result();
}

void clear_directory(const fs::path& directory)
{
    try {
        for (const auto& entry : fs::directory_iterator(directory)) {
            if (fs::is_directory(entry)) {
                // recursively
                clear_directory(entry.path());
            }
            else {
                fs::remove(entry);
            }
        }
    }
    catch (...)
    {
        std::cout << "Error cleaning directory" << std::endl;
    }
}

int count_files_in_directory(const fs::path& directory)
{
    int count = 0;
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (fs::is_regular_file(entry)) {
            count++;
        }
    }
    return count;
}

void write_stress_session_result(const SessionResultLog& result, const fs::path& directory)
{
    int session_idx = count_files_in_directory(directory);
    std::string session_result_filename = "session_result_" + std::to_string(session_idx) + ".txt";
    fs::path filepath = directory / session_result_filename;
    
    std::ofstream file(filepath);
    if (file.is_open()) {
        file << result.str();
    }
    else {
        std::cout << "Error writing session result" << std::endl;
    }

}

std::string SimulationEvent::str() const
{
    std::string type = "unknown";
    if (_event == collision) {
        type = "collision";
    }
    else if (_event == unsafe) {
        type = "unsafe";
    }
    else if (_event == stop) {
        type = "stop";
    }
    return "Event( step= " + std::to_string(_step) + ",  " + type + " )";
}


std::string FinishLog::str() const {
    std::string res = "FinishLog\n";

    res = res + "target_reached = " + std::to_string(_target_reached) + "\n\n";
    res = res + "steps = " + std::to_string(_steps) + "\n";
    res = res + "built_time = " + std::to_string(_built_time) + "\n\n";
    res = res + "collision_happened = " + std::to_string(_collision_happened) + "\n";
    res = res + "unsafe_happened = " + std::to_string(_unsafe_happened) + "\n";
    res = res + "stop_happened = " + std::to_string(_stop_happened) + "\n\n";

    res = res + "Quality data:\n";
    res = res + "min_dist_to_dynamic_obst = " + std::to_string(quality_data().min_obst_dist_dynamic) + "\n";
    res = res + "min_dist_to_staic_obst = " + std::to_string(quality_data().min_obst_dist_static) + "\n";
    res = res + "max_collision_risk = " + std::to_string(quality_data().max_collision_risk) + "\n";
    res = res + "rote_length = " + std::to_string(quality_data().route_length) + "\n";
    res = res + "route_time = " + std::to_string(quality_data().route_time) + "\n\n";
    
    res = res + "Task obsts info:\n";
    res = res + "num_dynamic_obsts = " + std::to_string(task_obsts_info().num_dynamic) + "\n";
    res = res + "num_static_obsts = " + std::to_string(task_obsts_info().num_static) + "\n";
    res = res + "obst_density = " + std::to_string(task_obsts_info().obsts_density) + "\n\n";

    res = res + _hyperparams.str() + "\n";
    
    res = res + "Events(\n";
    for (auto event : _events) {
        res = res + event.str() + "\n";
    }
    res = res + ")\n";
    return res;


}

void FinishLog::set_result()
{
    for (const auto& event : _events) {
        if (event.event() == collision) {
            _collision_happened = true;
        }
        else if (event.event() == unsafe) {
            _unsafe_happened = true;
        }
        else if (event.event() == stop) {
            _stop_happened = true;
        }
    }
}

std::string SessionResultLog::str() const
{
    std::string res = "Passed tests [" + std::to_string(result.passed_tests) + "]\n\n"
        + "cnt_collided= " + std::to_string(result.cnt_collided) + "\n"
        + "cnt_safe= " + std::to_string(result.cnt_safe) + "\n"
        + "cnt_unsafe= " + std::to_string(result.cnt_unsafe) + "\n"
        + "cnt_unreached= " + std::to_string(result.cnt_unreached) + "\n\n"
        + "av_steps= " + std::to_string(result.av_steps) + "\n"
        + "av_time= " + std::to_string(result.av_time) + "\n"
        + "max_time= " + std::to_string(result.max_time) + "\n\n"
        + "av_dynamic_obsts= " + std::to_string(result.av_dynamic_obsts) + "\n"
        + "av_static_obsts= " + std::to_string(result.av_static_obsts) + "\n\n"
        + "av_obsts_density= " + std::to_string(result.av_obsts_density) + "\n\n"
        + "av_min_dist_dynamic= " + std::to_string(result.av_min_dist_dynamic) + "\n"
        + "av_min_dist_static= " + std::to_string(result.av_min_dist_static) + "\n\n"
        + "av_route_length= " + std::to_string(result.av_route_length) + "\n"
        + "av_route_time= " + std::to_string(result.av_route_time) + "\n"
        + "av_max_collision_risk= " + std::to_string(result.av_max_collision_risk) + "\n\n\n"
        + "USED HYPERPARAMS=\n" + result.hyperparams_str + "\n";
 
    return res;
}
