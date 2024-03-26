#include "logger.h"

FinishLog::FinishLog(bool target_reached, unsigned int steps, double time, std::vector<SimulationEvent> events, const Hyperparams& hyperparams)
    :_target_reached{ target_reached }, _steps{ steps }, _built_time{ time },
    _events{ events }, _hyperparams{ hyperparams }
{
    set_result();
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
