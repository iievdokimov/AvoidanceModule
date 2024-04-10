#include <iostream>
#include "algorithm_usage.h"
#include "task.h"
#include "test_tools.h"


__declspec(dllexport) int main(){
    run_stress_tests(500, true);
    //build_traj();
    //write_traj();
    /*std::string task_filename = "./task.txt";
    Task task = create_task(task_filename);
    for (auto obst : task.obst_list()) {
        std::cout << obst.str() << std::endl;
    }*/
}
