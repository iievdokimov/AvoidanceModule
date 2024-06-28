#include <iostream>
#include "algorithm_usage.h"
#include "task.h"
#include "test_tools.h"
#include "trajectory_estimation.h"



__declspec(dllexport) int main(){
    //double data_rad = 400;
    //Test test(data_rad);
    //Task task = test.get_random_task_followtraj();

    // Результат сравнения:

    //TrajectoryEstimator est = comparison_build(task);

    //std::cout << est.new_traj_log().str() << std::endl;
    //std::cout << "Log of current: " << std::endl;
    //std::cout << est.cur_traj_log().str() << std::endl;

    
    bool use_saved_tests = false;
    run_stress_tests(10000, use_saved_tests);
    //build_traj();
    //write_traj();
    /*std::string task_filename = "./task.txt";
    Task task = create_task(task_filename);
    for (auto obst : task.obst_list()) {
        std::cout << obst.str() << std::endl;
    }*/
}
