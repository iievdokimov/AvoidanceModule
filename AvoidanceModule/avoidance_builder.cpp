#include <iostream>
#include "algorithm_usage.h"
#include "task.h"



__declspec(dllexport) int main(){
    //build_traj();
    //write_traj();
    std::string task_filename = "./task.txt";
    Task task = create_task(task_filename);
    for (auto obst : task.obst_list()) {
        std::cout << obst.str() << std::endl;
    }
}
