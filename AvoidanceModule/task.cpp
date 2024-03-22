#include "task.h"

#include <iostream>

Task create_task(const std::string& task_filename){
    std::ifstream task_file(task_filename);
    if (task_file.is_open())
    {
        double x, y, vx, vy, rad;
        int type;
        task_file >> x >> y >> vx >> vy >> rad >> type;
        //std::cout << "reading ship: " << x << " " << y << " " << vx << " " << vy << " " << " " << rad << " " << (ModelType)type << "  ended.\n";
        Ship task_ship = Ship(Vector(x, y), Vector(vx, vy), rad, (ModelType)type);
        task_file >> x >> y;
        Vector task_target{x, y};

        std::vector<ModelObject> task_obst_list;
        while (task_file >> x >> y >> vx >> vy >> rad >> type)
        {
            //std::cout << "reading obst: " << x << " " << y << " " << vx << " " << vy << " " << " " << rad << " " << (ModelType)type << "  ended.\n";
            ModelObject task_obst = ModelObject(Vector(x, y), Vector(vx, vy), rad, (ModelType)type);
            task_obst_list.push_back(task_obst);
        }

        Task res_task{task_ship, task_target, task_obst_list };
        return res_task;

    }
    else {
        std::cout << "error opening file: " << task_filename << std::endl;
    }
    task_file.close();
    
}
