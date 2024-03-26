#include "task.h"

#include <iostream>

Task create_task(const std::string& task_filename){
    std::ifstream task_file(task_filename);
    double scale = 0.25;
    if (task_file.is_open())
    {
        double x, y, vx, vy, rad, radar_rad, max_speed;
        int type, id;
        task_file >> x >> y >> vx >> vy >> rad >> type >> id >> max_speed >> radar_rad;
        //std::cout << "reading ship: " << x << " " << y << " " << vx << " " << vy << " " << " " << rad << " " << (ModelType)type <<
        //    " " << max_speed << " " << radar_rad << "  ended.\n";
        Ship task_ship = Ship(Vector(x, y), Vector(vx, vy), rad, (ModelType)type, id, max_speed, radar_rad);
        task_file >> x >> y;
        Vector task_target{x, y};
        //std::cout << "task ship: " << task_ship.str() << std::endl;
        std::vector<Obstacle> task_obst_list;
        while (task_file >> x >> y >> vx >> vy >> rad >> type >> id)
        {
            //std::cout << "reading obst: " << x << " " << y << " " << vx << " " << vy << " " << " " << rad << " " << (ModelType)type << "  ended.\n";
            Obstacle task_obst(Vector(x, y), Vector(vx, vy), rad, (ModelType)type, id);
            task_obst_list.push_back(task_obst);
        }

        Task res_task{task_ship, task_target, task_obst_list, scale };
        task_file.close();
        return res_task;

    }
    else {
        std::cout << "error opening file: " << task_filename << std::endl;
    }
    task_file.close();
}
