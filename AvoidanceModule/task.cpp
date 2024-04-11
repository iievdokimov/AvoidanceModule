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

void write_task(const Task& task, const std::string& task_filename)
{
    std::ofstream task_file;
    task_file.open(task_filename);
    if (task_file.is_open())
    {   
        // writing ship
        task_file 
            << task.ship().pos().x() << " " << task.ship().pos().y() << " "
            << task.ship().vel().x() << " " << task.ship().vel().y() << " "
            << task.ship().rad() << " " << (int)task.ship().type() << " "
            << task.ship().id() << " " << task.ship().max_speed() << " " << task.ship().radar_rad()
            << std::endl;
        // writing target
        task_file
            << task.target().x() << " " << task.target().y() << std::endl;
        // writing obsts
        for (const Obstacle& obst : task.obst_list()) {
            task_file
                << obst.pos().x() << " " << obst.pos().y() << " "
                << obst.vel().x() << " " << obst.vel().y() << " "
                << obst.rad() << " " << (int)obst.type() << " "
                << obst.id()
                << std::endl;
        }

    }
    else {
        std::cout << "error opening file: " << task_filename << std::endl;
    }
    task_file.close();
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
    }catch (...)
    {
        std::cout << "Error cleaning directory" << std::endl;
    }
}
