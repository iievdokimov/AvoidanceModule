#include "task.h"

#include <iostream>

/*
Task::Task(Ship ship, Vector target, std::vector<Obstacle> obst_list, std::vector<ModelState> follow_trajectory)
    : _ship{ ship }, _target{ target }, _obst_list{ obst_list }, _follow_trajectory{ follow_trajectory }
{
    configure_follow_targets();
};
*/

Task::Task(Ship ship, Vector target, std::vector<Obstacle> obst_list, std::vector<Vector> follow_targets) 
    : _ship{ ship }, _target{ target }, _obst_list{ obst_list }, _follow_targets{ follow_targets }
{
};


void Task::configure_follow_targets()
{
    for (const auto& state : _cur_trajectory) {
        // for i = 0...len_dtraj / diff_follow_targets
        // pos = pos + state.vel * i
        _follow_targets.push_back(state._pos);
    }
}


Task create_task(const std::string& task_filename){
    std::ifstream task_file(task_filename);
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

        Task res_task{task_ship, task_target, task_obst_list };
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
