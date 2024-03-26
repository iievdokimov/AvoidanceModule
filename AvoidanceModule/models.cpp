#include "models.h"

ModelObject::ModelObject(Vector pos, Vector vel, double rad, ModelType type, int id) 
	: _pos{ pos }, _vel{ vel }, _rad{ rad }, _type{ type }, _id{ id } { }

void ModelObject::set_vel(Vector velocity)
{
	_vel = velocity;
}

std::string ModelObject::str() const
{
	std::string res = "ModelObj(";
	res = res + std::to_string(_pos.x()) + ", " + std::to_string(_pos.y()) + ", " + std::to_string(_vel.x()) + ", "
		+ std::to_string(_vel.y()) + ", " + std::to_string(_rad) + ", " + std::to_string(_type) + ", " + std::to_string(_id) + ")";
	return res;
}


void ModelObject::move(int steps, double step_t)
{
	if (steps > 0) {
		for (int i = 0; i < steps; ++i) {
			_traj.push_back({ _pos, _vel });
			_pos = _pos.add(_vel.mul(step_t));
		}
	}
	else {
		//
	}
}

std::string Ship::str() const
{
	return "Ship(" + std::to_string(pos().x()) + ", " + std::to_string(pos().y()) + ", " + std::to_string(vel().x()) + ", "
		+ std::to_string(vel().y()) + ", " + std::to_string(rad()) + ", " + std::to_string(type()) + ", " + std::to_string(id()) + 
		", " + std::to_string(max_speed()) + ", " + std::to_string(radar_rad()) + ")";
}


double Obstacle::velocity_inside_vo(Vector ship_pos, Vector vel)
{
	if (vo_intersection(Vector(ship_pos.x() + vel.x(), ship_pos.y() + vel.y()))) {
		return 1.0;
	}
	return 0.0;
}


bool Obstacle::vo_intersection(Vector point)
{
    // order is significant !
    sort_collision_cone_edges();
    //collision_cone = self.collision_cone
    Vector point_top = _collision_cone[0];
    // collision cone edges assumed to be sorted
    // p_a is left edge, p_b is right edge
    Vector p_a = _collision_cone[1];
    Vector p_b = _collision_cone[2];
    
    // convert p_c - vec in local coordinates
    Vector p_c(point.x() - point_top.x(), point.y() - point_top.y());
    if (abs(p_c.x()) < 1.0 && abs(p_c.x()) < 1.0) {
        return true;
    }
    // '''TODO: when zero is solution and when not'''
    // zero relative - velocity is not a solution
    // edge of a collision cone
    if (p_c.x() == 0 and p_c.y() == 0) {
        return false;
    }
    // degrees
    double angle_ab = deg_clockwise_angle(p_a, p_b);
    double angle_ac = deg_clockwise_angle(p_a, p_c);
    double angle_cb = deg_clockwise_angle(p_c, p_b);
    
    // vec C between vec A and vec B
    double epsilon_degree = 0.01;
    // epsilon added to let ship choose the edge - velocities
    if (angle_ab >= angle_ac + angle_cb - epsilon_degree) {
        return true;
    }
    return false;
}


void Obstacle::update_collision_cone(const Ship& ship, double safe_dist)
{
    auto res_vo_built = build_vo(ship, safe_dist);
    bool success = res_vo_built.first;
    if (not success)
        return;
    set_collision_cone(res_vo_built.second, ship.pos());
}


void Obstacle::set_collision_cone(std::vector<Vector> vo_edges, Vector ship_pos)
{
    Vector cone_top(ship_pos.x() + vx(), ship_pos.y() + vy());
    _collision_cone = { cone_top, vo_edges[0], vo_edges[1] };
    // SORTING HERE IS UNNEEDED - only for visualization purposes
    sort_collision_cone_edges();
}


void Obstacle::sort_collision_cone_edges()
{
    // vo - sector cannot be more than PI / 2
    // (if ship out of obst-safe-rad)
    Vector left_edge = _collision_cone[1];
    Vector right_edge = _collision_cone[2];
    if (deg_clockwise_angle(left_edge, right_edge) <= 180.0) {
        _collision_cone[1] = left_edge;
        _collision_cone[2] = right_edge;
    }
    else {
        _collision_cone[1] = right_edge;
        _collision_cone[2] = left_edge;
    }
}

std::pair<bool, std::vector<Vector>> Obstacle::build_vo(const Ship& ship, double safe_dist)
{
    /*:
        param ship : object from which perspective collision cone is constructed
        param safe_dist - additional margin to obstacle radius to stay away from obst on the distance of safe dist
      
        : return : True if built was successful, False if ship is inside safe_dist or unsuccessful built, edges of collision cone
    */
    //std::cout << "build vo" << std::endl;
    try {
        double vo_rad = ship.rad() + rad();
        vo_rad += safe_dist;
        
        Vector vec_to_ship(pos().x() - ship.pos().x(), pos().y() - ship.pos().y());
        double dist = vec_to_ship.magnitude();


        //std::cout << "build vo 2" << std::endl;

        // problematic VO built
        // sailing inside safe_dist
        if (vo_rad > dist) {
            // if already inside safe zone 2 possible actions:
            // 1) trying to lower safe dist and build vo again
            // 2) building vo only for obst without adding safe dist

            //error_log = (f"VO_LOGICS_ERROR - vo_rad > dist: vo_rad={vo_rad}, dist={dist}\n"
            //f"FORCED: Reducing safe_dist to 0")
            // logger.save_error("building_VO_error", [error_log])

            double forced_threshold = 0;
            double vo_rad = ship.rad() + rad() + forced_threshold;
            if (vo_rad > dist) {
                std::string log_str = "Reduced vo_rad > dist. Can't build vo while colliding (or inside reduced safe zone).";
                //f"So the ship is inside reduced safe zone.")

                throw std::runtime_error(log_str);
            }
        }

        //std::cout << "build vo 3" << std::endl;

        double sin_alfa = vo_rad / dist;
        if (abs(sin_alfa) > 1.0) {
            sin_alfa = copysign(1.0, sin_alfa);
        }

        double alfa = asin(sin_alfa);

        //relative_pos = vec_to_ship
        // sector = vec_to_obst + -alfa(alfa + -beta)
        double beta = rad_clockwise_angle(Vector(1, 0), vec_to_ship);
            
        double vec_len = dist * cos(alfa);

        double v1x = vec_len * cos(beta - alfa);
        double v1y = vec_len * sin(beta - alfa);
        double v2x = vec_len * cos(beta + alfa);
        double v2y = vec_len * sin(beta + alfa);

        Vector v1(v1x, v1y);
        Vector v2(v2x, v2y);
        // delta_VO_speed = Vector(obst.vx(), obst.vy())
        /*
        v1.x += self.vx()
        v1.y += self.vy()
        v2.x += self.vx()
        v2.y += self.vy()
        */
        v1 = Vector(v1.x() + vx(), v1.y() + vy());
        v2 = Vector(v2.x() + vx(), v2.y() + vy());

        return { true, { v1, v2 } };
    }
    catch (const std::runtime_error& e) {         
        std::cout << "Building VO error: " << e.what() << std::endl;
        // logger.save_error("building_VO_error", [traceback.format_exc()])
        return { false, {} }; //{Vector(0, 0), Vector(0, 0), Vector(0, 0)}};
    }
}

