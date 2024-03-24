#include "risk_indices.h"
#include <iostream>

double collision_risk(const Ship& ship, Vector velocity, const Obstacle& obst, const Hyperparams& hyperparams)
{
    Vector vec_to_obst = obst.pos().sub(ship.pos());
    std::pair<double, double> dcpa_tcpa = calculate_dcpa_tcpa(ship, velocity, obst);
    double dcpa = dcpa_tcpa.first, tcpa = dcpa_tcpa.second;
    //ship_pos, obst_pos = copy.deepcopy(ship.pos()), copy.deepcopy(obst.pos())
    Vector pos_moved_ship = ship.pos().add(velocity);
    Vector pos_moved_obst = obst.pos().add(obst.vel());
    double distance = std::max(0.0, (pos_moved_obst.sub(pos_moved_ship)).magnitude() - obst.rad() - ship.rad());
    double relative_position_angle = deg_clockwise_angle(velocity, vec_to_obst);
    double obst_speed = obst.vel().magnitude();
    double speed_ratio = 0;
    if (obst_speed != 0)
        speed_ratio = ship.vel().magnitude() / obst_speed;


    // factors
    // f_dcpa, f_tcpa, f_distance, f_rp_danger, f_speed_ratio;
    double f_dcpa = 0 , f_tcpa = 0, f_distance = 0, f_rp_danger = 0, f_speed_ratio = 0;
    if (obst.vel().magnitude() != 0) {
        if (dcpa != -1 and tcpa != -1){
            f_dcpa = dcpa_index(dcpa, hyperparams.safe_dist);
            f_tcpa = tcpa_index(tcpa, hyperparams.safe_time);
        }
        f_distance = dist_index(distance, hyperparams.safe_dist);
        f_rp_danger = rp_index(relative_position_angle);
        if (speed_ratio != 0)
            f_speed_ratio = speed_ratio_index(speed_ratio);
    }
    else {
        if (relative_position_angle < 70 or 290 < relative_position_angle) {

            if (dcpa != -1 and tcpa != -1) {
                f_dcpa = dcpa_static_index(dcpa, hyperparams.safe_dist);
                f_tcpa = tcpa_static_index(tcpa, hyperparams.safe_time);
            }
            f_distance = dist_static_index(distance, hyperparams.safe_dist);
            f_rp_danger = rp_static_index(relative_position_angle);
        }
    }


    const std::vector<double>& factors_weights = hyperparams.risk_weights;

    //debug_print_cr_data();
    std::cout << obst.str() << std::endl;
    std::cout << "factors: dcpa=" << dcpa << ", tcpa=" << tcpa << ", distance=" << distance << ", rp="  << relative_position_angle << ", speedratio=" << speed_ratio << std::endl;
    std::cout << "f_factors: f_dcpa=" << f_dcpa << ", f_tcpa=" << f_tcpa << ", f_distance=" << f_distance << ", f_rp=" << f_rp_danger << ", f_speedratio=" << f_speed_ratio << std::endl;
    std::cout << std::endl;
    //double flag;
    //std::cin >> flag;

    double res_value = (f_dcpa * factors_weights[0] + f_tcpa * factors_weights[1] + f_distance * factors_weights[2] +
        f_rp_danger * factors_weights[3] + f_speed_ratio * factors_weights[4]);

    return res_value;
}

std::pair<double, double> calculate_dcpa_tcpa(const Ship& ship, Vector vel, const Obstacle& obst)
{
    double x1 = ship.pos().x(), y1 = ship.pos().y();
    double Vx1 = vel.x(), Vy1 = vel.y();
    double x2 = obst.pos().x(), y2 = obst.pos().y(), Vx2 = obst.vel().x(), Vy2 = obst.vel().y();
    
    Vector R(x2 - x1, y2 - y1);
    Vector V_rel(Vx2 - Vx1, Vy2 - Vy1);

    //std::cout << "dcpa_log: R, V_rel = " << R.str() << " " << V_rel.str() << std::endl;

    double relative_speed_projection = R.dot(V_rel) / R.magnitude();
    
    //std::cout << "dcpa_log: rsp = " << relative_speed_projection << std::endl;

    // for case of moving further from each other
    // scalar - mult >= 0 means that angle between R and V_rel is <= 0.5 * Pi
    // cause R is directed from ship to obst
    // POSITIVE  projection on MINUS R direction
    // means moving away in relative
    //  (if R would be directed from obst to ship sense of signs changes)
    if (relative_speed_projection >= 0) {
        return { -1, -1 };
    }
    // NEGATIVE projection means moving closer
    relative_speed_projection *= -1;
    double relative_dist = std::max(0.0, R.magnitude() - obst.rad() - ship.rad());

    //std::cout << "dcpa_log: relative dist, rsp-1= " << relative_dist << " " << relative_speed_projection << std::endl;
    double tcpa = relative_dist / abs(relative_speed_projection);
    //std::cout << "dcpa_log: tcpa= " << tcpa << std::endl;
    Vector dcpa_vec(x2 - x1 + tcpa * (Vx2 - Vx1), y2 - y1 + tcpa * (Vy2 - Vy1));
    //std::cout << "dcpa_log: dcpa_vec= " << dcpa_vec.str() << std::endl;
    double dcpa = std::max(0.0, dcpa_vec.magnitude() -
        obst.rad() - ship.rad()
    );

    return { dcpa, tcpa };
}

double generalized_rating_func(double arg, double arg_bound, double mult)
{
    double shuffle = 20;
    double delta = pow(arg_bound, 2) / 2;
    double max_val = delta + pow((arg_bound + shuffle), 2);
    if (arg < arg_bound) {
        return (pow((-arg + arg_bound + shuffle), 2) + delta) / max_val;
    }
    else if (arg < mult * arg_bound) {
        // ax + b = y
        double a = delta / (arg_bound * (1 - mult));
        double b = -a * arg_bound * mult;
        return (a * arg + b) / max_val;
    }
    else {
        return 0;
    }
}

double dist_index(double dist, double safe_dist) 
{
    return generalized_rating_func(dist, 2 * safe_dist, 4);
}

double dcpa_index(double dcpa, double safe_dist)
{
    return generalized_rating_func(dcpa, 2 * safe_dist, 4);
}

double tcpa_index(double tcpa, double safe_time)
{
    return generalized_rating_func(tcpa, 2 * safe_time, 5);
}

double rp_index(double angle)
{
    double max_value = pow(180, 2);
    return pow((angle - 180), 2) / max_value;
}

double rp_static_index(double angle)
{
    if (angle >= 245.0 and angle < 315.0) {
        return (angle - 270.0) / (double) 180.0;
    }
    else if (angle >= 45.0 and angle < 115.0) {
        return (90.0 - angle) / (double) 180.0;
    }
    else if (angle >= 315.0) {
        return (angle - 45.0 - 270.0) / 60.0 + (double)45.0 / 180.0;
    }
    else if (angle < 45.0) {
        return (60.0 - angle) / (double)60.0;
    }
    else {
        return 0;
    }   
}

double speed_ratio_index(double K)
{
    K = std::max(K, pow(0.1, 8));
    return (double)1.0 / (1.0 + (double)1.0 / pow(K, 2));
}

double dcpa_static_index(double dcpa, double safe_dist)
{
    return generalized_rating_func(dcpa, safe_dist, 2.0);
}

double tcpa_static_index(double tcpa, double safe_time)
{
    return generalized_rating_func(tcpa, safe_time, 1.5);
}

double dist_static_index(double dist, double safe_dist)
{
    return generalized_rating_func(dist, safe_dist, 1.5);
}


