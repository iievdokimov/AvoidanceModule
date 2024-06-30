#include "risk_indices.h"
#include <iostream>

double collision_risk(const Ship& ship, Vector velocity, const Obstacle& obst, const Hyperparams& hyperparams)
{
    // model type == static_obst
    // considering safe sailing towards obst if obst is outside ignore static obst dist
    if (obst.vel().x() == 0 && obst.vel().y() == 0 && 
        (points_dist(ship.pos(), obst.pos()) > hyperparams.ignore_static_obst_dist)) {
        return 0.0;
    }

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
    if (ship.vel().magnitude() != 0)
        speed_ratio = obst_speed / ship.vel().magnitude();


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
    /*
    std::cout << obst.str() << std::endl;
    std::cout << "factors: dcpa=" << dcpa << ", tcpa=" << tcpa << ", distance=" << distance << ", rp="  << relative_position_angle << ", speedratio=" << speed_ratio << std::endl;
    std::cout << "f_factors: f_dcpa=" << f_dcpa << ", f_tcpa=" << f_tcpa << ", f_distance=" << f_distance << ", f_rp=" << f_rp_danger << ", f_speedratio=" << f_speed_ratio << std::endl;
    std::cout << std::endl;
    */
    //double flag;
    //std::cin >> flag;

    double res_value = (f_dcpa * factors_weights[0] + f_tcpa * factors_weights[1] + f_distance * factors_weights[2] +
        f_rp_danger * factors_weights[3] + f_speed_ratio * factors_weights[4]);

    return res_value;
}

std::pair<double, double> calculate_dcpa_tcpa(const Ship& ship, Vector vel, const Obstacle& obst)
{
    double x1 = ship.pos().x(), y1 = ship.pos().y(), z1 = ship.pos().z();
    double Vx1 = vel.x(), Vy1 = vel.y(), Vz1 = vel.z();
    double x2 = obst.pos().x(), y2 = obst.pos().y(), z2 = obst.pos().z(),
        Vx2 = obst.vel().x(), Vy2 = obst.vel().y(), Vz2 = obst.vel().z();
    
    Vector R(x2 - x1, y2 - y1, z2 - z1);
    Vector V_rel(Vx2 - Vx1, Vy2 - Vy1, Vz2 - Vz1);

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
    Vector dcpa_vec(x2 - x1 + tcpa * (Vx2 - Vx1), y2 - y1 + tcpa * (Vy2 - Vy1), z2 - z1 + tcpa * (Vz2 - Vz1));
    //std::cout << "dcpa_log: dcpa_vec= " << dcpa_vec.str() << std::endl;
    double dcpa = std::max(0.0, dcpa_vec.magnitude() -
        obst.rad() - ship.rad()
    );

    return { dcpa, tcpa };
}

double old_generalized_rating_func(double arg, double arg_bound, double arg_bound_mult)
{
    double shuffle = 20;
    double delta = pow(arg_bound, 2) / 2;
    double max_val = delta + pow((arg_bound + shuffle), 2);
    if (arg < arg_bound) {
        return (pow((-arg + arg_bound + shuffle), 2) + delta) / max_val;
    }
    else if (arg < arg_bound_mult) {
        // ax + b = y
        double mult = arg_bound_mult / arg_bound;
        double a = delta / (arg_bound * (1 - mult));
        double b = -a * arg_bound * mult;
        return (a * arg + b) / max_val;
    }
    else {
        return 0;
    }
}

double f1_in_safe(double x, double x_mult) {
    return pow(((x - x_mult) / x_mult), 2);
}

double f2_in_safe(double x, double x_mult) {
    // ax + b = y
    double b = 1;
    double a = -1 / x_mult;
    return a * x + b;
}

double f3_in_safe(double x, double x_mult) {
    return -pow((x / x_mult), 3) + 1;
}

double f3_generalized_rating_func(double x, double x_safe, double x_track)
{
    // DEBUG ONLY:

    //return old_generalized_rating_func(x, x_safe, x_track);




    double y_step = 0.35;
    double y_step_lower = 0.15;
    if (x < x_safe) {
        return f1_in_safe(x, x_safe) * (1 - y_step) + y_step;
    }
    else if (x < x_track) {
        // ax + b = y
        double a = y_step_lower / (x_safe - x_track);
        double b = -a * x_track;
        return a * x + b;
    }
    else {
        return 0;
    }
}

double dist_index(double dist, double safe_dist) 
{
    return f3_generalized_rating_func(dist, 2*safe_dist, 8*safe_dist);
}

double dist_static_index(double dist, double safe_dist)
{
    return f3_generalized_rating_func(dist, safe_dist, 1.5*safe_dist);
}

double dcpa_index(double dcpa, double safe_dist)
{
    return f3_generalized_rating_func(dcpa, 2*safe_dist, 8*safe_dist);
}

double dcpa_static_index(double dcpa, double safe_dist)
{
    return f3_generalized_rating_func(dcpa, safe_dist, 2*safe_dist);
}

double tcpa_index(double tcpa, double safe_time)
{
    return f3_generalized_rating_func(tcpa, 2*safe_time, 5*safe_time);
}

double tcpa_static_index(double tcpa, double safe_time)
{
    return f3_generalized_rating_func(tcpa, safe_time, 1.5*safe_time);
}

/*
double rp_index(double angle)
{   
    if (angle >= 340 or angle <= 20){
        return 1;
    }
    double max_value = pow(180, 2);
    double k = pow((20 - 180), 2) / max_value;
    return pow((angle - 180), 2) / max_value / k;
}

double rp_static_index(double angle)
{
    if (angle >= 345.0 or angle <= 15.0) {
        return 1;
    }

    else if (angle >= 245.0 and angle < 315.0) {
        return (angle - 245.0) / (double)230.0;
    }
    else if (angle >= 45.0 and angle < 115.0) {
        return (115.0 - angle) / (double)230.0;
    }
    
    if (angle >= 315.0) {
        return (angle - 315.0) / 45.0 + (double)70.0 / 230.0;
    }
    else if (angle < 45.0) {
        return (45.0 - angle) / 45.0 + (double)70.0 / 230.0;
    }
    else {
        return 0;
    }
}
*/

double rp_index(double angle)
{
    double max_value = pow(180, 2);
    return pow((angle - 180), 2) / max_value;
}

double rp_static_index(double angle)
{
    if (angle >= 245.0 and angle < 315.0) {
        return (angle - 245.0) / (double)280.0;
    }
    else if (angle >= 45.0 and angle < 115.0) {
        return (115.0 - angle) / (double)280.0;
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
