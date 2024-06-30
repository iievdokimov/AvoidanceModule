#ifndef MATH_TOOLS
#define MATH_TOOLS

#include <cmath>
#include <math.h>
#include <vector>
#include <string>
#include <chrono>

#include <glm/glm.hpp>
#include "iostream"

class Vector {
public:
	Vector() :_x{ 0 }, _y{ 0 }, _z{ 0 } {}
	Vector(double x, double y, double z) :_x{ x }, _y{ y }, _z{ z } {}
	//Vector(double x, double y, double z = 0) :_x{ x }, _y{ y }, _z{ z } {}

	
	double magnitude() const;

	Vector add(const Vector& arg) const;
	Vector sub(const Vector& arg) const;
	Vector mul(double k) const;
	Vector normed() const;
	
	double x() const { return _x; };
	double y() const { return _y; };
	double z() const { return _z; };

	double dot(const Vector& arg) const;

	std::string str() const;
private:
	double _x, _y, _z;
};


double points_dist(Vector a, Vector b);

double radians(double deg_angle);

double degrees(double rad_angle);

double deg_signed_angle(Vector v1, Vector v2);

double rad_signed_angle(Vector v1, Vector v2);

double deg_unsigned_angle(Vector v1, Vector v2);

double rad_clockwise_angle(Vector v1, Vector v2);

double deg_clockwise_angle(Vector v1, Vector v2);

//Vector rotate_vector(Vector v, Vector k, double theta);

glm::dvec3 rotate_vector_glm(const glm::dvec3& v, const glm::dvec3& k, double theta);

std::vector<Vector> get_sector_vecs(Vector axis, double angle_step_slope, double angle_step_circle, double max_turn_angle);

bool in_sector(Vector point_vec, Vector vec1, Vector vec2);

Vector get_directional_vec(Vector pos, Vector target_pos);

#endif // MATH_TOOLS
