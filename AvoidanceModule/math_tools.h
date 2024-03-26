#ifndef MATH_TOOLS
#define MATH_TOOLS

#include <cmath>
#include <math.h>
#include <vector>
#include <string>

class Vector {
public:
	Vector() :_x{ 0 }, _y{ 0 } {}
	Vector(double x, double y) :_x{ x }, _y{ y } {}

	
	double magnitude() const;

	Vector add(const Vector& arg) const;
	Vector sub(const Vector& arg) const;
	Vector mul(double k) const;
	
	double x() const { return _x; };
	double y() const { return _y; };

	double dot(const Vector& arg) const;

	std::string str() const;
private:
	double _x, _y;
};


double points_dist(Vector a, Vector b);

double radians(double deg_angle);

double degrees(double rad_angle);

double deg_signed_angle(Vector v1, Vector v2);

double rad_signed_angle(Vector v1, Vector v2);

double deg_unsigned_angle(Vector v1, Vector v2);

double rad_clockwise_angle(Vector v1, Vector v2);

double deg_clockwise_angle(Vector v1, Vector v2);

Vector rotate_vector(Vector vec, double angle);

std::vector<Vector> get_sector_vecs(double start_angle, double end_angle, double len_vec, double step_angle);

bool in_sector(Vector point_vec, Vector vec1, Vector vec2);

Vector get_directional_vec(Vector pos, Vector target_pos);

#endif // MATH_TOOLS
