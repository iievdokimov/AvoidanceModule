#include "math_tools.h"


double Vector::magnitude() const
{
	return std::sqrt(_x * _x + _y * _y);
}

Vector Vector::add(const Vector& arg) const
{
	return Vector(arg._x + _x, arg._y + _y);
}

Vector Vector::sub(const Vector& arg) const
{
	return Vector(_x - arg._x, _y - arg._y);
}

Vector Vector::mul(double k) const
{
	return Vector(_x * k, _y * k);
}

double Vector::dot(const Vector& arg) const
{
	return _x * arg._x + _y * arg._y;
}

std::string Vector::str() const
{
	return "Vector(" + std::to_string(_x) + ", " + std::to_string(_y) + ")";
}

double points_dist(Vector a, Vector b)
{
	return b.sub(a).magnitude();
}

double radians(double deg_angle)
{
	return deg_angle * (acos(-1) / (double)180.0);
}

double degrees(double rad_angle)
{
	return rad_angle * ((double)180.0 / acos(-1));
}

double deg_signed_angle(Vector v1, Vector v2)
{
	return rad_signed_angle(v1, v2) * ((double)180.0 / acos(-1));
}

double rad_signed_angle(Vector v1, Vector v2)
{
	return atan2(v1.x() * v2.y() - v1.y() * v2.x(), v1.x() * v2.x() + v1.y() * v2.y());
}

double deg_unsigned_angle(Vector v1, Vector v2)
{
	return abs(deg_signed_angle(v1, v2));
}

double rad_clockwise_angle(Vector v1, Vector v2)
{
	double signed_angle = atan2(v2.y(), v2.x()) - atan2(v1.y(), v1.x());
	if (signed_angle < 0)
		signed_angle += 2 * acos(-1);
	return signed_angle;
}

double deg_clockwise_angle(Vector v1, Vector v2)
{
	return degrees(rad_clockwise_angle(v1, v2));
}

Vector rotate_vector(Vector vec, double angle)
{
	// angle: degrees
	// return: vector rotated to clockwise
	Vector Ox(1, 0);
	double cur_angle = deg_clockwise_angle(Ox, vec);
	double new_angle = cur_angle + angle;
	double rad_angle = radians(new_angle);
	double len_vec = vec.magnitude();
	double vx = len_vec * cos(rad_angle);
	double vy = len_vec * sin(rad_angle);
	return Vector(vx, vy);
}

std::vector<Vector> get_sector_vecs(double start_angle, double end_angle, double len_vec, double step_angle)
{
	double angle = start_angle;
	std::vector<Vector> res;
	while (angle <= end_angle) {
		Vector vec = rotate_vector(Vector(len_vec, 0), angle);
		res.push_back(vec);
		angle += step_angle;
	}
	return res;
}

bool in_sector(Vector p_c, Vector p_a, Vector p_b)
{
	// p_c = point_vec, p_a == vec1, p_b = vec2
	if (p_c.x() == 0 && p_c.y() == 0)
		return false;

	// degrees
	double angle_ab = deg_clockwise_angle(p_a, p_b);
	double angle_ac = deg_clockwise_angle(p_a, p_c);
	double angle_cb = deg_clockwise_angle(p_c, p_b);

	// vec C between vec A and vec B
	double epsilon_degree = 0.01;
	// epsilon added to let ship choose the edge-velocities
	if (angle_ab >= angle_ac + angle_cb - epsilon_degree)
	{
		return true;
	}
	return false;
}

Vector get_directional_vec(Vector pos, Vector target_pos)
{
	double dx = target_pos.x() - pos.x();
	double dy = target_pos.y() - pos.y();
	double dist = Vector(dx, dy).magnitude();
	return Vector(dx / dist, dy / dist);
}


