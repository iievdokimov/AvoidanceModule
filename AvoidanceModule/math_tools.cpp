#include "math_tools.h"


double Vector::magnitude() const
{
	return std::sqrt(_x * _x + _y * _y + _z * _z);
}

Vector Vector::add(const Vector& arg) const
{
	return Vector(arg._x + _x, arg._y + _y, arg._z + _z);
}

Vector Vector::sub(const Vector& arg) const
{
	return Vector(_x - arg._x, _y - arg._y, arg._z - _z);
}

Vector Vector::mul(double k) const
{
	return Vector(_x * k, _y * k, _z * k);
}

Vector Vector::normed() const
{	
	double norm = magnitude();
	return Vector(_x / norm, _y / norm, _z / norm);
}

double Vector::dot(const Vector& arg) const
{
	return _x * arg._x + _y * arg._y + _z * arg._z;
}

std::string Vector::str() const
{
	return "Vector(" + std::to_string(_x) + ", " + std::to_string(_y) + ", " + std::to_string(_z) + ")";
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


glm::dvec3 rotate_vector_glm(const glm::dvec3& v, const glm::dvec3& k, double theta)
{
	// v: a vector in 3D space
	// k: a unit vector describing the axis of rotation
	// theta: the angle (in radians) that v rotates around k
	double cos_theta = cos(theta);
	double sin_theta = sin(theta);
	glm::dvec3 rotated = (v * cos_theta) + (glm::cross(k, v) * sin_theta) + (k * glm::dot(k, v)) * (1 - cos_theta);
	return rotated;
}

std::vector<Vector> get_sector_vecs(Vector axis, double angle_step_slope, double angle_step_circle, double max_turn_angle)
{
	std::vector<Vector> res;
	res.push_back(axis);

	double slope_angle = angle_step_slope;

	//std::cout << "Input axis : " << axis.str() << std::endl;

	glm::vec3 v_axis = { axis.x(), axis.y(), axis.z()};
	glm::vec3 any = v_axis + glm::vec3(1, 1, 1);
	glm::vec3 perpendicular = glm::cross(v_axis, any);
	perpendicular = glm::normalize(perpendicular);

	//std::cout << "perpendicular : " << perpendicular.x << " " << perpendicular.y << " " << perpendicular.z << std::endl;
	while (slope_angle <= max_turn_angle) {
		// new circle
		glm::vec3 new_circle_vec = rotate_vector_glm(v_axis, perpendicular, radians(slope_angle));

		//std::cout << "New slope : " << slope_angle << std::endl;
		//std::cout << "New circle vec = " << new_circle_vec.x << " " << new_circle_vec.y << " " << new_circle_vec.z << std::endl;
		double circle_angle = 0;

		//Vector vec(new_circle_vec.x, new_circle_vec.y, new_circle_vec.z);
		//res.push_back(vec);
		//continue;

		int j = 0;
		while (circle_angle <= 360.0) {
			//std::cout << j << std::endl;
			//std::cout << circle_angle << std::endl;
			//++j;
			new_circle_vec = rotate_vector_glm(new_circle_vec, glm::normalize(v_axis), radians(circle_angle));
			Vector vec(new_circle_vec.x, new_circle_vec.y, new_circle_vec.z);
			res.push_back(vec);
			circle_angle += angle_step_circle;
		}
		slope_angle += angle_step_slope;
	}

	//std::cout << "NEW PORTION VECS" << std::endl;
	//for (auto el : res) {
	//	std::cout << el.str() << std::endl;
	//}

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
	Vector delta_pos = target_pos.sub(pos);
	return delta_pos.normed();
}


