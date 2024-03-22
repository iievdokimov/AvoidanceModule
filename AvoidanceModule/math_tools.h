#ifndef MATH_TOOLS
#define MATH_TOOLS

#include <cmath>


class Vector {
public:
	Vector(double x, double y) :_x{ x }, _y{ y } {}
	
	double magnitude() const;

	Vector add(const Vector& arg);
	Vector sub(const Vector& arg);
	Vector mul(double k);
	
	double x() const { return _x; };
	double y() const { return _y; };
private:
	double _x, _y;
};


#endif
