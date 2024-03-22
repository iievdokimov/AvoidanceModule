#include "math_tools.h"


double Vector::magnitude() const {
	return std::sqrt(_x * _x + _y * _y);
}

