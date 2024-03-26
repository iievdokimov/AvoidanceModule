#ifndef DYN_MODEL
#define DYN_MODEL


#include "models.h"
#include "math_tools.h"


class CasualDynamicModel {
public:
	CasualDynamicModel(const Ship& ship, double sector)
		: ship{ ship }, possible_turns_sector{ sector }, _prev_nonzero_vel{ ship.vel() } { };

	std::vector<Vector> all_possible_velocities() const;

	double get_course_angle() const;
	Vector prev_nonzero_vel() const { return _prev_nonzero_vel;  };
private:
	const Ship& ship;
	double possible_turns_sector;
	const Vector _Ox_axis{ 1, 0 };
	Vector _prev_nonzero_vel;
};



#endif //DYN_MODEL