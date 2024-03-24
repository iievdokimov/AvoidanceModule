#ifndef DYN_MODEL
#define DYN_MODEL


#include "models.h"


class CasualDynamicModel {
public:
	CasualDynamicModel(const Ship& ship, double sector) 
		: ship{ ship }, possible_turns_sector{ sector } { };

	std::vector<Vector> all_possible_velocities() const;

private:
	const Ship& ship;
	double possible_turns_sector;
};



#endif //DYN_MODEL