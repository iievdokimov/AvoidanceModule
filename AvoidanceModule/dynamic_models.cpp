#include "dynamic_models.h"

std::vector<Vector> CasualDynamicModel::all_possible_velocities() const
{
	std::vector<Vector> res;
	res.push_back({ 20, -20 });
	return res;
}
