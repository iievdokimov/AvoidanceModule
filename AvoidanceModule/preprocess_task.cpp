#include "preprocess_task.h"

std::vector<Obstacle> preprocess_coastline(const std::vector<Vector>& coastline, double obsts_rad, unsigned int start_id)
{
	std::vector<Obstacle> res;
	if (coastline.size() == 1) {
		return { Obstacle(Vector(coastline[0].x(), coastline[0].y()), Vector(0, 0), obsts_rad, ModelType::static_obst, start_id) };
	}
	
	unsigned int cur_id = start_id;
	for (int i = 1; i < coastline.size(); ++i) {
		for (const auto& pos : get_line_obsts(coastline[i - 1], coastline[i], obsts_rad)) {
			res.push_back(Obstacle(pos, Vector(0, 0), obsts_rad, ModelType::static_obst, cur_id));
			cur_id++;
		}
	}


	return res;
}

std::vector<Vector> get_line_obsts(Vector start, Vector end, double obsts_rad)
{
	std::vector<Vector> res;
	Vector vec_to_end = end.sub(start);
	int steps = ceil(vec_to_end.magnitude() / obsts_rad);
	for (int i = 0; i < steps; ++i) {
		double k = (double)i / steps;
		res.push_back(start.add(vec_to_end.mul(k)));
	}

	return res;
}
