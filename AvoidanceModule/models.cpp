#include "models.h"

ModelObject::ModelObject(Vector pos, Vector vel, double rad, ModelType type) 
	: _pos{ pos }, _vel{ vel }, _rad{ rad }, _type{ type } { }

std::string ModelObject::str() const
{

	std::string res = "ModelObj(";
	res = res + std::to_string(_pos.x()) + ", " + std::to_string(_pos.y()) + ", " + std::to_string(_vel.x()) + ", "
		+ std::to_string(_vel.y()) + ", " + std::to_string(_rad) + ", " + std::to_string(_type) + ")";
	return res;
}

/*
void ModelObject::move(int steps, double step_t){
	//for (int i = 0; i < steps; ++i) {}
	_traj.push_back({ _pos, _vel });
	_pos.add(_vel.mul(step_t));
}
*/