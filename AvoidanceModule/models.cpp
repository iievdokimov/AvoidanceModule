#include "models.h"

ModelObject::ModelObject(Vector pos, Vector vel, double rad, ModelType type, int id) 
	: _pos{ pos }, _vel{ vel }, _rad{ rad }, _type{ type }, _id{ id } { }

void ModelObject::set_vel(Vector velocity)
{
	_vel = velocity;
}

std::string ModelObject::str() const
{
	std::string res = "ModelObj(";
	res = res + std::to_string(_pos.x()) + ", " + std::to_string(_pos.y()) + ", " + std::to_string(_vel.x()) + ", "
		+ std::to_string(_vel.y()) + ", " + std::to_string(_rad) + ", " + std::to_string(_type) + ", " + std::to_string(_id) + ")";
	return res;
}


void ModelObject::move(int steps, double step_t)
{
	if (steps > 0) {
		for (int i = 0; i < steps; ++i) {
			_traj.push_back({ _pos, _vel });
			_pos = _pos.add(_vel.mul(step_t));
		}
	}
	else {
		//
	}
}

std::string Ship::str() const
{
	return "Ship(" + std::to_string(pos().x()) + ", " + std::to_string(pos().y()) + ", " + std::to_string(vel().x()) + ", "
		+ std::to_string(vel().y()) + ", " + std::to_string(rad()) + ", " + std::to_string(type()) + ", " + std::to_string(id()) + 
		", " + std::to_string(max_speed()) + ", " + std::to_string(radar_rad()) + ")";
}
