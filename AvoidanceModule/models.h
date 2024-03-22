#ifndef MODELS
#define MODELS

#include "math_tools.h"
#include <vector>
#include <string>



enum ModelType {
	static_obst = 0,
	dynamic_obst,
	ship_obst
};


struct ModelState {
	Vector _pos;
	Vector _vel;
};


class ModelObject {
public:
	ModelObject(Vector pos, Vector vel, double rad, ModelType type);

	ModelType type() const { return _type; };

	Vector pos() const { return _pos; };
	Vector vel() const { return _vel; };
	double rad() const { return _rad; };
	// Vector acceleration() const;
	const std::vector <ModelState>& traj() const { return _traj; };
	
	//void move(int steps, double step_t);
	double vx() const { return _vel.x(); };
	double vy() const { return _vel.y(); };

	std::string str() const;

private:
	ModelType _type;
	Vector _pos;
	Vector _vel;
	double _rad;
	std::vector <ModelState> _traj;
};



class Ship: public ModelObject{
public:
	Ship(Vector pos, Vector vel, double rad, ModelType type) :
		ModelObject(pos, vel, rad, type) {}

private:

};


class Obstacle : public ModelObject {

};




#endif // MODELS