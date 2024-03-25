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
	ModelObject(Vector pos, Vector vel, double rad, ModelType type, int id);

	ModelType type() const { return _type; };

	Vector pos() const { return _pos; };
	Vector vel() const { return _vel; };
	double rad() const { return _rad; };
	// Vector acceleration() const;
	const std::vector <ModelState>& traj() const { return _traj; };
	
	void move(int steps, double step_t = 1);
	double vx() const { return _vel.x(); };
	double vy() const { return _vel.y(); };
	int id() const { return _id; };

	void set_vel(Vector velocity);

	virtual std::string str() const;

private:
	ModelType _type;
	Vector _pos;
	Vector _vel;
	double _rad;
	int _id;
	std::vector <ModelState> _traj;

};



class Ship: public ModelObject{
public:
	Ship(Vector pos, Vector vel, double rad, ModelType type, int id, double max_speed, double radar_radius) :
		ModelObject(pos, vel, rad, type, id), _max_speed{ max_speed }, _radar_rad{ radar_radius } {}

	double max_speed() const { return _max_speed; };
	double radar_rad() const { return _radar_rad; };

	std::string str() const override;

private:
	double _max_speed;
	double _radar_rad;
};


class Obstacle : public ModelObject {
public:
	Obstacle(Vector pos, Vector vel, double rad, ModelType type, int id) :
		ModelObject(pos, vel, rad, type, id) {}

private:

};




#endif // MODELS