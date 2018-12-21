#ifndef _MODELEXPANDED_HH_
#define _MODELEXPANDED_HH_
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

typedef std::vector<ignition::math::Vector2d> VectorOf2d;

using namespace gazebo;

class ModelExpanded
{
private: 
	physics::ModelPtr model;
	VectorOf2d path;
	bool reachedGoal;
	bool trueAngle;
	int pointIndex;
	double previousAngle;
	bool init=false;
public:
	ModelExpanded();
	ModelExpanded(physics::ModelPtr _model, VectorOf2d _path);
	ModelExpanded & operator= (const ModelExpanded & another)
	{
		model = another.model;
		path = another.path;	
		reachedGoal = another.reachedGoal;
		trueAngle= another.trueAngle;
		pointIndex = another.pointIndex;
		return *this;
	}
	bool isReachedGoal();
	void calcControl();
	void stop();
	void setLinearVel(ignition::math::Vector2d vec);
	void setAngularVel(double angle);
	bool isBetween(ignition::math::Vector2d p1,ignition::math::Vector2d p2,ignition::math::Vector2d m); /*p1---m---p2*/
	bool isBetween(double angle1,double angle2,double curAngle);
	double getAngle(ignition::math::Vector2d vec1,ignition::math::Vector2d vec2);
	
};

#endif
