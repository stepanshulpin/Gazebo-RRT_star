#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "BoxExpanded.hh"
#include "ModelExpanded.hh"
#include "RRT_star.hh"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
class LaunchWorld : public WorldPlugin
{
private: 
	VectorOf2d path;
	ModelExpanded* model;
	physics::WorldPtr parent;
	double modelSize=0.5;/*bad practice*/

private: event::ConnectionPtr updateConnection;

public: 
void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
{	
	this->parent=_parent;
	ignition::math::Vector2d minPlace(-10,-10);
	ignition::math::Vector2d maxPlace(10,10);
	ignition::math::Vector2d init(-9,-9);
	ignition::math::Vector2d goal(9,9);

	transport::NodePtr node(new transport::Node());
	node->Init(_parent->Name());
	transport::PublisherPtr factoryPub =
			node->Advertise<msgs::Factory>("~/factory");
	msgs::Factory msg;

	ignition::math::Vector2d zeroAngle(1,0);
	ignition::math::Vector2d angleVec = goal-init;

	double angle = getAngle(zeroAngle,angleVec);
	msg.set_sdf_filename("model://robot");
	msgs::Set(msg.mutable_pose(),
			ignition::math::Pose3d(
					ignition::math::Vector3d(init.X(), init.Y(), 0),
					ignition::math::Quaterniond(0, 0, angle)));

	factoryPub->Publish(msg);

	BoxExpanded box1("box_1", ignition::math::Vector3d(-10,-10,0),
			ignition::math::Vector3d(0,-9.75,0.25));
	BoxExpanded box2("box_2", ignition::math::Vector3d(-0.25,-9.75,0),
			ignition::math::Vector3d(0,0,0.25));
	BoxExpanded box3("box_3", ignition::math::Vector3d(-10,-8,0),
			ignition::math::Vector3d(-2,-7.75,0.25));
	BoxExpanded box4("box_4", ignition::math::Vector3d(-2.25,-7.75,0),
			ignition::math::Vector3d(-2,0,0.25));
	BoxExpanded box5("box_5", ignition::math::Vector3d(-4.25,0,0),
			ignition::math::Vector3d(-4,10,0.25));
	BoxExpanded box6("box_6", ignition::math::Vector3d(-4,9.75,0),
			ignition::math::Vector3d(10,9.99,0.25));
	BoxExpanded box7("box_7", ignition::math::Vector3d(-0.25,0,0),
			ignition::math::Vector3d(10,0.25,0.25));
	BoxExpanded box8("box_8", ignition::math::Vector3d(9.75,0.25,0),
			ignition::math::Vector3d(10,9.75,0.25));
	BoxExpanded box9("box_9", ignition::math::Vector3d(-2.25,2,0),
			ignition::math::Vector3d(-2,8,0.25));
	BoxExpanded box10("box_10", ignition::math::Vector3d(-2,2,0),
			ignition::math::Vector3d(8,2.25,0.25));
	BoxExpanded box11("box_11", ignition::math::Vector3d(7.75,2.25,0),
			ignition::math::Vector3d(8,8,0.25));
	BoxExpanded box12("box_12", ignition::math::Vector3d(-2,7.75,0),
			ignition::math::Vector3d(7.75,8,0.25));
	BoxExpanded box13("box_13", ignition::math::Vector3d(-4,0,0),
			ignition::math::Vector3d(-2,0.25,0.25));

	insertModel(&box1,_parent);
	insertModel(&box2,_parent);
	insertModel(&box3,_parent);
	insertModel(&box4,_parent);
	insertModel(&box5,_parent);
	insertModel(&box6,_parent);
	insertModel(&box7,_parent);
	insertModel(&box8,_parent);
	insertModel(&box9,_parent);
	insertModel(&box10,_parent);
	insertModel(&box11,_parent);
	insertModel(&box12,_parent);
	insertModel(&box13,_parent);

	std::vector<BoxExpanded> boxes={box1,box2,box3,box4,box5,box6,box7,box8,box9,box10,box11,box12,box13};
	int N_steps=1000;
	double p=0.1;
	double accuracy=0.3;
	RRT_star alg(minPlace,maxPlace,init,goal,N_steps,p,accuracy,boxes,modelSize);
	alg.getPath(path);

	/*path.push_back(ignition::math::Vector2d(-5,-8));
	path.push_back(ignition::math::Vector2d(-9,-9));*/
	if(path.size()>0)
	{
		std::cout<<"path"<<std::endl;
		for(auto &i:path)
		{
			std::cout<<i<<"   ";
		}
		std::cout<<std::endl;
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind(&LaunchWorld::OnUpdate, this));
	}

}
private:
void insertModel(BoxExpanded* box, physics::WorldPtr _parent)
{
	sdf::SDF boxSDF;
	boxSDF.SetFromString(box->GetSDF());
	_parent->InsertModelSDF(boxSDF);
}

double getAngle(ignition::math::Vector2d vec1,ignition::math::Vector2d vec2)
{
	double scalar=vec1.X()*vec2.X()+vec1.Y()*vec2.Y();
	return std::acos(scalar/(length(vec1)*length(vec2)));
}

double length(ignition::math::Vector2d vec)
{
	return std::sqrt(std::pow(vec.X(),2)+std::pow(vec.Y(),2));
}

public: 
void OnUpdate()
{	
	if(model){
		if(!model->isReachedGoal())
			model->calcControl();
	}
	else{
		if(parent->ModelByName("my_robot"))
		{
			model=new ModelExpanded(parent->ModelByName("my_robot"),path);
		}
	}
}

};
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(LaunchWorld)
}
