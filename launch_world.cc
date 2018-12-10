#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "BoxExpanded.hh"
#include "RRT_star.hh"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
class LaunchWorld : public WorldPlugin
{
private: physics::ModelPtr model;
private: physics::WorldPtr parent;
private: VectorOf2d path;
private: bool complete=false;
private: bool completeRotation=false;
private: bool dal=false;
private: double modelSize=1;/*bad practice*/

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

	BoxExpanded box1("box_1", ignition::math::Vector3d(1,2,0), 
			ignition::math::Vector3d(6,2.5,1)); 
	BoxExpanded box2("box_2", ignition::math::Vector3d(-3,-8,0), 
			ignition::math::Vector3d(-2.5,-2,1)); 
	BoxExpanded box3("box_3", ignition::math::Vector3d(-2.5,2,0), 
			ignition::math::Vector3d(-2,4,1)); 
	BoxExpanded box4("box_4", ignition::math::Vector3d(-7,-2,0), 
			ignition::math::Vector3d(-2.5,-1.5,1)); 

	insertModel(&box1,_parent);
	insertModel(&box2,_parent);
	insertModel(&box3,_parent);
	insertModel(&box4,_parent);

	std::vector<BoxExpanded> boxes={box1,box2,box3,box4};
	int N_steps=10000;
	double p=0.1;
	double accuracy=0.3;
	RRT_star alg(minPlace,maxPlace,init,goal,N_steps,p,accuracy,boxes,modelSize);
	alg.getPath(path);
	std::cout<<"path"<<std::endl;
	for(auto &i:path)
	{
		std::cout<<i<<"   ";
	}
	std::cout<<std::endl;
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&LaunchWorld::OnUpdate, this));

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
	if(this->model){
		if(path.size()>0)
		{
			if(!complete){
				ignition::math::Vector2d curGoal = path[path.size()-2];
				ignition::math::Pose3<double> curPose = this->model->RelativePose();
				ignition::math::Vector2d dif(curGoal.X()-curPose.Pos().X(),curGoal.Y()-curPose.Pos().Y());
				if(!completeRotation)
				{
					ignition::math::Quaternion<double> curRot = curPose.Rot();
					ignition::math::Vector2d vec(1,std::tan(curRot.Z()));
					/*this->model->SetAngularVel(ignition::math::Vector3d(0,0,-getAngle(dif,vec)));*/
					completeRotation=true;
				}
				if(length(dif)>0.1)
				{
					dif.Normalize();
					dif*=std::sqrt(0.1);
					if(!dal)
					{
						this->model->SetLinearVel(ignition::math::Vector3d(dif.X(), dif.Y(), 0));
						dal=true;
					}
				}
				else
				{
					complete=true;
					this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
					std::cout<<"pose "<<curPose.Pos()<<std::endl;
				}
				this->model->SetLinearVel(ignition::math::Vector3d(dif.X(), dif.Y(), 0));
			}

		}
	}
	else{
		this->model=parent->ModelByName("my_robot");
	}
}

};
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(LaunchWorld)
}
