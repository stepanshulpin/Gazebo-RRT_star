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
public: 
	void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
	{
		BoxExpanded box1("box_1", ignition::math::Vector3d(1,2,0), 
				ignition::math::Vector3d(2,4,1)); 
		BoxExpanded box2("box_2", ignition::math::Vector3d(-3,-4,0), 
				ignition::math::Vector3d(-2,-2,1)); 
		BoxExpanded box3("box_3", ignition::math::Vector3d(-3,2,0), 
				ignition::math::Vector3d(-2,4,1)); 

		insertModel(&box1,_parent);
		insertModel(&box2,_parent);
		insertModel(&box3,_parent); 	

		std::vector<BoxExpanded> boxes={box1,box2,box3};
		ignition::math::Vector2d minPlace(-10,-10);
		ignition::math::Vector2d maxPlace(10,10);
		ignition::math::Vector2d init(-9,-9);
		ignition::math::Vector2d goal(9,9);
		int N_steps=1000;
		double p=0.05;
		double accuracy=0.3;
		RRT_star alg(minPlace,maxPlace,init,goal,N_steps,p,accuracy,boxes);
		VectorOf2d path;
		alg.getPath(path);
		std::cout<<"path"<<std::endl;
		for(auto &i:path)
		{
			std::cout<<i<<"   ";
		}
		std::cout<<std::endl;
		
	}
	void insertModel(BoxExpanded* box, physics::WorldPtr _parent)
	{
		sdf::SDF boxSDF;
		boxSDF.SetFromString(box->GetSDF());
		_parent->InsertModelSDF(boxSDF);
	}
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(LaunchWorld)
}
