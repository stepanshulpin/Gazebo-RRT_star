#include "ModelExpanded.hh"

double length(ignition::math::Vector2d vec)
{
	return std::sqrt(std::pow(vec.X(),2)+std::pow(vec.Y(),2));
}

ModelExpanded::ModelExpanded(physics::ModelPtr _model, VectorOf2d _path)
{
	model=_model;
	path=_path;
	reachedGoal=false;
	trueAngle=false;
	pointIndex=1;
}

bool ModelExpanded::isReachedGoal()
{
	return reachedGoal;
}

void ModelExpanded::calcControl()
{
	ignition::math::Vector2d previousPoint(path[path.size()-pointIndex]);
	ignition::math::Vector2d targetPoint(path[path.size()-pointIndex-1]);
	ignition::math::Pose3<double> curPose = model->RelativePose();
	ignition::math::Vector2d currentPoint(curPose.Pos().X(),curPose.Pos().Y());	

	ignition::math::Vector2d dif = targetPoint-previousPoint;

	ignition::math::Quaternion<double> currentRotation = curPose.Rot();
	if(!init)
	{
		previousAngle=currentRotation.Yaw();
		init=true;
	}
	ignition::math::Vector2d zeroAngle(1,0);
	double angle = std::atan2(dif.Y(),dif.X());
	if(!trueAngle){
		if(isBetween(previousAngle,angle,currentRotation.Yaw()))
		{
			/*std::cout<<"angle--->"<<angle<<std::endl;
			std::cout<<"currentRotation.Yaw--->"<<currentRotation.Yaw()<<std::endl;*/
			stop();
			trueAngle=true;
		}
		else
		{
			setAngularVel(angle-previousAngle);
		}
	}
	else
	{
		dif.Normalize();
		if(isBetween(previousPoint,currentPoint,targetPoint))
		{
			stop();
			if(pointIndex<(path.size()-1))
			{
				pointIndex++;
				trueAngle=false;
			}
			else{
				reachedGoal=true;
			}
		}
		else
		{
			setLinearVel(dif);
		}
	}

}

void ModelExpanded::stop()
{
	model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
	model->SetAngularVel(ignition::math::Vector3d(0,0,0));
}

void ModelExpanded::setLinearVel(ignition::math::Vector2d vec)
{
	this->model->SetLinearVel(ignition::math::Vector3d(vec.X(), vec.Y(), 0));
}
bool ModelExpanded::isBetween(double angle1,double angle2,double curAngle)
{
	if(angle1==angle2)
		return true;
	else
	{
		if(angle2==curAngle)
			return true;
		else
			return (std::min(angle1,curAngle)<angle2) && (angle2<std::max(angle1,curAngle));
	}
}

bool ModelExpanded::isBetween(ignition::math::Vector2d p1,ignition::math::Vector2d p2,ignition::math::Vector2d m)
{
	if(p1==m)
		return false;
	else{
		if(p2==m)
			return true;
		else
		{
			ignition::math::Vector2d a=p1-m;
			ignition::math::Vector2d b=p2-m;
			double sc=a.X()*b.X()+a.Y()*b.Y();
			return sc>0?false:true;
		}
	}
}
double ModelExpanded::getAngle(ignition::math::Vector2d vec1,ignition::math::Vector2d vec2)
{
	ignition::math::Vector2d vec=vec2-vec1;
	return std::atan2(vec.X(), vec.Y());
}

void ModelExpanded::setAngularVel(double angle)
{
	this->model->SetAngularVel(ignition::math::Vector3d(0,0,angle>0?0.3:-0.3));
}



