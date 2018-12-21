#ifndef _BOXEXPANDED_HH_
#define _BOXEXPANDED_HH_
#include "gazebo/gazebo.hh"

class BoxExpanded:public ignition::math::Box
{
private:
	std::string name;
	std::vector<ignition::math::Line2<double>> lines;
public:
	BoxExpanded(std::string _name, ignition::math::Vector3d _min, ignition::math::Vector3d _max):ignition::math::Box(_min, _max)
	{
		name=_name;
	}
	std::string GetSDF();
	std::vector<ignition::math::Line2<double>> getLines(double modelSize){
		if(lines.size()==0){
			lines.push_back(ignition::math::Line2<double>(Min().X()-modelSize,Min().Y()-modelSize, Max().X()+modelSize,Min().Y()-modelSize));
			lines.push_back(ignition::math::Line2<double>(Max().X()+modelSize,Min().Y()-modelSize, Max().X()+modelSize,Max().Y()+modelSize));
			lines.push_back(ignition::math::Line2<double>(Max().X()+modelSize,Max().Y()+modelSize, Min().X()-modelSize,Max().Y()+modelSize));
			lines.push_back(ignition::math::Line2<double>(Min().X()-modelSize,Max().Y()+modelSize, Min().X()-modelSize,Min().Y()-modelSize));
		}
		return lines;
	}
};
#endif
