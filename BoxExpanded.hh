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
	  		lines.push_back(ignition::math::Line2<double>(Min().X(),Min().Y(), Max().X(),Min().Y()));
	  		lines.push_back(ignition::math::Line2<double>(Max().X(),Min().Y(), Max().X(),Max().Y()));
	  		lines.push_back(ignition::math::Line2<double>(Max().X(),Max().Y(), Min().X(),Max().Y()));
	  		lines.push_back(ignition::math::Line2<double>(Min().X(),Max().Y(), Min().X(),Min().Y()));
	  }
	  std::string GetSDF();
	  std::vector<ignition::math::Line2<double>> getLines(){
		  return lines;
	  }
  };
#endif
