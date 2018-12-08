#include "BoxExpanded.hh"
	
std::string BoxExpanded::GetSDF()
{
	std::ostringstream stream;
	stream << "<sdf version ='1.6'>\
		<model name ="<<name<<">\
		<pose>"<<Center()<<" 0 0 0</pose>\
		<link name ='link'>\
		<pose>0 0 .5 0 0 0</pose>\
		<collision name ='collision'>\
			<geometry>\
				<box><size>"<<Size()<<"</size></box>\
			</geometry>\
		</collision>\
		<visual name ='visual'>\
			<geometry>\
				<box><size>"<<Size()<<"</size></box>\
			</geometry>\
		</visual>\
		</link>\
		</model>\
	</sdf>";
return stream.str();
}




