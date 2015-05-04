#include <ros.h>
#include <cob_3d_experience_mapping/SensorInfoArray.h>
#include <cob_3d_visualization/simple_marker.h>

struct Bark {
	float r,g,b;
	float x,y,r;
	std::string name;
};

std::vector<Bark> g_barks;
ros::Publisher g_si_pub;

void process(const float x, const float y)
{
	//vis
	cob_3d_visualization::RvizMarkerManager::get().clear();
	SensorInfoArray sia;
	
	for(size_t i=0; i<g_barks.size(); i++)
	{
		if( std::pow(g_barks[i].x-x,2)+std::pow(g_barks[i].y-y,2) <= std::pow(g_barks[i].r,2) )
			sia.infos.push_back(si);
		
		cob_3d_visualization::RvizMarker scene;
		scene.color(1-e,e,0.);
	}
	
	g_si_pub.publish(sia);
	cob_3d_visualization::RvizMarkerManager::get().publish();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "exp_mapping");
	ros::NodeHandle n;
	g_si_pub = n.advertise<cob_3d_experience_mapping::SensorInfoArray>("sensor_info", 10);
	cob_3d_visualization::RvizMarkerManager::get()
		.createTopic("sim_barks")
		.setFrameId("/map");
		//.clearOld();
		
	XmlRpc::XmlRpcValue barks;
	if (private_nh.getParam("barks", barks))
	{
	  std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;
	  for (i = barks.begin(); i != barks.end(); i++)
	  {
		Bark b;

		b.name = i->first;
		
		b.r = i->second["r"];
		b.g = i->second["g"];
		b.b = i->second["b"];
		
		b.x = i->second["x"];
		b.y = i->second["y"];
		b.r = i->second["r"];

		g_barks.push_back(b);
	  }
	}
	else
		ROS_WARN("barks parameter missing");

	tf::TransformListener listener;
	ros::Rate rate(4.0);
	while(ros::ok()) {
		tf::StampedTransform transform;
		try{
			listener.lookupTransform("/odom", "/base_link",  
								   ros::Time(0), transform);
			process(transform.getOrigin().x(), transform.getOrigin().y());
		}
		catch (tf::TransformException ex){
		  ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
		}
		ros::spinOnce();
		
		rate.sleep();
	}
	
	return 0;
}
