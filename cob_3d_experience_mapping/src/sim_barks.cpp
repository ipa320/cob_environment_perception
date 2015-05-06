#include <ros/ros.h>
#include <cob_3d_experience_mapping/SensorInfoArray.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <cob_3d_visualization/simple_marker.h>

struct Bark {
	double cr,cg,cb;
	double x,y,r,ri;
	std::string name;
};

std::vector<Bark> g_barks;
ros::Publisher g_si_pub;

void process(const float x, const float y)
{
	//vis
	cob_3d_visualization::RvizMarkerManager::get().clear();
	cob_3d_experience_mapping::SensorInfoArray sia;
	
	for(size_t i=0; i<g_barks.size(); i++)
	{
		cob_3d_experience_mapping::SensorInfo si;
		si.id  = (int)i;
		
		if( std::pow(g_barks[i].x-x,2)+std::pow(g_barks[i].y-y,2) <= std::pow(g_barks[i].r,2) &&
			std::pow(g_barks[i].x-x,2)+std::pow(g_barks[i].y-y,2) >= std::pow(g_barks[i].ri,2) )
			sia.infos.push_back(si);
		
		cob_3d_visualization::RvizMarker scene;
		scene.circle(g_barks[i].r, Eigen::Vector3f(g_barks[i].x,g_barks[i].y,0), g_barks[i].ri);
		scene.color(g_barks[i].cr,g_barks[i].cg,g_barks[i].cb, 0.2);
	}
	
	g_si_pub.publish(sia);
	cob_3d_visualization::RvizMarkerManager::get().publish();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "sim_barks");
	ros::NodeHandle n("~");
	g_si_pub = n.advertise<cob_3d_experience_mapping::SensorInfoArray>("sensor_info", 10);
	cob_3d_visualization::RvizMarkerManager::get()
		.createTopic("sim_barks")
		.setFrameId("/map");
		//.clearOld();
		
	XmlRpc::XmlRpcValue barks;
	if (n.getParam("barks", barks))
	{
	  for (int i = 0; i<barks.size(); i++)
	  {
		Bark b;

		b.name = (std::string)barks[i]["name"];
			
		b.cr = barks[i]["cr"];
		b.cg = barks[i]["cg"];
		b.cb = barks[i]["cb"];
		
		b.x = barks[i]["x"];
		b.y = barks[i]["y"];
		b.r = barks[i]["r"];
		b.ri = barks[i]["ri"];

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
			listener.lookupTransform("/map", "/base_link", 
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
