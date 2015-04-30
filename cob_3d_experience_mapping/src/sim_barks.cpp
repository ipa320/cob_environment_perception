#include <ros.h>
#include <cob_3d_experience_mapping/SensorInfoArray.h>
#include <cob_3d_visualization/simple_marker.h>

{
	//vis
	cob_3d_visualization::RvizMarkerManager::get().clear();
	SensorInfoArray sia;
	
	{
		sia.infos.push_back(si);
		
		cob_3d_visualization::RvizMarker scene;
		scene.color(1-e,e,0.);
	}
	
	publish(sia);
	cob_3d_visualization::RvizMarkerManager::get().publish();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "exp_mapping");
	cob_3d_visualization::RvizMarkerManager::get()
		.createTopic("sim_barks")
		.setFrameId("/map");
		//.clearOld();

	ros::spin();
	
	return 0;
}
