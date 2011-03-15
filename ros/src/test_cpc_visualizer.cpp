#include <pcl_visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <cob_vision_ipa_utils/cpc_point.h>

typedef CPCPoint Point;

int 
  main (int argc, char **argv)
{
	pcl::PointCloud<Point> cloud;

	pcl::PCDReader pcd;
	if (!pcd.read (argv[1], cloud))
	return (-1);

	pcl_visualization::PCLVisualizer p ("test");
	p.setBackgroundColor (1, 1, 1);

	// Handler random color demo

    std::cerr << "PointCloudColorHandlerRGBField demo." << std::endl;
    pcl_visualization::PointCloudColorHandlerRGBField<Point> handler (cloud);

	p.addPointCloud (cloud, handler, "cloud_rgb");

	for (unsigned int i = 0; i<cloud.points.size(); i++)
	{
		if(cloud.points[i].isFeature)
		{
			std::string name;
			std::stringstream out;
			out << "feature" << i;
			name = out.str();
			p.addSphere(cloud.points[i], 0.01, 1, 0, 0, name);
		}
	}

	p.spin();
	p.removePointCloud ("cloud_rgb");

}
