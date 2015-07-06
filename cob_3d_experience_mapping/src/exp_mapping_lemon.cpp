#include <cob_3d_experience_mapping/mapping.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <cob_3d_experience_mapping/visualization/graph.hpp>

int main() {
	typedef cob_3d_experience_mapping::State<cob_3d_experience_mapping::Empty, float > State;
	typedef cob_3d_experience_mapping::Transformation<float, 2, 1, typename State::TPtr> Transformation;
	cob_3d_experience_mapping::Context<float /*energy*/, State /*state*/, Eigen::Matrix<float,1,2>/*energy weight*/> ctxt;
	
	lemon::ListDigraph graph;
	lemon::ListDigraph::NodeMap<typename State::TPtr> cells(graph);
	lemon::ListDigraph::ArcMap <typename Transformation::TPtr> trans(graph);
	
	cob_3d_experience_mapping::algorithms::step(graph, ctxt, cells, trans);
	cob_3d_experience_mapping::visualization::VisualizationHandler<typename State::TGraph, lemon::ListDigraph::NodeMap<typename State::TPtr>, typename State::TPtr, typename State::TArcIterator> vis;
	
	vis.init();
	vis.visualize(graph, cells, typename State::TPtr());
	
	return 0;
}
