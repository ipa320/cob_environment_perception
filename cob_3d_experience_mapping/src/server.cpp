#include <cob_3d_experience_mapping/mapping.h>
#include <cob_3d_experience_mapping/helpers/network.h>

#include "../include/ros_node.hpp"

typedef ROS_Node<As_Node> ns;
//typedef cob_3d_experience_mapping::ClientIdTsGenerator<ns::State, ns::Feature, uint8_t /*client id*/> TClientIdGenerator;
typedef cob_3d_experience_mapping::Context<ns::Scalar /*energy*/, ns::State /*state*/, ns::Feature, Eigen::Matrix<float,1,2>/*energy weight*/, ns::Transformation/*tranformation*/ > TContext;
typedef cob_3d_experience_mapping::IncrementalContextContainer<TContext, ns::TGraph, ns::TMapStates, ns::TMapTransformations, uint8_t /*client id*/> CtxtContainer;

typedef ns::State State;
typedef typename ns::State::TransitionSerialization TransitionSerialization;
typedef typename ns::Feature::FeatureSerialization FeatureSerialization;

HIBERLITE_EXPORT_CLASS(CtxtContainer)
HIBERLITE_EXPORT_CLASS(State)
HIBERLITE_EXPORT_CLASS(TransitionSerialization)
HIBERLITE_EXPORT_CLASS(FeatureSerialization)

int main(int argc, char **argv) {	
    boost::asio::io_service io_service;

    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), 12347);
    boost::asio::ip::tcp::acceptor acceptor(io_service, endpoint);
    	
	typename CtxtContainer::TClientId running_client_id_ = 0;
	TContext ctxt_;	
	ns::TGraph graph_;
	ns::TMapStates states_(graph_);
	ns::TMapTransformations trans_(graph_);
	
	CtxtContainer container(&ctxt_, &graph_, &states_, &trans_, 1);

    for (;;)
    {
      boost::asio::ip::tcp::iostream stream;
      boost::system::error_code ec;
      acceptor.accept(*stream.rdbuf(), ec);
      if (!ec)
      {
		  container.on_client(stream, running_client_id_);
      }
    }
	
	return 0;
}
