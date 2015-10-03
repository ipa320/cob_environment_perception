#include <cob_3d_experience_mapping/mapping.h>

template <int NUM_TRANS=2, int NUM_ROT=1, typename Scalar=float>
class ExperienceMappingInterface
{
public:
	typedef lemon::ListDigraph TGraph;
	typedef cob_3d_experience_mapping::State<cob_3d_experience_mapping::Empty, Scalar, TGraph> State;
	typedef cob_3d_experience_mapping::Feature<State, cob_3d_experience_mapping::Empty> Feature;
	typedef cob_3d_experience_mapping::Transformation<Scalar, NUM_TRANS, NUM_ROT, typename State::TPtr> Transformation;
	
private:

	cob_3d_experience_mapping::Context<Scalar /*energy*/, State /*state*/, Feature, Eigen::Matrix<float,1,2>/*energy weight*/, Transformation/*tranformation*/> ctxt_;
	
	TGraph graph_;
	TGraph::NodeMap<typename State::TPtr> cells_;
	TGraph::ArcMap <typename Transformation::TPtr> trans_;
	
	double time_last_odom_;
	
public:
  // Constructor
  ExperienceMappingInterface():
	cells_(graph_),
	trans_(graph_),
	time_last_odom_(-1)
  {
  }

  virtual ~ExperienceMappingInterface()
  {}

  void onInit() {
	  cob_3d_experience_mapping::algorithms::init<Transformation>(graph_, ctxt_, cells_, trans_);
  }
  
  void on_sensor_info(const std::vector<int> &infos) {
	  static int ts=0;
	  ++ts;
	  
	   cob_3d_experience_mapping::algorithms::reset_features(ctxt_.active_cells());
	  
	  for(size_t i=0; i<infos.size(); i++) {
		  ctxt_.add_feature(infos[i], ts);
	  }
  }
  
  void on_odom(const double stamp, const typename Transformation::TLink &link) {
	  if(time_last_odom_>=0 && (stamp-time_last_odom_)<10) {		  
		  Eigen::Vector3f dbg_pose;
		  Transformation action(link*(stamp-time_last_odom_), ctxt_.current_active_cell());
		  action.deviation() = 0.0025f;
		  Transformation action_derv(link, ctxt_.current_active_cell());
		  action_derv.deviation() = 0.0025f;
		  
		  cob_3d_experience_mapping::algorithms::step(graph_, ctxt_, cells_, trans_, action, action_derv, dbg_pose);
	  }
	  time_last_odom_ = stamp;
  }
  
};

class ExperienceMappingInterface<>;
