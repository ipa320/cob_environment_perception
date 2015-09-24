#pragma once

#include <std_msgs/ColorRGBA.h>
#include <dynamic_reconfigure/server.h>
#include <cob_3d_experience_mapping/SettingsConfig.h>
#include <cob_3d_experience_mapping/Vis_SettingsConfig.h>

//! interfaces and implementations of cob_3d_experience_mapping
namespace cob_3d_experience_mapping {
	
	/*! \class Parameter
		\brief Container for parameter storage, serialization

		Templated class which contains all parameters. It allows serialization from (and to):
		 - ROS parameter server
		 - ROS dynamic reconfiguration
		 - Boost serialization
	*/
	template<class TEnergyFactor, class TDist>
	struct Parameter {		
		TDist 					prox_thr_;	//!< transformation dependent normalization factor(s): normalized_action = N(action)
		typename TDist::Scalar 	deviation_factor_;	//!< deviation factor of normalized action if not present by input
		
		int est_occ_;			//!< number of estimated minimal occurances of a feature
		int min_age_;			//!< minmal age before a feature can be "seen" and influence a state (in number of newly created states)
		int max_active_states_;	//!< size active state list

#ifdef CLOUD_
		std::string cloud_addr_;		//!< address of server for cloud synchronsization, e.g. localhost:12345
		float cloud_sync_interval_;		//!< synchronization interval in seconds to update map data with server
#endif

#ifdef VIS_
		//visualization
		std_msgs::ColorRGBA vis_color_state_;	//!< visualization: color of a state (Marker)

		//dynamic_reconfigure::Server<cob_3d_experience_mapping::SettingsConfig> server_settings_;
		//dynamic_reconfigure::Server<cob_3d_experience_mapping::Vis_SettingsConfig> server_vis_settings_;
#endif

		Parameter() {
			prox_thr_(0) = 0.5;
			prox_thr_(1) = 0.35;
			//energy_max_ = 3;//1.25;
			est_occ_ = 5;
			min_age_ = 3;
			max_active_states_ = 200;
			deviation_factor_ = 0.1;
			
			prox_thr_(0) = 0.5;
			prox_thr_(1) = 0.5;
			//energy_max_ = 1;//1.25;
			est_occ_ = 1;
			min_age_ = 3;
			max_active_states_ = 100;
			deviation_factor_ = 0.075;
			
#ifdef CLOUD_
			cloud_addr_ = "";
			cloud_sync_interval_ = 10.f;
#endif

#ifdef VIS_
			vis_color_state_.r = 0.5;
			vis_color_state_.g = 0.5;
			vis_color_state_.b = 0.5;
			vis_color_state_.a = 0.5;
#endif

			//TODO: read from parameter server
#ifdef VIS_
			//server_settings_.setCallback(boost::bind(&Parameter<TEnergyFactor, TDist>::cb_settings, this, _1, _2));
			//server_vis_settings_.setCallback(boost::bind(&Parameter<TEnergyFactor, TDist>::cb_vis_settings, this, _1, _2));
#endif
		}
		
		//!< (de)serialization function for boost/hiberlite
		UNIVERSAL_SERIALIZE()
		{
		    assert(version==CURRENT_SERIALIZATION_VERSION);
		    
		    for(int i=0; i<prox_thr_.rows(); i++)
				for(int j=0; j<prox_thr_.cols(); j++) {
					char buf[16];
					sprintf(buf, "prox_thr_%d_%d", i,j);
					ar & UNIVERSAL_SERIALIZATION_NVP_NAMED(buf, prox_thr_(i,j));
				}
		    ar & UNIVERSAL_SERIALIZATION_NVP(est_occ_);
		    ar & UNIVERSAL_SERIALIZATION_NVP(min_age_);
		    ar & UNIVERSAL_SERIALIZATION_NVP(max_active_states_);
		    ar & UNIVERSAL_SERIALIZATION_NVP(deviation_factor_);
		}

#ifdef VIS_
		//!< deserialization function for dynamic reocnfigure (settings needed for algo.)
		void cb_settings(cob_3d_experience_mapping::SettingsConfig &config, uint32_t level) {
			prox_thr_(0) = config.translation;
			prox_thr_(1) = config.rotation;
			est_occ_ = config.est_occ;
			min_age_ = config.min_age;
			max_active_states_ = config.max_active_states;
			deviation_factor_ = config.deviation_factor;
		}

		//!< deserialization function for dynamic reocnfigure (optional visualization configuration)
		void cb_vis_settings(cob_3d_experience_mapping::Vis_SettingsConfig &config, uint32_t level) {
			vis_color_state_.r = config.groups.state_color.r;
			vis_color_state_.g = config.groups.state_color.g;
			vis_color_state_.b = config.groups.state_color.b;
			vis_color_state_.a = config.groups.state_color.a;
		}
#endif
		
	};
	
}
