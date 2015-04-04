#pragma once

#include <std_msgs/ColorRGBA.h>
#include <dynamic_reconfigure/server.h>
#include <cob_3d_experience_mapping/SettingsConfig.h>
#include <cob_3d_experience_mapping/Vis_SettingsConfig.h>

namespace cob_3d_experience_mapping {
	
	template<class TEnergyFactor, class TDist>
	struct Parameter {
		//typedef float TScalar;
		
		TEnergyFactor algo1_energy_weight_;
		TEnergyFactor algo2_inhibition_constant_;
		
		TDist prox_thr_;

		//visualization
		std_msgs::ColorRGBA vis_color_cell_;

		dynamic_reconfigure::Server<cob_3d_experience_mapping::SettingsConfig> server_settings_;
		dynamic_reconfigure::Server<cob_3d_experience_mapping::Vis_SettingsConfig> server_vis_settings_;

		Parameter() {
			//TODO: read from parameter server

			server_settings_.setCallback(boost::bind(&Parameter<TEnergyFactor, TDist>::cb_settings, this, _1, _2));
			server_vis_settings_.setCallback(boost::bind(&Parameter<TEnergyFactor, TDist>::cb_vis_settings, this, _1, _2));
		}

		void cb_settings(cob_3d_experience_mapping::SettingsConfig &config, uint32_t level) {
			//algo1_energy_weight_ = config.algo1_energy_weight;
			//algo2_inhibition_constant_ = config.algo2_inhibition_constant;
			prox_thr_(0) = config.translation;
			prox_thr_(1) = config.rotation;

			ROS_INFO("updated settings");
		}

		void cb_vis_settings(cob_3d_experience_mapping::Vis_SettingsConfig &config, uint32_t level) {
			vis_color_cell_.r = config.groups.cell_color.r;
			vis_color_cell_.g = config.groups.cell_color.g;
			vis_color_cell_.b = config.groups.cell_color.b;
			vis_color_cell_.a = config.groups.cell_color.a;

			ROS_INFO("updated vis. settings");
		}
		
	};
	
}
