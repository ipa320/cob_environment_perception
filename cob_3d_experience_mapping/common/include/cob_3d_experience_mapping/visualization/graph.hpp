#pragma once

//includes
#include <cob_3d_visualization/simple_marker.h>

namespace cob_3d_experience_mapping {
	namespace visualization {
	
		template<class TGraph, class TMapStates, class TMapTransformations, class TNode, class TArcIter>
		struct VisualizationHandler {
			static void init() {
	std::cout<<"init vis"<<std::endl;
				cob_3d_visualization::RvizMarkerManager::get()
					.createTopic("expierience_mapping_marker")
					.setFrameId("/base_link");
					//.clearOld();
			}
			
			VisualizationHandler() {
				init();
			}
			
			~VisualizationHandler() {
			}
			
			typedef std::map<const TNode, Eigen::Affine3f> TVisitedList;
			
			void visualize(const TGraph &graph, const TMapStates &states, TMapTransformations &trans, const TNode &start_node) {
				TVisitedList visited;
				cob_3d_visualization::RvizMarkerManager::get().clear();
				Eigen::Affine3f pos; pos = Eigen::Translation3f(0,0,0);
				rec_vis(graph, states, trans, start_node, visited, start_node->dist_dev(), 20, pos);
				cob_3d_visualization::RvizMarkerManager::get().publish();
			}
			
		private:
			template<class TMeta> static void visualize_meta(const TNode &act_node, const TMeta &meta, const Eigen::Vector3f &pos) {/*dummy*/}
			
			void visualize_node(const TNode &act_node, const Eigen::Affine3f &aff, const double max_dist_dev) {
				if(act_node->dist_dev()/max_dist_dev>30)
					return;
				
				Eigen::Vector3f pos = aff.translation();
				
				//if pose is available
				//pos(0) = act_node->dbg().pose_(0);
				//pos(1) = act_node->dbg().pose_(1);
				
				const Eigen::Vector3f pos_o = (aff*Eigen::Translation3f(0,0,std::min(act_node->dist_dev()/max_dist_dev-1.,3.))).translation();
				const Eigen::Vector3f pos_h = (aff*Eigen::Translation3f(act_node->dist_trv(),0,0)).translation();

				const double e = act_node->dist_trv();
				{
					cob_3d_visualization::RvizMarker scene;
					scene.sphere(pos);
					scene.color(1-e,e,0.);
				}
				{
					cob_3d_visualization::RvizMarker scene;
					scene.line(pos, pos_o,0.03f);
					scene.color(0,0,1,0.5);
				}
				{
					cob_3d_visualization::RvizMarker scene;
					scene.line(pos, pos_h,0.03f);
					scene.color(0,1,0,0.5);
				}
				{
					cob_3d_visualization::RvizMarker scene;
					scene.line(pos_h, pos_o,0.03f);
					scene.color(1,1,1,0.5);
				}
				
				{
					cob_3d_visualization::RvizMarker scene;
					char buf[256];
					sprintf(buf, " d=%.3f h=%.3f o=%.3f", act_node->d(), act_node->dist_trv(), act_node->dist_dev());
					scene.text(act_node->dbg().name_+" "+act_node->dbg().info_+buf);
					scene.move(pos+0.3*Eigen::Vector3f::UnitZ());
				}
				
				//TODO: visualize orientation
				//TODO: visualize meta data
				//visualize_meta(act_node, act_node.meta(), pos);
			}
			
			bool rec_vis(const TGraph &graph, const TMapStates &states, TMapTransformations &trans, const TNode &act_node, TVisitedList &visted, const double max_dist_dev, const int depth=-1, const Eigen::Affine3f &pos=Eigen::Affine3f()) {
				//only visited each node once
				if(!act_node || depth==0)
					return false;
					
				typename TVisitedList::const_iterator it = visted.find(act_node);
				if(it!=visted.end()) {
					cob_3d_visualization::RvizMarker scene;
					scene.line((Eigen::Vector3f)pos.translation(), (Eigen::Vector3f)it->second.translation(), 0.025f);
					scene.color(1,1,1,0.35);
					return false;
				}

				visted[act_node] = pos;
				
				visualize_node(act_node, pos, max_dist_dev);
				
				//go through all edges
				for(TArcIter ait(act_node->template arc_flex_begin<TArcIter>(graph)); ait!=act_node->template arc_flex_end<TArcIter>(graph); ++ait) {
					TNode opposite = states[act_node->opposite_node(graph, ait)];
					
					
					Eigen::Affine3f new_pos = pos*trans[ait]->affine().inverse();
					if(rec_vis(graph, states, trans, opposite, visted, max_dist_dev, depth-1, new_pos)) {
						cob_3d_visualization::RvizMarker scene;
						scene.line((Eigen::Vector3f)pos.translation(), (Eigen::Vector3f)new_pos.translation(), 0.025f);
					scene.color(1,1,1,0.35);
					}
				}

				return true;
			}
		};
	}
}
