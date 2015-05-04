#pragma once

//includes
#include <cob_3d_visualization/simple_marker.h>

namespace cob_3d_experience_mapping {
	namespace visualization {
	
		template<class TGraph, class TMapCells, class TMapTransformations, class TNode, class TArcIter>
		struct VisualizationHandler {
			static void init() {
				cob_3d_visualization::RvizMarkerManager::get()
					.createTopic("expierience_mapping_marker")
					.setFrameId("/map");
					//.clearOld();
			}
			
			VisualizationHandler() {
				init();
			}
			
			~VisualizationHandler() {
			}
			
			typedef std::map<const TNode*, bool> TVisitedList;
			
			void visualize(const TGraph &graph, const TMapCells &cells, TMapTransformations &trans, const TNode &start_node) {
				TVisitedList visited;
				cob_3d_visualization::RvizMarkerManager::get().clear();
				rec_vis(graph, cells, trans, start_node, visited);
				cob_3d_visualization::RvizMarkerManager::get().publish();
			}
			
		private:
			template<class TMeta> static void visualize_meta(const TNode &act_node, const TMeta &meta, const Eigen::Vector3f &pos) {/*dummy*/}
			
			void visualize_node(const TNode &act_node, const Eigen::Affine3f &aff) {
				ROS_INFO("visualize_node");
				
				const Eigen::Vector3f pos = aff.translation();

				const double e = act_node->dist_h()*2;
				{
					cob_3d_visualization::RvizMarker scene;
					scene.sphere(pos);
					scene.color(1-e,e,0.);
				}
				
				{
					cob_3d_visualization::RvizMarker scene;
					char buf[128];
					sprintf(buf, "%s %s e=%.3f", act_node->dbg().name_.c_str(), act_node->dbg().info_.c_str(), e);
					scene.text(buf);
					scene.move(pos+0.3*Eigen::Vector3f::UnitZ());
				}
				
				//TODO: visualize orientation
				//TODO: visualize meta data
				//visualize_meta(act_node, act_node.meta(), pos);
			}
			
			bool rec_vis(const TGraph &graph, const TMapCells &cells, TMapTransformations &trans, const TNode &act_node, TVisitedList &visted, const int depth=-1, const Eigen::Affine3f &pos=Eigen::Affine3f()) {
				//only visited each node once
				if(!act_node || depth==0 || visted.find(&act_node)!=visted.end()) {
					ROS_INFO("skip vis.");
					return false;
				}

				visted[&act_node] = true;
				
				visualize_node(act_node, pos);
				
				//go through all edges
				for(TArcIter ait(act_node->edge_begin(graph)); ait!=act_node->edge_end(graph); ++ait) {
					TNode opposite = cells[act_node->opposite_node(graph, ait)];
					
					
					Eigen::Affine3f new_pos = pos*trans[ait]->affine();
					if(rec_vis(graph, cells, trans, opposite, visted, depth-1, new_pos)) {
						cob_3d_visualization::RvizMarker scene;
						scene.line((Eigen::Vector3f)pos.translation(), (Eigen::Vector3f)new_pos.translation());
					}
				}

				return true;
			}
		};
	}
}
