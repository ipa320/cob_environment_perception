#pragma once

//includes
#include <cob_3d_visualization/simple_marker.h>

namespace cob_3d_experience_mapping {
	namespace visualization {
	
		template<class TGraph, class TMapCells, class TNode, class TArcIter>
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
			
			void visualize(const TGraph &graph, const TMapCells &cells, const TNode &start_node) {
				TVisitedList visited;
				rec_vis(graph, cells, start_node, visited);
			}
			
		private:
			template<class TMeta> static void visualize_meta(const TNode &act_node, const TMeta &meta, const Eigen::Vector3f &pos) {/*dummy*/}
			
			void visualize_node(const TNode &act_node, const Eigen::Vector3f &pos) {
				{
					cob_3d_visualization::RvizMarker scene;
					scene.sphere(pos);
					scene.color(0.1,1.,0.1);
				}
				
				//TODO: visualize orientation
				//TODO: visualize meta data
				//visualize_meta(act_node, act_node.meta(), pos);
			}
			
			bool rec_vis(const TGraph &graph, const TMapCells &cells, const TNode &act_node, TVisitedList &visted, const int depth=-1, const Eigen::Vector3f &pos=Eigen::Vector3f::Zero()) {
				//only visited each node once
				if(!act_node || depth==0 || visted.find(&act_node)!=visted.end())
					return false;
				cob_3d_visualization::RvizMarkerManager::get().clear();

				visted[&act_node] = true;
				
				visualize_node(act_node, pos);
				
				 //go through all edges
				for(TArcIter ait(act_node->edge_begin(graph)); ait!=act_node->edge_end(graph); ++ait) {
					TNode opposite = cells[act_node->opposite_node(graph, ait)];
					
					Eigen::Vector3f new_pos = pos;// + ;
					if(rec_vis(graph, cells, opposite, visted, depth-1, new_pos)) {
						cob_3d_visualization::RvizMarker scene;
						scene.line(pos, new_pos);
					}
				}

				cob_3d_visualization::RvizMarkerManager::get().publish();
				return true;
			}
		};
	}
}
