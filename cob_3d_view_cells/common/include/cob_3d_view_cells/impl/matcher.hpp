#pragma once

#include "../registration.h"

template<typename Content, typename SimplePoint, typename _ID>
int Matcher<Content,SimplePoint,_ID>::get_id() {
	ROS_INFO("got %d features", (int)num_found_);
	
	static ID last_id = 0;
	
	std::map<typename Content::value_type, float> intersection;
	typename std::map<typename Content::value_type, float>::const_iterator best = intersection.end();
	for(ContentsIteratorConst it2 = contents_.begin(); it2!=contents_.end(); it2++)
		for(typename std::vector<ContentPtr>::const_iterator it = it2->cnt_.begin(); it!=it2->cnt_.end(); it++)
			for(typename Content::const_iterator jt = (*it)->begin(); jt!=(*it)->end(); jt++) {
				float &v = intersection[*jt];
				v += 1.f/it2->cnt_.size();
				if( (best==intersection.end() || v>best->second) && intersection.find(*jt)->first<last_id-100)
					best = intersection.find(*jt);
			}
		
	//debug
	/*for(typename std::map<typename Content::value_type, float>::const_iterator it = intersection.begin(); it!=intersection.end(); it++)
			ROS_INFO("node %d -> %f", (int)it->first, it->second/(float)num_found_);
	ROS_INFO("%d %d", (int)num_found_, (int)contents_.size());
	if(best->second/(float)num_found_ > 1) while(1);*/
			
	static int stat_num=0, stat_found=0;
	++stat_num;
	
	if(best!=intersection.end() && (float)best->second>num_found_*int_thr_ && num_found_>20) {
		ROS_INFO("found match %d with %f", (int)best->first, best->second/(float)num_found_);
		//update id & validate with registration
		VRegistration reg;
		
		for(ContentsIteratorConst it2 = contents_.begin(); it2!=contents_.end(); it2++) {
			for(typename std::vector<ContentPtr>::const_iterator it = it2->cnt_.begin(); it!=it2->cnt_.end(); it++) {
				bool found = false;
				for(typename Content::const_iterator jt = (*it)->begin(); jt!=(*it)->end(); jt++)
					if(*jt==best->first) {
						
						//look for all matching keypoints... (TODO: improve)
						for(size_t i=0; keypoints_ && i<keypoints_->size(); i++)
							if( ((*keypoints_)[i].getVector3fMap()-it2->cur_.getPos().getVector3fMap()).squaredNorm() < std::pow(0.75f,2) &&
								std::abs((*keypoints_)[i].getVector3fMap()(2)-it2->cur_.getPos().getVector3fMap()(2))<0.2f )
								reg.add((*keypoints_)[i], jt->getPos());
						found=true;
						break;
					}
				if(!found) {
//					for(size_t i=0; i<(*it)->size(); i++)
//						ROS_INFO("has %d", (int)(**it)[i]);
//?					(*it)->push_back(best->first);
//					ROS_INFO("added ft: node %d -> %d", (int)(*it)->back(), (int)best->first);
				}
			}
		}
		bool result = true, tt=false;
		//reg.computeTF(&result);
		
		ROS_INFO("result: %d %d", (int)result, (int)tt);
		if(result) {
			++stat_found;
			return best->first;
		}
	}
	else if(best!=intersection.end())
		ROS_INFO("best match %d with %f", (int)best->first, best->second/(float)num_found_);
		
	//otherwise create new id
	const ID id = create_new_id<ID>();
	//if(id<300)
	{
		for(ContentsIterator it2 = contents_.begin(); it2!=contents_.end(); it2++)
			for(typename std::vector<ContentPtr>::iterator it = it2->cnt_.begin(); it!=it2->cnt_.end(); it++) {
				bool found = false;
				for(typename Content::const_iterator jt = (*it)->begin(); jt!=(*it)->end(); jt++)
					if(*jt==id) {
						found=true;
						break;
					}
				if(!found) (*it)->push_back(it2->cur_ = id);
			}
	}
		
	ROS_INFO("no match -> new node %d  (%d/%d)", id, stat_found, stat_num);
	return last_id = id;
}
