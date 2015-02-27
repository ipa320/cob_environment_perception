#pragma once

template<typename Feature, typename Distance, typename Content>
std::vector<typename SearchSpace<Feature,Distance,Content>::ContentPtr> SearchSpace<Feature,Distance,Content>::lookup(const Feature &ft)
{		
	std::vector<ContentPtr> ret;
	
	//first search
	Entry e(ft);
	//e.print();
	
	/*{
		std::pair<typename Tree::const_iterator,float> found = this->tree_.find_nearest(e);
		if(found.first != this->tree_.end()) 
			ROS_INFO("next match %f", found.second);
	}*/
	
	std::vector<Entry> result;
	this->tree_.find_within_range(e, dist_thr_*ft.area, std::back_inserter(result));
	for(size_t i=0; i<result.size(); i++)
		if(std::abs(ft.area-result[i].ft_.area)<std::max(ft.area,result[i].ft_.area)*0.1f && e.distance_to2(result[i])<dist_thr_*ft.area) {
			ret.push_back(result[i].content_);
			//ROS_INFO("area %f <-> %f", ft.area, result[i].ft_.area);
		}
	//ROS_INFO("found %d features", (int)ret.size());
    if(ret.size()>0)
		return ret;
		
	//if nothing found -> create new one
	this->tree_.insert( e );
	ret.push_back(e.content_);
	//ROS_INFO("added new feature (tree size %d)", (int)this->tree_.size());
		
	return ret;
}
