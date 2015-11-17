#pragma once

#include <vector>

template<typename Content, typename SimplePoint, typename _ID=DefaultID>
class Matcher {
public:
	typedef boost::shared_ptr<Content> ContentPtr;
	typedef typename Content::value_type Connection;
	typedef _ID ID;
	typedef typename pcl::PointCloud<SimplePoint> PointCloud;
	
private:
	struct InputContent {
		std::vector<ContentPtr> cnt_;
		Connection cur_;
		
		InputContent(const std::vector<ContentPtr> &cnt, const Connection &cur):
			cnt_(cnt), cur_(cur)
		{}
		
		bool operator==(const InputContent &o) const {
			return cnt_.get()==o.cnt_.get();
		}
	};
	std::vector<InputContent> contents_;
	size_t num_found_;
	float int_thr_;
	typename PointCloud::ConstPtr keypoints_;
	
	typedef typename std::vector<InputContent>::iterator ContentsIterator;
	typedef typename std::vector<InputContent>::const_iterator ContentsIteratorConst;
	
public:
	Matcher(): num_found_(0) {}
	
	void setKeypoints(const typename PointCloud::ConstPtr &kps) {keypoints_ = kps;}
	
	void push_back(const std::vector<ContentPtr> &cnt, const Connection &current_view) {
		InputContent inp(cnt,current_view);
		/*for(size_t i=0; i<contents_.size(); i++)
			if(inp==contents_[i]) {
				ROS_INFO("already in there");
				return;
			}*/
		size_t n=0;
		for(size_t i=0; i<cnt.size(); i++) n+=cnt[i]->size();
		//ROS_INFO("added %d/%d contents",(int)n,(int)cnt.size());
		num_found_ += cnt.size();
		contents_.push_back(inp);
	}
	
	int get_id();
	
	void setIntersectionThreshold(const float t) {int_thr_=t;}
};

#include "impl/matcher.hpp"
