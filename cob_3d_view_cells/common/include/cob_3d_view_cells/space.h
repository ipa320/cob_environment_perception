#pragma once

#include <string>
#include <boost/shared_ptr.hpp>
#include <kdtree++/kdtree.hpp>

template<typename Feature, typename Distance=float, typename Content=DefaultContent>
class SearchSpace {
public:
	typedef boost::shared_ptr<Content> ContentPtr;
	typedef Content TContent;
	
private:
	
	struct Entry {
		enum {DIMENSION=Feature::DIMENSION};
		
		Entry(const Feature &ft) : ft_(ft), content_(new Content)
		{}
		
		Feature ft_;
		ContentPtr content_;
	  
		void print() const {
			for(size_t i=0; i<32; i++)
				std::cout<<ft_.feature[i]<<" ";
			std::cout<<std::endl;
		}
		
		float distance_to2(const Entry &o) {
			float dist=0;
			for(size_t i=0; i<DIMENSION; i++)
				dist += std::abs((*this)[i]-o[i]);
			return dist;
		}
	  
		inline float operator[](size_t const N) const { return ft_.feature[N]; }
		typedef float value_type;
	};
	
	typedef KDTree::KDTree<Entry::DIMENSION, Entry> Tree;
	
	Tree tree_;
	Distance dist_thr_;
	
public:

	std::vector<ContentPtr> lookup(const Feature &ft);
	void finish() {this->tree_.optimise();}

	//IO
	bool load(const std::string &fn) {throw "to implememented";return false;}
	bool save(const std::string &fn) {throw "to implememented";return false;}
	
	//configuration
	void setDistThreshold(const Distance &dist) {dist_thr_ = dist;}
};

#include "impl/space.hpp"
