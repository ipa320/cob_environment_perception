#pragma once

#inlcude <string>
#include <vector>

template<typename Feature, typename Distance=float, typename Content=std::vector<int> >
class SearchSpace {
	Distance dist_thr_;
public:

	void lookup(const Feature &ft, std::vector<Content> &contents);

	//IO
	bool load(const std::string &fn);
	bool save(const std::string &fn);

	//working methods
	void add(const Feature &ft);
	
	//configuration
	void setDistThreshold(const Distance &dist) {dist_thr_ = dist;}
};
