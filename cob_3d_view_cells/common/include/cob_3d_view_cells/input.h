#pragma once

#include <vector>

template<typename _ID>
class Connection {
public:
	typedef _ID ID;
	typedef pcl::PointXYZ Point;
	
private:
	ID id_;
	Point pos_;
	
public:
	Connection(const ID &id, const Point &pos):
		id_(id), pos_(pos)
	{}	
	template<typename Feature>
	Connection(const Feature &ft):
		id_(-1), pos_(ft)
	{}
	
	operator ID() const {return id_;}
	operator ID&() {return id_;}
	Connection operator=(const ID &id) {id_=id;return *this;}
	
	const Point &getPos() const {return pos_;}
};

typedef int DefaultID;
typedef Connection<DefaultID> DefaultConnection;
typedef std::vector<DefaultConnection> DefaultContent;
