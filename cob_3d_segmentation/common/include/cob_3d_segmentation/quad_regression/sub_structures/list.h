/*
 * list.h
 *
 *  Created on: 11.06.2012
 *      Author: josh
 */

#ifndef LIST_H_
#define LIST_H_




/**
 * contains x,y coordinate alias index (v)
 * hops gives number of possible uses (rest)
 */
struct SVALUE
{
  unsigned int v,hops;

  SVALUE(){}

  SVALUE(const unsigned int v, const unsigned int hops)
  :v(v), hops(hops)
  {}

};

/**
 * fast visited list (pre-allocate values)
 */
template<typename VALUE>
struct VISITED_LIST {
  int size;
  int pos;
  std::vector<VALUE> vals;

  VISITED_LIST():size(0),pos(-1), vals(1000) {
  }

  inline void init()
  {
    size=(0);
    pos=(-1);
  }

  inline void add(const VALUE &v) {
    if((size_t)size>=vals.size())
      vals.push_back(v);
    else
      vals[size]=v;
    ++size;
  }

  inline void move() {++pos;}

  inline void remove() {
    --size;
    if(pos<size) vals[pos]=vals[size];
    --pos;
  }

  inline void replace(const VALUE &v) {
    vals[pos]=v;
  }
};


/**
 * used as parameter for outline calculation
 * point with additional information if point was in front of other object
 */
struct SXY {
  int x,y;
  bool back;
};

/**
 * sort 2D points from left to right, top to down
 */
struct SXYcmp {
  inline bool operator() (const SXY& lhs, const SXY& rhs) const
  {if(lhs.y==rhs.y)
    return lhs.x<rhs.x;
  return lhs.y<rhs.y;}
};

#endif /* LIST_H_ */
