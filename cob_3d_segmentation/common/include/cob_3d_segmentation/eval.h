/*
 * eval.h
 *
 *  Created on: 15.01.2013
 *      Author: josh
 */

#ifndef EVAL_H_
#define EVAL_H_

class BinaryClassification {

public:

  struct Data {
    size_t num;
    size_t true_pos;
    size_t false_pos;
    int label, mark;

    Data():num(0), true_pos(0), false_pos(0), label(-1), mark(-1) {}
  };

private:

  std::map<int, std::map<int, size_t> > label_to_mark;
  std::map<int, Data> stat_label;

  std::map<int, std::map<int, size_t> > mark_to_label;
  std::map<int, Data> stat_mark;

  bool isMarkMaxForLabel(int mark, int l) {
    const size_t g = mark_to_label[mark][l];
    size_t m = g;
    for(std::map<int, size_t>::const_iterator it2 = mark_to_label[mark].begin(); it2!=mark_to_label[mark].end(); it2++)
      m = std::max(m, it2->second);
    return m==g;
  }

public:

  template<typename PointLabel>
  void addPC(const boost::shared_ptr<const pcl::PointCloud<PointLabel> > &labeled_pc) {
    if(!labeled_pc) return;

    for(size_t x=0; x<labeled_pc->size(); x++)
      if(pcl_isfinite((*labeled_pc)[x].z)) addLabelPoint( (*labeled_pc)[x].label );
  }

  void addLabelPoint(const int l) {
    stat_label[l].num++;
    stat_label[l].label = l;
  }

  void addMark(const int mark, const int l) {
    if(label_to_mark[l].find(mark)==label_to_mark[l].end())
      label_to_mark[l][mark]=0;
    else
      label_to_mark[l][mark]++;

    stat_mark[mark].num++;
    stat_mark[mark].label = mark;
    if(mark_to_label[mark].find(l)==mark_to_label[mark].end())
      mark_to_label[mark][l]=0;
    else
      mark_to_label[mark][l]++;
  }

  const BinaryClassification &finish() {
    for(std::map<int, Data>::iterator it = stat_label.begin(); it!=stat_label.end(); it++) {
      const int l = it->first;
      ROS_ASSERT(l==it->second.label);

      for(std::map<int, size_t>::const_iterator it2 = label_to_mark[l].begin(); it2!=label_to_mark[l].end(); it2++) {
        if(it2->second > it->second.true_pos &&
            isMarkMaxForLabel(it2->second, l)
            ) {
          it->second.false_pos += it->second.true_pos;
          it->second.true_pos = it2->second;
          it->second.mark = it2->first;
        }
        else
          it->second.false_pos += it2->second;
      }
    }

    std::cout<<*this;

    return *this;
  }

  void get_rate(double &true_positive, double &false_positive) const {
    true_positive = false_positive = 0;
    size_t num = 0;

    for(std::map<int, Data>::const_iterator it = stat_label.begin(); it!=stat_label.end(); it++) {
      true_positive += it->second.true_pos;
      false_positive += it->second.false_pos;
      num += std::max(it->second.num, it->second.true_pos+it->second.false_pos);
    }

    true_positive/=num;
    false_positive/=num;
  }

  ///for debugging
  friend std::ostream &operator<<(std::ostream &os, const BinaryClassification &bc);
};

std::ostream &operator<<(std::ostream &os, const BinaryClassification &bc) {
  os<<"Num Labels: "<<bc.stat_label.size()<<"\n";

  for(std::map<int, BinaryClassification::Data>::const_iterator it = bc.stat_label.begin(); it!=bc.stat_label.end(); it++) {
    const int l = it->first;
    os<<"Label: "<<l<<" "<<it->second.num<<"\n";

    if(bc.label_to_mark.find(l)==bc.label_to_mark.end()) continue;

    for(std::map<int, size_t>::const_iterator it2 = bc.label_to_mark.find(l)->second.begin(); it2!=bc.label_to_mark.find(l)->second.end(); it2++)
      os<<"Mark: "<<it2->first<<" "<<it2->second<<"\n";
  }

  return os;
}

class RunningStat
{
public:
  RunningStat() : m_n(0),m_oldM(0),m_newM(0),m_oldS(0),m_newS(0) {}

  void Clear()
  {
    m_n = 0;
  }

  void Push(double x)
  {
    m_n++;

    // See Knuth TAOCP vol 2, 3rd edition, page 232
    if (m_n == 1)
    {
      m_oldM = m_newM = x;
      m_oldS = 0.0;
    }
    else
    {
      m_newM = m_oldM + (x - m_oldM)/m_n;
      m_newS = m_oldS + (x - m_oldM)*(x - m_newM);

      // set up for next iteration
      m_oldM = m_newM;
      m_oldS = m_newS;
    }
  }

  void Push(double x, double w)
  {
    // See Knuth TAOCP vol 2, 3rd edition, page 232
    if (m_n == 0)
    {
      m_n+=w;
      m_oldM = m_newM = w*x;
      m_oldS = 0.0;
    }
    else
    {
      m_n+=w;
      m_newM = m_oldM + (w*x - m_oldM)/m_n;
      m_newS = m_oldS + (w*x - m_oldM)*(w*x - m_newM);

      // set up for next iteration
      m_oldM = m_newM;
      m_oldS = m_newS;
    }
  }

  int NumDataValues() const
  {
    return m_n;
  }

  double Mean() const
  {
    return (m_n > 0) ? m_newM : 0.0;
  }

  double Variance() const
  {
    return ( (m_n > 1) ? m_newS/(m_n - 1) : 0.0 );
  }

  double StandardDeviation() const
  {
    return sqrt( Variance() );
  }

private:
  double m_n;
  double m_oldM, m_newM, m_oldS, m_newS;
};



#endif /* EVAL_H_ */
