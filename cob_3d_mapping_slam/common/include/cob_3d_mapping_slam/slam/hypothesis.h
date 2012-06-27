/*
 * hypothesis.h
 *
 *  Created on: 29.05.2012
 *      Author: josh
 */

#ifndef HYPOTHESIS_H_
#define HYPOTHESIS_H_


namespace Slam  /// namespace for all Slam related stuff
{

  template<typename NODE>
  class Hypothesis
  {
    typedef typename NODE::OBJCTXT OBJCTXT;
    typedef typename OBJCTXT::DOF6 DOF6;

    struct SHYPO
    {
      typename DOF6::TYPE error_rate_, success_rate_;
      std::vector<SWAY<NODE> > cons_;
    };

    typedef std::map<typename NODE::Ptr,SHYPO> HIST;

    HIST history_;
  public:

    typename DOF6::TYPE getOverallScore() const {
      typename DOF6::TYPE mx=0;

      for(typename HIST::const_iterator it=history_.begin(); it!=history_.end(); ++it)
      {
        mx = std::max(mx,it->error_rate_);
      }

      return mx;
    }
  };

}


#endif /* HYPOTHESIS_H_ */
