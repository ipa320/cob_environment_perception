

template<typename OBJECT_CONTEXT>
bool Node<OBJECT_CONTEXT>::registration(const OBJCTXT &ctxt, DOF6 &tf, typename DOF6::TYPE &probability_success_rate, typename DOF6::TYPE &probability_error_rate)
{
  //1. find correspondences
  //2. register
  return ctxt_.registration(ctxt, tf, probability_success_rate, probability_error_rate);
}


template<typename OBJECT_CONTEXT>
bool Node<OBJECT_CONTEXT>::addCtxt(const OBJCTXT &ctxt, const DOF6 &tf)
{
  //1. find correspondences
  //2. register
  return ctxt_.add(ctxt, tf);
}
