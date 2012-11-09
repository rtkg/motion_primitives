/**
 * @author Robert Krug
 * @date   Fri, Oct 9, 2012
 *
 */

#include "primitive_controllers/canonical_system.h"

namespace PrimitiveControllers
{
  //-----------------------------------------------------------------------------------------------
  CanonicalSystem::CanonicalSystem() : s_(0.0), Tau_(1.0), mvmt_finished_(false) {}
  //-----------------------------------------------------------------------------------------------
  CanonicalSystem::~CanonicalSystem() {}
  //-----------------------------------------------------------------------------------------------
  void  CanonicalSystem::reset()
  {
    s_=0.0;
    mvmt_finished_=false;
  }
  //-----------------------------------------------------------------------------------------------
  void  CanonicalSystem::setTau(double Tau)
  {
    Tau_=Tau;
  }
  //-----------------------------------------------------------------------------------------------
  double CanonicalSystem::getPhaseVariable()
  {
    return s_;
  }
  //-----------------------------------------------------------------------------------------------
  void CanonicalSystem::update(double Td)
  {
    s_=s_+Td/Tau_;

    if (s_ >= 1.0)
      mvmt_finished_=true;
  }
  //-----------------------------------------------------------------------------------------------
  bool CanonicalSystem::isMovementFinished()
  {
    return mvmt_finished_;
  }
  //-----------------------------------------------------------------------------------------------
}//end namespace
