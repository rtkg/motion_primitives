/**
 * @author Robert Krug
 * @date   Fri, Oct 9, 2012
 *
 */


#ifndef canonical_system_h___
#define canonical_system_h___

#include "ros/ros.h"

namespace PrimitiveControllers
{
  /**
   *@brief ...
   */
  class CanonicalSystem
  {

  public:

    CanonicalSystem();
    CanonicalSystem(double Td);
    ~CanonicalSystem();

    void reset();
    void setTau(double Tau);
    double getPhaseVariable();
    void update(double Td);
    bool isMovementFinished();

  private:

    double s_;
    double Tau_;
    bool mvmt_finished_;
  };
}//end namespace
#endif
