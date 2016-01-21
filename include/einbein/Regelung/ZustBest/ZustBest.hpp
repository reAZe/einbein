#ifndef EINBEIN_ZUSTBEST_HPP
#define EINBEIN_ZUSTBEST_HPP

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/math/Matrix.hpp>


using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


namespace einbein{
  class ZustBest : public eeros::control::Block{
  public:
    ZustBest();
    virtual ~ZustBest();
    
    //define inputs
    virtual eeros::control::Input<Vector3>& getIn_enc(){return in_enc;}
    virtual eeros::control::Input<Vector3>& getIn_denc(){return in_denc;}
    
    //define outputs
    virtual eeros::control::Output<int>& getOut_Zustand(){return out_Zustand;}
    
    
    
  protected:
    //define inputs

    eeros::control::Input<Vector3> in_enc;
    eeros::control::Input<Vector3> in_denc;
    
    //define outputs
    eeros::control::Output<int> out_Zustand;
    
    
  private:
    virtual void run();
    
    double  dz_dif;
    Vector3 enc, denc;
    
    int Zustand;
    
    double Ts; 
    
    
  };// end class ZustBest
}// end namespace ZustBest


#endif // EINBEIN_ZUSTBEST_HPP
