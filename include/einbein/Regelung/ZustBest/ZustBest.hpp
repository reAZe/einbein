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
    ZustBest(double ts);
    virtual ~ZustBest();
    
    //define inputs
    virtual eeros::control::Input<double>& getIn_ddzIMU(){return in_ddzIMU;}
    virtual eeros::control::Input<double>& getIn_dzIMU(){return in_dzIMU;}
    virtual eeros::control::Input<Vector3>& getIn_enc(){return in_enc;}
    virtual eeros::control::Input<Vector3>& getIn_denc(){return in_denc;}
    
    //define outputs
    virtual eeros::control::Output<unsigned short>& getOut_Zustand(){return out_Zustand;}
    
    
    
  protected:
    //define inputs
    eeros::control::Input<double> in_ddzIMU;
    eeros::control::Input<double> in_dzIMU;
    eeros::control::Input<Vector3> in_enc;
    eeros::control::Input<Vector3> in_denc;
    
    //define outputs
    eeros::control::Output<unsigned short> out_Zustand;
    
    
  private:
    virtual void run();
    
    double ddzIMU, dzIMU, ddzIMU_1, dzIMU_1, dz_dif;
    Vector3 enc, denc;
    
    unsigned short Zustand;
    
    double Ts; 
    Vector3 enc_dif, enc_1;

    
    
  };// end class ZustBest
}// end namespace ZustBest


#endif // EINBEIN_ZUSTBEST_HPP
