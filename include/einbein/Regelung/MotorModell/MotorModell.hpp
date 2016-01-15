#ifndef EINBEIN_MOTORWODELL_HPP
#define EINBEIN_MOTORMODELL_HPP


#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/math/Matrix.hpp>



using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


namespace einbein{
  class MotorModell : public eeros::control::Block{
    
    public:
	      MotorModell();
	      virtual ~MotorModell();
	     
	      //define inputs
	      virtual eeros::control::Input<Vector3>& getIn_FM_Soll(){return in_FM_Soll;}
	      
	      //define outputs
	      virtual eeros::control::Output<Vector3>& getOut_VM_Soll(){return out_VM_Soll;}
      
	    
    protected: 
	      //define inputs
	      eeros::control::Input<Vector3> in_FM_Soll;	
      
	      //define outputs
	      eeros::control::Output<Vector3> out_VM_Soll;

	      
    private:  
//-----------------run Methode------------------------------------------------	      
	      virtual void run();
	      
	      //input 
	      Vector3 FM_Soll;
	      double kF;
	      
	      //output
	      Vector3 IM_Soll, VM_Soll;
    
  };//end class MotorModell
  
  
}//end namspace einbein

#endif// EINBEIN_MOTORMODELL_HPP