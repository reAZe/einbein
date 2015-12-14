#ifndef EINBEIN_PMOTOR_HPP
#define EINBEIN_PMOTOR_HPP


#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/math/Matrix.hpp>



using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


namespace einbein{
  class PMotor : public eeros::control::Block{
    
    public:
	      PMotor();
	      virtual ~PMotor();
	     
	      //define inputs
	      virtual eeros::control::Input<Vector3>& getIn_FSoll(){return in_FSoll;}
	      virtual eeros::control::Input<Vector3>& getIn_enc(){return in_enc;}
	      virtual eeros::control::Input<Vector3>& getIn_d_enc(){return in_d_enc;}
	      
	      //define outputs
	      virtual eeros::control::Output<Vector3>& getOut_FSollMot(){return out_FSollMot;}
      
	    
    protected: 
	      //define inputs
	      eeros::control::Input<Vector3> in_FSoll;	
	      eeros::control::Input<Vector3> in_enc;
	      eeros::control::Input<Vector3> in_d_enc;
      
	      //define outputs
	      eeros::control::Output<Vector3> out_FSollMot;

	      
    private:  
//-----------------run Methode------------------------------------------------	      
	      virtual void run();
	      //input
	      Vector3 FSoll, enc, d_enc;

	      
	      //intern
	      Vector3 vMax_1, vMax_2;
	            

	      
	      //output
	      Vector3 FSollMot;
	    

	      

	   

    
  };//end class PMotor
  
  
}//end namspace einbein

#endif // EINBEIN_PMOTOR_HPP