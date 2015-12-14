#ifndef EINBEIN_CONSTINPUT_HPP
#define EINBEIN_CONSTINPUT_HPP


#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/math/Matrix.hpp>



using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


namespace einbein{
  class constInput : public eeros::control::Block{
    
    public:
	      constInput();
	      virtual ~constInput();
	     
	      //define inputs
	      virtual eeros::control::Input<Vector3>& getIn_xSoll(){return in_xSoll;}
	      virtual eeros::control::Input<Vector3>& getIn_xStart(){return in_xStart;}
	      
	      
	      //define outputs
	      virtual eeros::control::Output<Vector3>& getOut_xStartConst(){return out_xStartConst;}
      
	    
    protected: 
	      //define inputs
	      eeros::control::Input<Vector3> in_xSoll;	
	      eeros::control::Input<Vector3> in_xStart;
      
	      //define outputs
	      eeros::control::Output<Vector3> out_xStartConst;

	      
    private:  
//-----------------run Methode------------------------------------------------	      
	      virtual void run();
	      Vector3 xSoll, xStart, xStartConst;
	      Vector3 xSoll_1;

    
  };//end class constInput
  
  
}//end namspace einbein

#endif // EINBEIN_CONSTINPUT_HPP