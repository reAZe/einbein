#ifndef EINBEIN_TRANSITIONBLOCK_HPP
#define EINBEIN_TRANSITIONBLOCK_HPP

#include <mutex>

#include <eeros/math/Matrix.hpp>
#include <eeros/control/TransitionBlock.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>


using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;



namespace einbein{
  class TransitionBlockEncoder : public eeros::control::TransitionBlock{
    
    public:
	      TransitionBlockEncoder();
	      virtual ~TransitionBlockEncoder();
	     
	      //define inputs
	      virtual eeros::control::Input<Vector3>& getIn_fEnc() {return in_fEnc;}
	      virtual eeros::control::Input<Vector3>& getIn_d_fEnc() {return in_d_fEnc;}
	  
	      
	      //define outputs
	      virtual eeros::control::Output<Vector3>& getOut_sEnc(){return out_sEnc;}
	      virtual eeros::control::Output<Vector3>& getOut_d_sEnc(){return out_d_sEnc;}
	    
    protected: 
	      //define inputs
	      eeros::control::Input<Vector3> in_fEnc;
	      eeros::control::Input<Vector3> in_d_fEnc;
	      
	      //define outputs
	      eeros::control::Output<Vector3> out_sEnc;
	      eeros::control::Output<Vector3> out_d_sEnc;
  
	      
    private:  
//-----------------run Methode------------------------------------------------	      
	      virtual void runA(); // timedomain fast
	      virtual void runB(); // timedomain slow
	      
	      Vector3 enc_transition;
	      
	      std::mutex mutex;

    
  };//end class TransitionBlockEncoder
  
  
}//end namspace einbein


#endif // EINBEIN_TRANSITIONBLOCK_HPP