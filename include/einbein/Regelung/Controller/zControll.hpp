#ifndef EINBEIN_CONTROLLER_HPP
#define EINBEIN_CONTROLLER_HPP

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/math/Matrix.hpp>


using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


namespace einbein{
  class zControll : public eeros::control::Block{
    
    public:
	      zControll();
	      virtual ~zControll();
	     
	      //define inputs
	      virtual eeros::control::Input<double>& getIn_dEncoder(){return in_dEncoder;}
	      virtual eeros::control::Input<int>& getIn_Zustand(){return in_Zustand;}  
	      
	      //define outputs
	      virtual eeros::control::Output<double>& getOut_Fz(){return out_Fz;}
	     
	 
	      
	    
    protected: 
	      //define inputs
	      eeros::control::Input<double> in_dEncoder;	
	      eeros::control::Input<int> in_Zustand;
	      
	      
	      //define outputs
	      eeros::control::Output<double> out_Fz;

  
	      
    private:  
//-----------------run Methode------------------------------------------------	      
	      virtual void run();
	      double d_enc1, Fz;
	      int Zustand;
	    

    
  };//end class ZCONTROLLER
  
  
}//end namspace einbein

#endif // EINBEIN_CONTROLLER_HPP