#ifndef EINBEIN_PDV_HPP
#define EINBEIN_PDV_HPP


#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/math/Matrix.hpp>



using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


namespace einbein{
  class PDV : public eeros::control::Block{
    
    public:
	      PDV(double Kp, double Kd, double M, double ts);
	      virtual ~PDV();
	     
	      //define inputs
	      virtual eeros::control::Input<double>& getIn_xSoll_0(){return in_xSoll_0;}
	      virtual eeros::control::Input<double>& getIn_xIst_0(){return in_xIst_0;}
	      
	      //define outputs
	      virtual eeros::control::Output<double>& getOut_F_0(){return out_F_0;}
	      virtual eeros::control::Output<double>& getOut_d_xSoll_0(){return out_d_xSoll_0;}
	      virtual eeros::control::Output<double>& getOut_dd_xSoll_0(){return out_dd_xSoll_0;}
	      virtual eeros::control::Output<double>& getOut_d_xIst_0(){return out_d_xIst_0;}
	      
	      
	      
	    
    protected: 
	      //define inputs
	      eeros::control::Input<double> in_xSoll_0;	
	      eeros::control::Input<double> in_xIst_0;	
      
	      //define outputs
	      eeros::control::Output<double> out_F_0;
	      eeros::control::Output<double> out_d_xSoll_0;
	      eeros::control::Output<double> out_dd_xSoll_0;
	      eeros::control::Output<double> out_d_xIst_0;

	      
    private:  
//-----------------run Methode------------------------------------------------	      
	      virtual void run();
	      double kp_Vk, Ts, kv_Vk, m_Vk;
	      //input
	      double xSoll_0, xIst_0;
	      
	      //intern
	      double kp, kd, m, kp_, kd_, Tv, Tb;
	      double y, y_1, e, e_1;
	      
	      double dxIst, xIst_1, xMax, s, vMax, aMax, vSoll, KP;
	      
	      
	      //output
	      double F_0;

	      

	   

    
  };//end class PDV
  
  
}//end namspace einbein

#endif // EINBEIN_PDV_HPP