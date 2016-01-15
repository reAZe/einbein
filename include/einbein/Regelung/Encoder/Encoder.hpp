#ifndef EINBEIN_ENCODER_HPP
#define EINBEIN_ENCODER_HPP


#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/math/Matrix.hpp>

#include <eeros/control/PeripheralInput.hpp>
#include <einbein/Regelung/Controller/constantController.hpp>
#include <eeros/hal/FlinkFqd.hpp>


using namespace eeros;
using namespace eeros::hal;
using namespace eeros::control;
using namespace eeros::logger;
using namespace einbein;


namespace einbein{
  class Encoder : public eeros::control::Block{
    
    public:
	      Encoder(double ts);
	      virtual ~Encoder();
	     
	      //define inputs
	      //keine Inputs
	  	      
	      //define outputs
	      virtual eeros::control::Output<Vector3>& getOut_enc(){return out_enc;}
	      virtual eeros::control::Output<Vector3>& getOut_d_enc(){return out_d_enc;}
	      
	    
    protected: 
	      //define inputs
	      //keine Inputs
	      
	      //define outputs
	      eeros::control::Output<Vector3> out_enc;
	      eeros::control::Output<Vector3> out_d_enc;
  
	      
    private:  
//-----------------run Methode------------------------------------------------	      
	      virtual void run();
	      
	      //Input flink
	      eeros::control::PeripheralInput<double> enc1;
	      eeros::control::PeripheralInput<double> enc2;
	      eeros::control::PeripheralInput<double> enc3;
	      
	      double enc1_read, enc2_read, enc3_read;
	      double d_enc1_read, d_enc2_read, d_enc3_read;
	      double Ts; 
	      double enc1_read_1, enc2_read_1, enc3_read_1;
	      Vector3 enc, d_enc, d_enc_Filter;
	      
	      
	      //Filter 
	      Vector3 d_enc_1, d_enc_2, d_enc_3, d_enc_4, d_enc_5, d_enc_6, d_enc_7, d_enc_8;
	      double a0, a1, a2, a3, a4, a5, a6, a7, a8;
	      
	      

    
  };//end class Encoder
  
  
}//end namspace einbein

#endif // EINBEIN_ENCODER_HPP