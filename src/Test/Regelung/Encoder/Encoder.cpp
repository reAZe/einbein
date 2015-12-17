#include <cmath>
#include <einbein/Regelung/Base2Tool/constantFusspunkt.hpp>
#include <einbein/Regelung/Encoder/Encoder.hpp>
#include <einbein/templates_function.hpp>
#include <eeros/hal/FlinkFqd.hpp>
#include <eeros/core/System.hpp>

#include "iostream"

//namespace
using namespace einbein;
using namespace eeros;
using namespace eeros::hal;
using namespace eeros::math;


//Konstruktor
Encoder::Encoder(double ts):
      enc1("mot1Enc"),
      enc2("mot2Enc"),
      enc3("mot3Enc"){
	
      enc1_read_1 = 0;
      enc2_read_1 = 0;
      enc3_read_1 = 0;
      
      d_enc1_read = 0;
      d_enc2_read = 0;
      d_enc3_read = 0;
		
       Ts = ts;
      printf("Ts %f\n", Ts);
      //Anfangswert setzen
      enc = {0, 0, 0};
      d_enc = {0, 0, 0};
      
      out_enc.getSignal().setValue(enc); 
      out_d_enc.getSignal().setValue(d_enc);

};

//Destruktor
Encoder::~Encoder(){};



void Encoder::run(){
//-----------------------------  set Input-------------------------------------------------
// kein Input--> es wird über flink die Daten des Encoders eingelesen
	  
  
  
//-----------------------------  run ------------------------------------------------- 

//Input  
//   enc1 von Motor1
//   enc2 von Motor2
//   enc3 von Motor3  
      enc1.run();
      enc2.run();
      enc3.run();
      
  
      enc1_read = enc1.getOut().getSignal().getValue();
      enc2_read = enc2.getOut().getSignal().getValue();
      enc3_read = enc3.getOut().getSignal().getValue();
      
 
      //Geschwindigkeitsänderung (Differentation)
      d_enc1_read   = (enc1_read - enc1_read_1)/Ts;
      d_enc2_read   = (enc2_read - enc2_read_1)/Ts;
      d_enc3_read   = (enc3_read - enc3_read_1)/Ts;
      

      //Werte Übergeben
      enc1_read_1   = enc1_read;
      enc2_read_1   = enc2_read;
      enc3_read_1   = enc3_read;

      
//Output  
//   enc(0) von Motor3
//   enc(1) von Motor2
//   enc(3) von Motor1      
      
      enc(0) = enc3_read;
      enc(1) = enc1_read;
      enc(2) = enc2_read;
      
      d_enc(0) = d_enc3_read;
      d_enc(1) = d_enc1_read;
      d_enc(2) = d_enc2_read;
           
      
//-----------------------------  set Output -------------------------------------------------
      out_enc.getSignal().setValue(enc);  
      out_d_enc.getSignal().setValue(d_enc); 
      //set Timestamp
      //out_enc.getSignal().setTimestamp(in_Zustand.getSignal().getTimestamp());
      out_enc.getSignal().setTimestamp(eeros::System::getTimeNs());
     
      //set Name
      out_enc.getSignal().setName("enc1: enc2: enc3 [m]");
      out_d_enc.getSignal().setName("d_enc1: d_enc2: d_enc3 [m/s]");
      
}//end run





  


