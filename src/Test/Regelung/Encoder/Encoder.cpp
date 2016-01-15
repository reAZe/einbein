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
      
      
     //Filter
    d_enc_1.zero();	d_enc_2.zero();   	d_enc_3.zero();		d_enc_4.zero();	   
    d_enc_5.zero();	d_enc_6.zero();		d_enc_7.zero();		d_enc_8.zero();
    //Parameter FIR-Filter (Grenzfrequenz bei 50 Hz)
    a0 = 0.008493236699016; a1 = 0.04447252355589; a2 = 0.1172425051929; a3 = 0.1990075219855;
    a4 = 0.2357294785155;   a5 = 0.1990075219855;  a6 = 0.1172425051929; a7 = 0.04447252355589;
    a8 = 0.008493236699016;
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
      
      
//set Output  
//   enc(0) von Motor3
//   enc(1) von Motor2
//   enc(3) von Motor1      
      
      enc(0) = enc3_read;
      enc(1) = enc1_read;
      enc(2) = enc2_read;
      
      d_enc(0) = d_enc3_read;
      d_enc(1) = d_enc1_read;
      d_enc(2) = d_enc2_read;
      
      
//Filter 
      for (int i = 0; i < 3; i++){
	d_enc_Filter(i) = a0*d_enc(i) + a1*d_enc_1(i) + a2*d_enc_2(i) + a3*d_enc_3(i) + a4*d_enc_4(i) + a5*d_enc_5(i) + a6*d_enc_6(i)  +  a7*d_enc_7(i)  + a8*d_enc_8(i) ;
      }
      
      d_enc_8 = d_enc_7;
      d_enc_7 = d_enc_6;
      d_enc_6 = d_enc_5;
      d_enc_5 = d_enc_4;
      d_enc_4 = d_enc_3;
      d_enc_3 = d_enc_2;
      d_enc_2 = d_enc_1;
      d_enc_1 = d_enc;
      
      d_enc_1 = d_enc;
      
           
      
//-----------------------------  set Output -------------------------------------------------
      out_enc.getSignal().setValue(enc);  
      out_d_enc.getSignal().setValue(d_enc_Filter); 
      //set Timestamp
      //out_enc.getSignal().setTimestamp(in_Zustand.getSignal().getTimestamp());
      out_enc.getSignal().setTimestamp(eeros::System::getTimeNs());
     
      //set Name
      out_enc.getSignal().setName("enc1: enc2: enc3 [m]");
      out_d_enc.getSignal().setName("d_enc1: d_enc2: d_enc3 [m/s]");
      
}//end run





  


