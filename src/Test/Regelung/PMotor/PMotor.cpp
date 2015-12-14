#include <cmath>
#include <einbein/Regelung/PMotor/PMotor.hpp>
#include <einbein/Regelung/PMotor/constantsPMotor.hpp>
#include <iostream>



//namespace
using namespace einbein;
using namespace eeros;
using namespace eeros::math;


//Konstruktor
PMotor::PMotor(){ 
    FSollMot.zero();
    vMax_1.zero();
    vMax_2.zero();
    //Anfangszustand des Ausgang setzen
    out_FSollMot.getSignal().setValue(FSollMot);
};

//Destruktor
PMotor::~PMotor(){};



void PMotor::run(){
//-----------------------------  set Input---------------------------------------------
      FSoll 		= in_FSoll.getSignal().getValue();
      enc		= in_enc.getSignal().getValue(); 
      d_enc		= in_d_enc.getSignal().getValue();
 
    //--------------------------- P-Regler Motor (Anschlagsregelung) -------------------------------------------

      //Berechnen der maximalen Geschwindigkeit an den Enden.   
   for (int i=0; i < 3; i++){
     vMax_1(i) = -sqrt(2*a_max*(enc(i)-x_max1(i)));
     vMax_2(i) = sqrt(2*a_max*(x_max2(i)-enc(i)));
     }

      
      //NAN
    for (int i = 0; i < 3; i++){
      if (isnan(vMax_1(i))){vMax_1(i) = 0;}
      if (isnan(vMax_2(i))){vMax_2(i) = 0;}
    }    
           
      
      //Ausgang schalten (switch zwischen Anschlagsregelung und kartesische Fusspunktregelung)
    for (int i = 0; i < 3; i++){
      if(vMax_1(i) >= d_enc(i)){
	FSollMot(i) = kp1(i)*(vMax_1(i)-d_enc(i));
      }
      
      else if (vMax_2(i) <= d_enc(i)){
	FSollMot(i) = kp2(i)*(vMax_2(i)-d_enc(i));
	      }
      
      else {
	FSollMot(i) = FSoll(i);
	}
    }
  

	
    //-----------------------------  set Output ------------------------------------------ 

      out_FSollMot.getSignal().setValue(FSollMot);
       
      //set Timestamp
      out_FSollMot.getSignal().setTimestamp(in_FSoll.getSignal().getTimestamp());

      //set Name
      out_FSollMot.getSignal().setName("Motorenkraft [m]");
     
    
}//end run






  


