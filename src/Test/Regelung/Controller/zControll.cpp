#include <cmath>
#include <einbein/Regelung/Controller/zControll.hpp>
#include <einbein/templates_function.hpp>

#include "iostream"

//namespace
using namespace einbein;
using namespace eeros;
using namespace eeros::math;


//Konstruktor
zControll::zControll(){
  
};

//Destruktor
zControll::~zControll(){};



void zControll::run(){
//-----------------------------  set Input-------------------------------------------------
    d_enc1 		= in_dEncoder.getSignal().getValue();
    Zustand		= in_Zustand.getSignal().getValue();

 
//-----------------------------  run ------------------------------------------------- 
  // Fz = signum(d_enc1)*(-1000);
   
  switch (Zustand){
    
    case 3: //Einziehen
      Fz = 1000;
      break;
      
    case 5:
      Fz = -1000;
      break;  
      
    default:
      Fz = 0;
      
    
  }
     //Fz = 0;      
      
//-----------------------------  set Output -------------------------------------------------
    out_Fz.getSignal().setValue(Fz);  
    

    //set Timestamp
    out_Fz.getSignal().setTimestamp(in_dEncoder.getSignal().getTimestamp());

      
}//end run

