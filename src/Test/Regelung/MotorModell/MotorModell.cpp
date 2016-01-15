#include <cmath>
#include <einbein/Regelung/MotorModell/MotorModell.hpp>
#include <einbein/templates_function.hpp>


//namespace
using namespace einbein;
using namespace eeros;
using namespace eeros::math;


//Konstruktor
MotorModell::MotorModell(){
    FM_Soll.zero();
    kF 	= 12.65;	//Force Constant (Datasheet Linear DC-Servomotors Faulhaber Series LM 1247 ..01)  [N/A]   
    //Anfangszustand des Ausgang setzen
    out_VM_Soll.getSignal().setValue(IM_Soll);
};

//Destruktor
MotorModell::~MotorModell(){};



void MotorModell::run(){
      //-----------------------------  set Input---------------------------------------------
      FM_Soll 	= in_FM_Soll.getSignal().getValue();
      
      
      //-----------------------------  run ------------------------------------------
      //Ausgang der Motoren sind vertauscht. In Regelung wird mit anderer Motorenkonstellation 
      //gearbeitet
      IM_Soll(0) = FM_Soll(0)/kF;
      IM_Soll(1) = FM_Soll(1)/kF;
      IM_Soll(2) = FM_Soll(2)/kF;
      
      VM_Soll = IM_Soll*1.0/0.045;		//0.045 entspricht Umrechungsfaktor bei +-10V mit Imax = 0.48
      
      VM_Soll = IM_Soll*1.0/0.145;		//0.145 entspricht Umrechungsfaktor bei +-10V mit Imax = 1.45
      
      //-----------------------------  set Output ------------------------------------------ 
      out_VM_Soll.getSignal().setValue(VM_Soll);
       
      //set Timestamp
      //out_IM_Soll.getSignal().setTimestamp(FM_Soll.getSignal().getTimestamp());

      //set Name
      out_VM_Soll.getSignal().setName("Motorenspannungen [V]");
     
    
}//end run






  


