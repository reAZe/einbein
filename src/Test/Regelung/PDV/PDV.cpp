#include <cmath>
#include <einbein/Regelung/PDV/PDV.hpp>
#include <einbein/templates_function.hpp>

#include <iostream>


//namespace
using namespace einbein;
using namespace eeros;
using namespace eeros::math;


//Konstruktor
PDV::PDV(double kp_vk, double kv_vk, double m_vk, double ts){
    kp_Vk = kp_vk;	//kp_vk = omega0/(2*D)
    kv_Vk = kv_vk;	//kv_Vk = 2*D*omega0
    m_Vk = m_vk;  
    Ts = ts;
    
    F_0 = 0;
        
    //Anfangszustand des Ausgang setzen
    out_F_0.getSignal().setValue(F_0);
};

//Destruktor
PDV::~PDV(){};



void PDV::run(){
//-----------------------------  set Input---------------------------------------------
      xSoll_0 		= in_xSoll_0.getSignal().getValue();
      xIst_0		= in_xIst_0.getSignal().getValue(); 
    
   //--------------------------- PDV-Regler -------------------------------------------
    
      
      //Ableitungen 
      d_xSoll_0   =(xSoll_0 - xSoll_0_1)/Ts;
      dd_xSoll_0  =(d_xSoll_0 - d_xSoll_0_1)/Ts;

      d_xIst_0    = (xIst_0 - xIst_0_1)/Ts;

      
      error_Kp    = xSoll_0 - xIst_0;
      d_x         = error_Kp*kp_Vk + d_xSoll_0 - d_xIst_0;
      //PD mit Vorsteuerung
      dd_x        = d_x*kv_Vk + dd_xSoll_0;
      F_0      	  = dd_x*m_Vk;
      
      //PD
      //F_0 	  = d_x*kv_Vk*m_Vk;



      xSoll_0_1   = xSoll_0;
      d_xSoll_0_1 = d_xSoll_0;
      xIst_0_1    = xIst_0;


      
	
    //-----------------------------  set Output ------------------------------------------ 

      out_F_0.getSignal().setValue(F_0);
       
      //set Timestamp
      out_F_0.getSignal().setTimestamp(in_xSoll_0.getSignal().getTimestamp());

      //set Name
      out_F_0.getSignal().setName("Sollkratft im {0}  [m]");
     
    
}//end run






  


