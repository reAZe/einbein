#include <cmath>
#include <einbein/Regelung/PDV/PDV.hpp>
#include <einbein/templates_function.hpp>

#include <iostream>


//namespace
using namespace einbein;
using namespace eeros;
using namespace eeros::math;


//Konstruktor
PDV::PDV(double Kp, double Kd, double M, double ts){
    
    kp = Kp;
    kd = Kd;
    m = M;
    Ts = ts; 
    
    kp_ = kp*m;
    kd_ = kd*m;
    Tv  = kd_/kp_;
    Tb  = 0.1*Tv; 	//Nachstellzeit
        
    //Anfangszustand des Ausgang setzen
    out_F_0.getSignal().setValue(0);
    
    
    
    xIst_1 = 0; dxIst = 0;
    s = 0.02;
    vMax = 2;
    aMax = 5;
    KP = 20;
    
     
};

//Destruktor
PDV::~PDV(){};



void PDV::run(){
//-----------------------------  set Input---------------------------------------------
      xSoll_0 		= in_xSoll_0.getSignal().getValue();
      xIst_0		= in_xIst_0.getSignal().getValue(); 
    
   //--------------------------- PDV-Regler -------------------------------------------
      //printf("xSoll_0 %f; xIst_0(e) %f, \n" , xSoll_0,  xIst_0);
      e = xSoll_0 - xIst_0;
      dxIst = (xIst_0-xIst_1)/Ts;
      //xMax = xSoll_0 - s - (vMax*vMax)/(2*aMax);
      xMax = 0.1;
      //printf("e %f; fabs(e) %f, s %f, xMax %f, \n" , e,  fabs(e), s, xMax);
      if (fabs(e) <= 2*s){//um Nullpunkt    
	y = ((kp_*Ts+kp_*Tv)*e-kp_*Tv*e_1 + Tb*y_1)/(Ts+Tb);
	//saturation
	if(y > 100){
	    y = 100;
	    }
	else if(y < -100){
	    y = -100;
	    }
	else if(isnan(y)){
	    y = 0;
	    printf("NaN in PD.cpp");
	    }
	else if(isinf(y)){
	    y = 0;
	    printf("inf in PD.cpp");
	    }
      }//end if abs(e)<2s

      else if((e > 2*s) && (e < xMax)){//Wurzelfunktion positiv
	vSoll = sqrt(2*aMax*fabs(e)-s);
	y = KP *(vSoll- dxIst);
	//printf("Wurzel positiv/n");
      }
      
      else if ((e < -2*s) && (e > -xMax)){//Wurzelfunktion negativ
	vSoll = -sqrt(2*aMax*fabs(e)+s);
	y = KP *(vSoll- dxIst);
	//printf("Wurzel neg/n");
      }
      
      else if (fabs(e) > xMax){//Maximale Geschwindigkeit
	vSoll = signum(e)*vMax;
	y = KP *(vSoll- dxIst);
	//printf("Wurzel max/n");
      }
      else{printf("nothing/n");}
      
  
      y_1 = y;
      e_1 = e;
      xIst_1 = xIst_0;
	
    //-----------------------------  set Output ------------------------------------------ 

      out_F_0.getSignal().setValue(y);
      out_d_xSoll_0.getSignal().setValue(0);
      out_dd_xSoll_0.getSignal().setValue(0);
      out_d_xIst_0.getSignal().setValue(e);
      
      
      //set Timestamp
      out_F_0.getSignal().setTimestamp(in_xSoll_0.getSignal().getTimestamp());
      out_d_xSoll_0.getSignal().setTimestamp(in_xSoll_0.getSignal().getTimestamp());
      out_dd_xSoll_0.getSignal().setTimestamp(in_xSoll_0.getSignal().getTimestamp());
      out_d_xIst_0.getSignal().setTimestamp(in_xSoll_0.getSignal().getTimestamp());

      
      //set Name
      out_F_0.getSignal().setName("Sollkratft im {0}  [m]");
      out_d_xSoll_0.getSignal().setName("d_xSoll [m/s]");
      out_dd_xSoll_0.getSignal().setName("dd_xSoll [m/s²]");
      out_d_xIst_0.getSignal().setName("d_xIst [m/s]");
     
    
}//end run



      



  


