#include <cmath>
#include <einbein/Regelung/PDV/PDV.hpp>
#include <einbein/templates_function.hpp>

#include <iostream>


//namespace
using namespace einbein;
using namespace eeros;
using namespace eeros::math;


//Konstruktor
PDV::PDV(double Kp, double Kd, double M, double Kv, double ts){
    
    //PD-Regler
    kp = Kp;
    kd = Kd;
    m = M;
    Ts = ts; 
    
    //PD-Regler mit Nachstellzeit
    kp_ = kp*m;
    kd_ = kd*m;
    Tv  = kd_/kp_;
    Tb  = 0.1*Tv; 	//Nachstellzeit  
    
    //Wurzelregler
    s = 0.00025;
    vMax = 0.5;
    aMax = 15;
    KP = Kv; 		//Kp = 2.8 
    
    //Ableitungen
    xSoll_1 = 0; xIst_1 = 0; dxIst = 0;
    
    //Filter
    dxIst_1 = 0; dxIst_2 = 0; dxIst_3 = 0; dxIst_4 = 0;
    dxIst_5 = 0; dxIst_6 = 0; dxIst_7 = 0; dxIst_8 = 0;
    //Parameter FIR-Filter (Grenzfrequenz bei 50 Hz)
    a0 = 0.008493236699016; a1 = 0.04447252355589; a2 = 0.1172425051929; a3 = 0.1990075219855;
    a4 = 0.2357294785155;   a5 = 0.1990075219855;  a6 = 0.1172425051929; a7 = 0.04447252355589;
    a8 = 0.008493236699016;
    
    //Anfangszustand des Ausgang setzen
    out_F_0.getSignal().setValue(0);
    
     
};

//Destruktor
PDV::~PDV(){};



void PDV::run(){
//-----------------------------  set Input---------------------------------------------
      xSoll_0 		= in_xSoll_0.getSignal().getValue();
      xIst_0		= in_xIst_0.getSignal().getValue(); 
    
   //--------------------------- PDV-Regler -------------------------------------------
      
      //Regelaweichung
      e = xSoll_0 - xIst_0;
      
      //Ableitung bilden
      dxSoll = (xSoll_0 -xSoll_1)/Ts;
      dxIst = (xIst_0-xIst_1)/Ts;
      dxIstFilter = a0*dxIst + a1*dxIst_1 + a2*dxIst_2 + a3*dxIst_3 + a4*dxIst_4 + a5*dxIst_5  + a6*dxIst_6  +  a7*dxIst_7  + a8*dxIst_8 ;
      //Grenzposition zwischen maximalen Geschwindigkeit und Wurzelfunktion
      xMaxPositiv = (vMax*vMax)/(2*aMax) + s;
      xMaxNegativ = -(vMax*vMax)/(2*aMax) - s;
     
  
      
      
      
      
     
      //Wahl des Reglers
      if (fabs(e) <= 2*s){//um Nullpunkt 
	//PD-Regler mit Nachstellzeit
	//y = ((kp_*Ts+kp_*Tv)*e-kp_*Tv*e_1 + Tb*y_1)/(Ts+Tb);
	
	//PD-Regler ohne Nachstellzeit und gefilterter Geschwindigkeit
	y = m*(kp*e+kd*(dxSoll-dxIstFilter));
	//y = 0;
	//saturation
// 	if(y > 100){
// 	    y = 100;
// 	    }
// 	else if(y < -100){
// 	    y = -100;
// 	    }
// 	else if(isnan(y)){
// 	    y = 0;
// 	    printf("NaN y in PD.cpp\n");
// 	    }
// 	else if(isinf(y)){
// 	    y = 0;
// 	    printf("inf y in PD.cpp\n");
// 	    }
      }//end if abs(e)<2s
      


     else if (e < xMaxNegativ){//Maximale negative Geschwindigkeit
	vSoll = -vMax;
	y = KP *(vSoll- dxIstFilter);
	//printf("Wurzel max/n");
      }//end Maximale Geschwindigkeit
  
  
     else if (e > xMaxPositiv){//Maximale positive Geschwindigkeit
	vSoll = vMax;
	y = KP *(vSoll- dxIstFilter);
	//printf("Wurzel max/n");
      }//end Maximale Geschwindigkeit
 
 
     else if ((xMaxNegativ < e) && (e < -2*s)){//Wurzelfunktion negativ
	vSoll = -sqrt(fabs(2*aMax*(e+s)));
	y = KP *(vSoll- dxIstFilter);
	//printf("Wurzel neg/n");
      }//end Wurzelfunktion negativ 
      
      
      else if((2*s < e) && (e < xMaxPositiv)){//Wurzelfunktion positiv
	vSoll = sqrt(2*aMax*(e-s));
	  if(isnan(vSoll)){"NaN vsoll in PDV.cpp\n"; }
	y = KP *(vSoll- dxIstFilter);
	//printf("Wurzel positiv/n");
      }//end Wurzelfunktion positiv
      
     
     
      else{ y = 0;
	  printf("nothing to do in pdv.cpp/n");
      }
      

      y_1 = y;
      e_1 = e;
      xIst_1 = xIst_0;
      xSoll_1 = xSoll_0;
      
      //Filter  
      dxIst_8 = dxIst_7;
      dxIst_7 = dxIst_6;  
      dxIst_6 = dxIst_5;
      dxIst_5 = dxIst_4;
      dxIst_4 = dxIst_3;   
      dxIst_3 = dxIst_2;
      dxIst_2 = dxIst_1;
      dxIst_1 = dxIst;

     
	
    //-----------------------------  set Output ------------------------------------------ 
      out_F_0.getSignal().setValue(y);

      //set Timestamp
      out_F_0.getSignal().setTimestamp(in_xSoll_0.getSignal().getTimestamp());


      //set Name
      out_F_0.getSignal().setName("Sollkratft im {0}  [m]");

    
}//end run



      



  


