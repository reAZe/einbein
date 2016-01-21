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
    Zustand		= in_Zustand.getSignal().getValue();

 
//-----------------------------  run ------------------------------------------------- 
  // Fz = signum(d_enc1)*(-1000);
   
  switch (Zustand){
    
    case 0: //Fehler
	F_z = 0;
	xf_end_0   = 0;				yf_end_0   = 0;
	F_beta1_x  = 0;        			F_beta1_z  = 0;
	F_alpha1_y = 0;       			F_alpha1_z = 0;  
	printf("error Controller.cpp: Zustand 0 wird angegeben\n");
	
	break;//case 0
		
    case 1: //Luft
	    F_z = 0;
	    F_beta1_x  = 0;        			F_beta1_z  = 0;
	    F_alpha1_y = 0;       			F_alpha1_z = 0;
	    
	  //Positionsregelung x-Richtung
	    if(posx_calc == 0){/*
	      Positionsregelung: 
	      Positionsregelung berechnet eine
	      Sollgeschwindigkeit,welche der Geschwindigkeitregelung übergeben 
	      wird*/
	      vx_Platte_soll = Kp_vx * (x_Platte_soll - rx_Platte_Landung);
	    
	      //maximale Geschwindigkeit
	      if (vx_Platte_soll >= vx_Platte_max){
		  vx_Platte_soll = vx_Platte_max;
	      }//end if
	      if (vx_Platte_soll <= -vx_Platte_max){
		  vx_Platte_soll = -vx_Platte_max;
	      }// end if


	      //Geschwindigkeitsregelung
	      xf_end_intern = vx_Platte_Absprung * T_Boden/2 + Kp_x*(vx_Platte_Absprung - vx_Platte_soll);
		
	      posx_calc = 1;
	    }//end if posx_calc

	    
	    //Positionsregelung y-Richtung 
	    if(posy_calc == 0){/*
	      Positionsregelung: 
	      Positionsregelung berechnet eine
	      Sollgeschwindigkeit,welche der Geschwindigkeitregelung übergeben 
	      wird*/
	      vy_Platte_soll = Kp_vy * (y_Platte_soll - ry_Platte_Landung);
	      
	      //maximale Geschwindigkeit
	      if (vy_Platte_soll >= vy_Platte_max){
		  vy_Platte_soll = vy_Platte_max;
	      }//end if
	      if (vy_Platte_soll <= -vy_Platte_max){
		  vy_Platte_soll = -vy_Platte_max;
	      }// end if
      
	      // Geschwindigkeitsregelung
	      yf_end_intern = vy_Platte_Absprung * 0.3 + Kp_y*(vy_Platte_Absprung - vy_Platte_soll);
			
	    posx_calc = 1;
	    }//end if posx_calc
      
      
	    xf_end_0 =  xf_end_intern;
	    yf_end_0 =  yf_end_intern;
	    
	   //TODO PID-Regler für Höhenregelung einfügen. Siehe Simulink, Block Controller
	    
	    
	    break;//case 1
    
       
    case 3: //Einziehen
      F_z = 1000;
      break;
      
    case 5:
      F_z = -1000;
      break;  
      
    default:
      F_z = 0;
      
    
  }
     //Fz = 0;      
      
//-----------------------------  set Output -------------------------------------------------
      out_xf_end_0.getSignal().setValue(xf_end_0);
      out_yf_end_0.getSignal().setValue(yf_end_0);
      out_F_beta1_x.getSignal().setValue(F_beta1_x); 
      out_F_alpha1_y.getSignal().setValue(F_alpha1_y); 
      out_F_z.getSignal().setValue(F_z);   
      
      //set Timestamp
      out_xf_end_0.getSignal().setTimestamp(in_Zustand.getSignal().getTimestamp());
      out_yf_end_0.getSignal().setTimestamp(in_Zustand.getSignal().getTimestamp());
      out_F_beta1_x.getSignal().setTimestamp(in_Zustand.getSignal().getTimestamp());
      out_F_alpha1_y.getSignal().setTimestamp(in_Zustand.getSignal().getTimestamp());
      out_F_z.getSignal().setTimestamp(in_Zustand.getSignal().getTimestamp());

      //set Name
      out_xf_end_0.getSignal().setName("Sollwert Fusspositoin x-Richtung [m]");
      out_yf_end_0.getSignal().setName("Sollwert Fusspositoin y-Richtung [m]");
      out_F_beta1_x.getSignal().setName("Kraft in x-Richtung für Körperhalteregelung [N]");
      out_F_alpha1_y.getSignal().setName("Kraft in y-Richtung für Körperhalteregelung [N]");
      out_F_z.getSignal().setName("Kraft in z-Richtung für den Sprung [N]");

      
}//end run

