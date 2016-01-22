#include <einbein/Regelung/ZustBest/ZustBest.hpp>
#include <einbein/Regelung/ZustBest/constantZustBest.hpp>
#include <unistd.h>
#include <einbein/templates_function.hpp>

//namespace
using namespace einbein;
using namespace eeros;
using namespace eeros::math;


//Konstruktor
ZustBest::ZustBest(double ts){
  Zustand = 1; 	//Anfangszustand
  i_wait = 0; 	//Zählvariable
  this->ts=ts;
};

//Destruktor
ZustBest::~ZustBest(){};



void ZustBest::run(){
  /* --------------------------------
   * | Situationen des Hüpfroboters |
   * --------------------------------
   * Zustand 0: Fehler
   * Zustand 1: Luft
   * Zustand 2: Landung
   * Zustand 3: Einziehen
   * Zustand 4: Scheitelpunkt Boden
   * Zustand 5: Schub
   * Zustand 6: Absprung
   */
  
  
  //----------------------------- set Input -----------------------------

  enc		= in_enc.getSignal().getValue();
  denc 		= in_denc.getSignal().getValue();
     
  
  //----------------------------- run -----------------------------
  
  
    switch (Zustand){
    case 1://Luft
        if(((denc(0) < toleranzBodenGeschw) && (denc(1) < toleranzBodenGeschw)) || ((denc(0) < toleranzBodenGeschw) && (denc(2) < toleranzBodenGeschw)) || ((denc(1) < toleranzBodenGeschw) && (denc(2) < toleranzBodenGeschw))){
	    Zustand = 2;
	}
      
	break; //case 1
        

    case 2://Landung --> transienter Zustand
       if(waitFunction(time_Landung,ts)){//wait
	  Zustand = 3;
       }
       break; //case 2

    case 3: //Einziehen
	if(signum(denc(0)>0) && signum(denc(1)>0) && signum(denc(2)>0)){
	    Zustand = 4;
	}     
	break;
	
	
    case 4: //Scheitelpunkt Boden --> transienter Zustand
	 if(waitFunction(time_Scheitelpunkt,ts)){//wait
	    if(signum(denc(0)>0) && signum(denc(1)>0) && signum(denc(2)>0)){
	      Zustand = 5;
	    }  
	    else {
	      Zustand = 3;
	    }
	 }
	    
	break; //case 4

	
    case 5://Schub
        if (norm(enc) > toleranzLuft){
            Zustand = 6;
	}
	break; //case 5

    case 6: //Absprung --> transienter Zustand 
         if(waitFunction(time_Absprung,ts)){//wait
	     Zustand = 1;
	  }
 	break; //case 6
      
  }//enc case Zustand
  
    
  
  //Versuch ohne Signum Geschwindigket und warten
  /*
  switch (Zustand){
    case 1://Sinkflug
        if(((denc(0) < toleranzBodenGeschw) && (denc(1) < toleranzBodenGeschw)) || ((denc(0) < toleranzBodenGeschw) && (denc(2) < toleranzBodenGeschw)) || ((denc(1) < toleranzBodenGeschw) && (denc(2) < toleranzBodenGeschw))){
            Zustand = 2;
	}
      
	break; //case 1
        

    case 2://Landung
       Zustand = 3;
       usleep(1000);
       break; //case 2

    case 3: //Scheitelpunkt Boden: es wird nur ein Encoder beobachtet
      if(norm(enc) < 0.078){
	for(unsigned i = 0; i <= 2; ++i){
	  if ((-toleranzScheitelBodenGeschw <= denc(i)) && (denc(i) <= toleranzScheitelBodenGeschw)){ 
		  Zustand = 4;
		  usleep(5000);	//warten, dass Umkehrpunkt sicher erreicht ist
	  }
	}
      }
	break; //case 3
	
    case 4:
	//usleep(1000);
        Zustand = 5;
	break; //case 4

    case 5:
        if (norm(enc) > toleranzLuft){
            Zustand = 6;
	}
	break; //case 5

    case 6:  
	usleep(1000);
        Zustand = 1;
 	break; //case 6
      
  }//enc case Zustand
  */

  
  
  //----------------------------- set Output -----------------------------
  out_Zustand.getSignal().setValue(Zustand);
  out_Zustand.getSignal().setTimestamp(in_enc.getSignal().getTimestamp()); 
}// end run




bool ZustBest::waitFunction(double time, double Ts){
  if (i_wait >= (time/Ts)){
    i_wait = 0;
    return true;
  }
  else{
    i_wait++;
    return false;
  }
  
}
