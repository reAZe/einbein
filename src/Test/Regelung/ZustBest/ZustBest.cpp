#include <einbein/Regelung/ZustBest/ZustBest.hpp>
#include <einbein/Regelung/ZustBest/constantZustBest.hpp>
#include <unistd.h>
#include <einbein/templates_function.hpp>

//namespace
using namespace einbein;
using namespace eeros;
using namespace eeros::math;


//Konstruktor
ZustBest::ZustBest(){
  Zustand = 1; //Anfangszustand
  enc_1 = Vector3({0,0,0});
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
   
  enc_dif = enc - enc_1;
  
  
  //----------------------------- run -----------------------------
  switch (Zustand){
    case 1:
       // if(enc_dif(0) < -toleranzBoden) { 
// 	if(((enc_dif(0) < -toleranzBoden) && (enc_dif(1) < -toleranzBoden)) || ((enc_dif(0) < -toleranzBoden) && (enc_dif(2) < -toleranzBoden)) || ((enc_dif(1) < -toleranzBoden) && (enc_dif(2) < -toleranzBoden))){
//             Zustand = 2;
// 	}
      
      if(((denc(0) < toleranzBodenGeschw) && (denc(1) < toleranzBodenGeschw)) || ((denc(0) < toleranzBodenGeschw) && (denc(2) < toleranzBodenGeschw)) || ((denc(1) < toleranzBodenGeschw) && (denc(2) < toleranzBodenGeschw))){
            Zustand = 2;
	}
      
	break; //case 1
        

    case 2:
       //usleep(1000);
       Zustand = 3;
       break; //case 2

    case 3: //es wird nur ein Encoder beobachtet
        if ((-0.005 <= denc(0)) && (denc(0) <= 0.005)){ 
                Zustand = 4;
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
  
  enc_1 = enc;
  
  
  //----------------------------- set Output -----------------------------
  out_Zustand.getSignal().setValue(Zustand);
  
}// end run
