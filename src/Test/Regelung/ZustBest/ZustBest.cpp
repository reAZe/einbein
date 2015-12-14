#include <einbein/Regelung/ZustBest/ZustBest.hpp>
#include <einbein/Regelung/ZustBest/constantZustBest.hpp>

//namespace
using namespace einbein;
using namespace eeros;
using namespace eeros::math;


//Konstruktor
ZustBest::ZustBest(double ts){
  Ts = ts; 
  
  enc_1 = Vector3({0,0,0});
};

//Destruktor
ZustBest::~ZustBest(){};



void ZustBest::run(){
  /* --------------------------------
   * | Situationen des Hüpfroboters |
   * --------------------------------
   * Zustand 0: Fehler
   * Zustand 1: Sinkflug
   * Zustand 2: Landung
   * Zustand 3: Einziehen
   * Zustand 4: Scheitelpunkt Boden
   * Zustand 5: Schub
   * Zustand 6: Absprung
   * Zustand 7: Steigflug
   * Zustand 8: Scheitelpunkt Luft
   */
  
  
  //----------------------------- set Input -----------------------------
  ddzIMU	= in_ddzIMU.getSignal().getValue();
  dzIMU		= in_dzIMU.getSignal().getValue();
  enc		= in_enc.getSignal().getValue();
  
  
  
  //----------------------------- run -----------------------------
  enc_dif = enc - enc_1;
  
  /* Bestimmen der Scheitelpunkte => Geschwindigkeit um 0
   * v_grenze wird benötigt, um sicherzustellen, dass nur die Beschleunigung
   * am Scheitelpunkt verglichen wird.
   */
  if((dzIMU > -v_grenze) && (dzIMU < v_grenze)){
    //Scheitelpunkt Luft
    if(ddzIMU <= 0)       
      Zustand = 8;
        
    //Scheitelpunkt Boden
    else if(ddzIMU > 0)   
      Zustand = 4;
    
    //Error
    else 
      Zustand = 0;       
  }
  
  // Roboter bewegt sich nach unten
  else if(dzIMU  < (-v_grenze)){
    /* Unterscheiden ob am Boden oder in der Luft.
     * Mithilfe der z-Komponente des Fusspunktes wird unterschieden, ob er
     * sich am Boden oder in der Luft befindet. Der Zustand Landung
     * beschreibt den unsicheren Zustand zwischen Boden und Luft 
     * (Fusspunktberechnung). 
     */
    
    //Sinkflug
    if((enc_dif(0) > toleranzLuft) && (enc_dif(1) > toleranzLuft) && (enc_dif(2) > toleranzLuft))            
      Zustand = 1; 
    
    //Einziehen    
    else if((enc_dif(0) < toleranzBoden) && (enc_dif(1) < toleranzBoden) && (enc_dif(2) < toleranzBoden))
      Zustand = 3;
    
    //Landung
    else
      Zustand = 2;
  }
  
  //Roboter bewegt sich nach oben    
  else if(dzIMU > v_grenze){
    //Steigflug
    if((enc_dif(0) > toleranzLuft) && (enc_dif(1) > toleranzLuft) && (enc_dif(2) > toleranzLuft))   
      Zustand = 7;
    
    //Schub    
    else if((enc_dif(0) < toleranzBoden) && (enc_dif(1) < toleranzBoden) && (enc_dif(2) < toleranzBoden))
      Zustand = 5;
    
    //Absprung
    else 
      Zustand = 6; 
  }
 
  //error   
  else 
    Zustand = 0;
  
  
  enc_1 = enc;
  
  
  //----------------------------- set Output -----------------------------
  out_Zustand.getSignal().setValue(Zustand);
  
}// end run
