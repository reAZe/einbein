#include <einbein/Regelung/ZustBest/ZustBest.hpp>
#include <einbein/Regelung/ZustBest/constantZustBest.hpp>

//namespace
using namespace einbein;
using namespace eeros;
using namespace eeros::math;


//Konstruktor
ZustBest::ZustBest(double ts){
  Ts = ts; 
  Zustand = 8; //Anfangszustand
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
//   dzIMU		= in_dzIMU.getSignal().getValue();
  enc		= in_enc.getSignal().getValue();
  
  
  
  
  //----------------------------- run -----------------------------

  
  // Differenz des aktuellen und letzten Encoderwerts bilden
  enc_dif = enc - enc_1;
  // Geschwindigkeit des Roboter berechnen
  dzIMU = dzIMU_1 + (ddzIMU+ddzIMU_1)*Ts/2; 
  // Differenz der aktuellen und letzten Geschwindigkeit bilden
  dz_dif = dzIMU - dzIMU_1;
  // Werte übergeben
  dzIMU_1 = dzIMU;
  ddzIMU_1 = ddzIMU;
  enc_1 = enc;

  /* Bestimmen der Scheitelpunkte => Geschwindigkeit um 0
   * v_grenze wird benötigt, um sicherzustellen, dass nur die Beschleunigung
   * am Scheitelpunkt verglichen wird.
   */
//   if((dzIMU < v_grenze) && (dzIMU > -v_grenze)){
    //Scheitelpunkt Luft
    if((Zustand == 7) && (dzIMU < v_grenze) && (dzIMU > -v_grenze))
      Zustand = 8;
        
    //Scheitelpunkt Boden
    else if((Zustand == 3) && (dz_dif > toleranzLuftGeschw)){
      dzIMU_1 = 0;
      Zustand = 4;
    }     
//   }
  
  // Roboter bewegt sich nach unten
//   else if(dzIMU  < (-v_grenze)){
    /* Unterscheiden ob am Boden oder in der Luft.
     * Mithilfe der z-Komponente des Fusspunktes wird unterschieden, ob er
     * sich am Boden oder in der Luft befindet. Der Zustand Landung
     * beschreibt den unsicheren Zustand zwischen Boden und Luft 
     * (Fusspunktberechnung). 
     */
    
    //Sinkflug
    else if((Zustand == 8) && (dzIMU < -toleranzStartGeschw)) 
      Zustand = 1; 
    
    //Einziehen    
    else if(Zustand == 2)
      Zustand = 3;
    
    //Landung
    else if((Zustand == 1) && ((enc_dif(0) < -toleranzBoden) && (enc_dif(1) < -toleranzBoden) && (enc_dif(2) < -toleranzBoden)))
      Zustand = 2;
//   }
  
  //Roboter bewegt sich nach oben    
//   else if(dzIMU > v_grenze){
    //Steigflug
    else if(Zustand == 6)
      Zustand = 7;
    
    //Schub    
    else if((Zustand == 4) && (dzIMU > toleranzLuftGeschw))
      Zustand = 5;
    
    //Absprung
    else if((Zustand == 5) && ((enc_dif(0) < -toleranzLuft) && (enc_dif(1) < -toleranzLuft) && (enc_dif(2) < -toleranzLuft)))
      Zustand = 6; 

 
  
  
  //----------------------------- set Output -----------------------------
  out_Zustand.getSignal().setValue(Zustand);
  
}// end run
