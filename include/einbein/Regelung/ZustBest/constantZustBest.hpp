#ifndef EINBEIN_CONSTANTZUSTBESTHPP
#define EINBEIN_CONSTANTZUSTBESTHPP



namespace einbein{
  /* ----------------------------------------------------------
   * | Konstanten für die Zustandsbestimmung des Hüpfroboters |
   * ----------------------------------------------------------
   */
  
  
  //Geschwindigkeitsgrenze für das Bestimmen der Zustandsänderung des Scheitelpunktes
  //in der Luft und am Boden
  static const double v_grenze = 0.01;
  static const double toleranzLuftGeschw = 0.005;
  static const double toleranzBodenGeschw = 0.01;
  static const double toleranzStartGeschw = 0.02;

  //Variabeln für Zustandsbestimmung zwischen Boden und Luft (Absprung/Landung)
  static const double toleranzLuft    = 0.001;         
  static const double toleranzBoden   = 0.001;
 
  
  
}


#endif // EINBEIN_CONSTANTZUSTBESTHPP