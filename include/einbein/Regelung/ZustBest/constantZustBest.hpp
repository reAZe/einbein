#ifndef EINBEIN_CONSTANTZUSTBESTHPP
#define EINBEIN_CONSTANTZUSTBESTHPP



namespace einbein{
  /* ----------------------------------------------------------
   * | Konstanten für die Zustandsbestimmung des Hüpfroboters |
   * ----------------------------------------------------------
   */
  
  
  //Geschwindigkeitsgrenze für das Bestimmen der Zustandsänderung des Scheitelpunktes
  //in der Luft und am Boden
  static const double v_grenze = 0.1;

  //Variabeln für Zustandsbestimmung zwischen Boden und Luft (Absprung/Landung)
  static const double toleranzLuft    = 0.001;         
  static const double toleranzBoden   = 0.001;
  
  
}


#endif // EINBEIN_CONSTANTZUSTBESTHPP