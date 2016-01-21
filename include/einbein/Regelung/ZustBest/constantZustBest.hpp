#ifndef EINBEIN_CONSTANTZUSTBESTHPP
#define EINBEIN_CONSTANTZUSTBESTHPP



namespace einbein{
  /* ----------------------------------------------------------
   * | Konstanten für die Zustandsbestimmung des Hüpfroboters |
   * ----------------------------------------------------------
   */
  

  //Variabeln für Zustandsbestimmung zwischen Boden und Luft (Absprung/Landung)
  static const double toleranzLuft    = 0.11;         
  static const double toleranzBoden   = 0.0007;
  static const double toleranzBodenGeschw = -0.7;
 
  
  
}


#endif // EINBEIN_CONSTANTZUSTBESTHPP