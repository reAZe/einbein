#ifndef EINBEIN_CONSTANTZUSTBESTHPP
#define EINBEIN_CONSTANTZUSTBESTHPP



namespace einbein{
  /* ----------------------------------------------------------
   * | Konstanten für die Zustandsbestimmung des Hüpfroboters |
   * ----------------------------------------------------------
   */
  

  //Variabeln für Zustandsbestimmung zwischen Boden und Luft (Absprung/Landung)
  static const double toleranzLuft    = 0.13;         
  static const double toleranzBodenGeschw = -0.82;
  static const double toleranzScheitelBodenGeschw = 0.008;		//maximal detektierbare Geschwindigkeit bei Ts = 1ms => 4.4 mm/s
  
}


#endif // EINBEIN_CONSTANTZUSTBESTHPP